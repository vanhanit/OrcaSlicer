// Orca: sub-layered outer walls. The outermost wall loops of a layer are printed as a stack of
// thinner passes at ascending Z, generated from the mesh re-sliced at each sub-layer height (see
// Layer::wall_sub_slices), while the layer's inner walls and infill still print once at the full
// layer height. This file owns everything between those sub-slices and the emitted extrusions.

#include "WallSublayers.hpp"

#include "ClipperUtils.hpp"
#include "Fill/FillBase.hpp"
#include "Geometry.hpp"
#include "Layer.hpp"
#include "PerimeterGenerator.hpp"
#include "Print.hpp"
#include "Surface.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace Slic3r {

// An overhang path this short is carried by the wall on either side of it - it is a bridge whose
// anchors are closer together than the thread is thick. Printing it as an overhang costs a
// deceleration, a drop to bridging speed and a fan blast for a fraction of a millimetre of material,
// which is a visible hitch in the wall and buys nothing.
static constexpr double SUBLAYER_MIN_OVERHANG = 3.;  // x external wall width
// A bridge has to be straight, so an overhang run anchored by wall at both ends becomes the chord
// between those anchors. Past this it is the shape of the model, not a gap in it, and is left alone.
static constexpr double SUBLAYER_MAX_BRIDGE_CHORD = 8.;  // x external wall width
// Print quality, not correctness: everything a pass fills stands on material by construction, but a
// patch smaller than a few extrusion lines still comes out as a dab of a millimetre or two that
// shows on the surface and costs a travel. Area rather than width, because the ring a gently sloped
// surface needs is narrow but long.
static constexpr double SUBLAYER_MIN_SUPPORT_AREA = 12.;  // x solid infill width^2


int wall_sublayer_count(const PrintRegionConfig &config, coordf_t layer_height)
{
    if (config.wall_loops <= 0 || layer_height <= 0.)
        return 1;
    const double target = config.wall_sublayer_height.get_abs_value(layer_height);
    if (target <= 0.)
        return 1;
    int n = std::clamp((int) std::lround(layer_height / target), 1, MAX_WALL_SUBLAYERS);
    // A sub-layer thinner than this asks the extruder for an unprintable amount of material, so
    // prefer fewer, thicker passes over honoring the requested height exactly.
    while (n > 1 && layer_height / n < MIN_WALL_SUBLAYER_HEIGHT)
        -- n;
    return n;
}

// Solid fill printed inside a wall pass, on the strip the pass above it would otherwise land on with
// nothing underneath. Flow and height are the sub-layer's, not a bridging flow: a bridge thread is as
// thick as the nozzle and would stand proud of the pass.
static void append_sublayer_support_fill(ExtrusionEntityCollection &out,
                                         const ExPolygons          &areas,
                                         const Flow                &flow,
                                         float                      angle,
                                         const Layer               &layer,
                                         const PrintRegionConfig   &region_config)
{
    std::unique_ptr<Fill> filler(Fill::new_from_type(ipRectilinear));
    filler->set_bounding_box(layer.object()->bounding_box());
    filler->layer_id            = layer.id();
    filler->z                   = layer.print_z;
    filler->angle               = angle;
    filler->spacing             = flow.spacing();
    filler->link_max_length     = (coord_t) scale_(3. * filler->spacing);
    filler->print_config        = &layer.object()->print()->config();
    filler->print_object_config = &layer.object()->config();

    FillParams params;
    params.density        = 1.f;
    params.dont_adjust    = true;
    params.flow           = flow;
    params.extrusion_role = erSolidInfill;
    params.config         = &region_config;

    ExtrusionEntitiesPtr entities;
    Surface              surface(stInternalSolid, ExPolygon());
    for (const ExPolygon &expoly : areas) {
        surface.expolygon = expoly;
        // Spacing is modified in place by the filler to report its adjustment.
        filler->spacing = flow.spacing();
        filler->fill_surface_extrusion(&surface, params, entities);
    }
    out.append(std::move(entities));
}

// Tidy the overhang paths the wall generator split each loop into against the material below the
// pass: demote the ones too short to be real overhangs, and straighten the rest between their
// anchors. See SUBLAYER_MIN_OVERHANG and SUBLAYER_MAX_BRIDGE_CHORD.
static void tidy_pass_overhangs(ExtrusionEntityCollection &entities, double min_overhang, double max_chord)
{
    for (ExtrusionEntity *entity : entities.entities) {
        if (auto *nested = dynamic_cast<ExtrusionEntityCollection*>(entity)) {
            tidy_pass_overhangs(*nested, min_overhang, max_chord);
            continue;
        }
        auto *loop = dynamic_cast<ExtrusionLoop*>(entity);
        if (loop == nullptr || loop->paths.size() < 3)
            continue;
        for (ExtrusionPath &path : loop->paths)
            if (path.role() == erOverhangPerimeter && path.polyline.length() < min_overhang)
                path.set_extrusion_role(erExternalPerimeter);
        for (size_t i = 0; i < loop->paths.size(); ++ i) {
            ExtrusionPath &path = loop->paths[i];
            if (path.role() != erOverhangPerimeter || path.polyline.size() < 3)
                continue;
            // Anchored only if the wall on either side of it is standing on something.
            const ExtrusionPath &before = loop->paths[(i + loop->paths.size() - 1) % loop->paths.size()];
            const ExtrusionPath &after  = loop->paths[(i + 1) % loop->paths.size()];
            if (before.role() == erOverhangPerimeter || after.role() == erOverhangPerimeter)
                continue;
            // Keep the end points as they are - they carry the Z the path was generated with.
            const Point3 &a = path.polyline.points.front(), &b = path.polyline.points.back();
            if (Vec2d(double(b.x() - a.x()), double(b.y() - a.y())).norm() > max_chord)
                continue;
            path.polyline.points = { a, b };
        }
    }
}

// A pass may print a wall that overhangs - an outward-rolling surface lands every pass outside the
// one below and is built exactly that way - but not one with nothing under it at all. Where material
// appears part way up a layer over open space, such as the lintel above a window, the pass would jump
// to it, lay it in mid air and jump away. It is left to the layer's own pass, which prints it at
// print_z over the whole layer height, with the bridge handling that needs.
//
// An overhanging perimeter still encloses the material it is growing out from, so a closed loop is
// judged by whether the model below fills what it bounds, and an open path - a thin wall, a single
// line with nothing either side of it - by whether the model below lies under the line itself.
static bool stranded_over_air(const Polyline &q, const ExPolygons &below)
{
    if (q.points.size() > 2 && q.points.front() == q.points.back()) {
        Polygon pg(q.points);
        pg.points.pop_back();
        const double enclosed = std::abs(pg.area());
        if (enclosed <= 0.)
            return false;
        double covered = 0.;
        for (const ExPolygon &e : intersection_ex(ExPolygons{ExPolygon(pg)}, below))
            covered += e.area();
        return covered < 0.1 * enclosed;
    }
    double air = 0.;
    for (const Polyline &r : diff_pl(Polylines{q}, below))
        air += r.length();
    return air > 0.9 * q.length();
}

static void drop_airborne_islands(ExtrusionEntityCollection &entities, const ExPolygons &below)
{
    if (below.empty())
        return;
    ExtrusionEntitiesPtr kept;
    kept.reserve(entities.entities.size());
    for (ExtrusionEntity *entity : entities.entities) {
        Polylines pl;
        entity->collect_polylines(pl);
        bool stranded = ! pl.empty();
        for (const Polyline &q : pl)
            if (! stranded_over_air(q, below)) {
                stranded = false;
                break;
            }
        if (stranded)
            delete entity;
        else
            kept.emplace_back(entity);
    }
    entities.entities = std::move(kept);
}

// A void the layer's own slice does not have is one that closes part way up the layer. The sub-slices
// below it see it open, so left alone the passes trace its outline and the core carves it out of the
// layer. Neither is wanted: the layer prints across it at print_z and bridges it the way an ordinary
// layer does. Decided per hole and the kept holes copied across untouched, so that no boundary is
// reclipped: filling a hole by unioning its own footprint back in leaves hairline crescents along
// the coincident edges, which the perimeter generator then walls in.
static ExPolygons keep_layer_voids(const ExPolygons &src, const ExPolygons &layer_voids)
{
    ExPolygons kept;
    kept.reserve(src.size());
    for (const ExPolygon &expoly : src) {
        ExPolygon out(expoly.contour);
        for (const Polygon &hole : expoly.holes) {
            double covered = 0.;
            for (const ExPolygon &shared : intersection_ex(ExPolygons{ExPolygon(hole)}, layer_voids))
                covered += shared.area();
            if (covered > 0.05 * std::abs(hole.area()))
                out.holes.emplace_back(hole);
        }
        kept.emplace_back(std::move(out));
    }
    return kept;
}

WallSublayerContext wall_sublayer_prepare(const LayerRegion &layerm, const LayerRegionPtrs &compatible_regions, bool spiral_mode)
{
    WallSublayerContext      ctx;
    const Layer             *layer         = layerm.layer();
    const PrintRegionConfig &region_config = layerm.region().config();
    const size_t             num_passes    = layer->wall_sub_slices.size();

    if (num_passes == 0 || spiral_mode)
        return ctx;
    ctx.band_walls = std::clamp(region_config.wall_sublayer_loops.value, 0, region_config.wall_loops.value);
    if (ctx.band_walls == 0)
        return ctx;

    ctx.pass_slices.resize(num_passes);
    for (size_t k = 0; k < num_passes; ++ k) {
        ExPolygons expolygons;
        for (const LayerRegion *other : compatible_regions) {
            const int region_id = other->region().print_object_region_id();
            if (region_id >= 0 && size_t(region_id) < layer->wall_sub_slices[k].region_slices.size())
                append(expolygons, layer->wall_sub_slices[k].region_slices[region_id]);
        }
        ctx.pass_slices[k] = union_ex(expolygons);
    }

    // Everything the layer's own pass emits - its walls and its infill alike - is printed at print_z
    // over the full layer height, so it occupies its whole column from bottom_z upwards and is only
    // valid where the model is solid for that entire height. Against the layer's mid-height slice
    // instead it reaches into ground that is a hole at some sub-layer, and its innermost kept wall
    // comes down on a column a pass already printed. An empty pass does not constrain it, or a
    // feature ending part way up a layer would take the whole layer's content with it.
    for (const ExPolygons &pass : ctx.pass_slices) {
        if (pass.empty())
            continue;
        ctx.core_region     = ctx.core_region_set ? intersection_ex(ctx.core_region, pass) : pass;
        ctx.core_region_set = true;
    }
    if (! ctx.core_region_set)
        return ctx;

    // The bottom text of a 3DBenchy is a recess a fraction of a layer deep. Every pass below its
    // ceiling sees the characters as holes, so the passes were drawing the outline of each one and
    // the core was leaving a matching hole in a layer whose own slice is solid there - the layer
    // bridges the recess, and can only do so if it is given the material to bridge with.
    ExPolygons layer_voids;
    for (const LayerRegion *other : compatible_regions)
        for (const Surface &surface : other->slices.surfaces)
            for (const Polygon &hole : surface.expolygon.holes) {
                layer_voids.emplace_back(hole);
                layer_voids.back().contour.reverse();
            }
    for (ExPolygons &pass : ctx.pass_slices)
        if (! pass.empty())
            pass = keep_layer_voids(pass, layer_voids);
    ctx.core_region = keep_layer_voids(ctx.core_region, layer_voids);
    ctx.core_base   = ctx.core_region;

    // A pass extrudes over air only as wall - a wall may overhang, and the generator classifies it as
    // one. Anything else it would have to fill over air is handed back to the layer's own pass, which
    // prints at print_z over the whole layer height and so bridges it the way an ordinary layer does:
    // with a bridge flow, bridge speed and the fan on. A pass cannot do that itself, because a bridge
    // is a thread as thick as the nozzle and would stand proud of a pass a fraction of that height.
    //
    // The wall band is printed everywhere the pass has material, so it is ground for the pass above
    // whether or not it overhangs. Approximated here from the sub-slice, because the real band is not
    // known until the generator has run and the core has to be settled before that.
    const coord_t band_width = coord_t(ctx.band_walls * layerm.flow(frExternalPerimeter, layer->height).scaled_width());
    const ExPolygons *below  = layer->wall_sublayer_support(0);
    ExPolygons        ground = below == nullptr ? ExPolygons() : *below;
    ExPolygons        stranded;
    for (size_t k = 0; k < num_passes; ++ k) {
        if (ctx.pass_slices[k].empty()) {
            ground.clear();
            continue;
        }
        const ExPolygons band     = diff_ex(ctx.pass_slices[k], offset_ex(ctx.pass_slices[k], - float(band_width)));
        const ExPolygons fillable = diff_ex(diff_ex(ctx.pass_slices[k], ctx.core_region), band);
        append(stranded, diff_ex(fillable, ground));

        ExPolygons printed = band;
        append(printed, intersection_ex(fillable, ground));
        ground = union_ex(printed);
    }
    if (! stranded.empty()) {
        append(stranded, ctx.core_region);
        ctx.core_region = union_ex(stranded);
    }
    return ctx;
}

void wall_sublayer_generate(LayerRegion               &layerm,
                            const LayerRegionPtrs     &compatible_regions,
                            const WallSublayerContext &ctx,
                            bool                       arachne,
                            bool                       spiral_mode,
                            double                     model_rotation_rad)
{
    const Layer             *layer         = layerm.layer();
    const PrintRegionConfig &region_config = layerm.region().config();
    const PrintObjectConfig &object_config = layer->object()->config();
    const PrintConfig       &print_config  = layer->object()->print()->config();
    const int                region_id     = layerm.region().print_object_region_id();
    const size_t             num_passes    = layer->wall_sub_slices.size();

    // One wall generator run per sub-layer, each fed the mesh re-sliced at that sub-layer's height.
    // Overhangs and bridges are classified against the sub-layer below, so a wall supported by the
    // pass beneath it is no longer treated as an overhang of the whole layer.
    layerm.sublayer_perimeters.resize(num_passes);
    // The footprint of each pass's wall band. The band of pass k+1 is laid on what pass k printed, so
    // this drives the support fill below.
    std::vector<ExPolygons> pass_band(num_passes);
    for (size_t k = 0; k < num_passes; ++ k) {
        const WallSubSlice &sub = layer->wall_sub_slices[k];
        if (ctx.pass_slices[k].empty())
            continue;

        SurfaceCollection band_slices;
        band_slices.append(offset_ex(ctx.pass_slices[k], ClipperSafetyOffset), stInternal);

        // The fill surfaces and the extra perimeters belong to the layer's own pass, which sees the
        // whole layer. The gap fill is kept and appended after the walls of this same sub-layer; the
        // no-overlap area is kept because it is the area left inside this pass's wall band.
        ExtrusionEntityCollection band_gap_fill;
        SurfaceCollection         band_fill_surfaces;
        ExPolygons                band_fill_no_overlap;

        PerimeterGenerator bg(
            &band_slices, &compatible_regions, sub.height, sub.slice_z, layerm.flow(frPerimeter, sub.height),
            &region_config, &object_config, &print_config, spiral_mode, model_rotation_rad,
            &layerm.sublayer_perimeters[k], &band_gap_fill, &band_fill_surfaces, &band_fill_no_overlap);

        bg.lower_slices = layer->wall_sublayer_support(k);
        if (layer->upper_layer != nullptr) {
            bg.upper_slices             = &layer->upper_layer->lslices;
            bg.upper_slices_same_region = &layer->upper_layer->get_region(region_id)->slices;
        }
        bg.layer_id            = (int) layer->id();
        bg.sublayer_band_walls = ctx.band_walls;
        bg.ext_perimeter_flow  = layerm.flow(frExternalPerimeter, sub.height);
        // Not a bridge flow: that is a thread as thick as the nozzle, which inside a pass a fraction
        // of a layer high would stand proud of the pass and the one above would plough through it.
        // The overhang role is kept, so these still print at bridge speed with the part cooling fan.
        bg.overhang_flow     = layerm.flow(frPerimeter, sub.height);
        bg.solid_infill_flow = layerm.flow(frSolidInfill, sub.height);

        if (arachne)
            bg.process_arachne();
        else
            bg.process_classic();

        const double ext_width = double(bg.ext_perimeter_flow.scaled_width());
        tidy_pass_overhangs(layerm.sublayer_perimeters[k], SUBLAYER_MIN_OVERHANG * ext_width,
                            SUBLAYER_MAX_BRIDGE_CHORD * ext_width);

        layerm.sublayer_perimeters[k].append(std::move(band_gap_fill.entities));

        pass_band[k] = diff_ex(ctx.pass_slices[k], union_ex(band_fill_no_overlap));
    }

    // Everything the passes own but their walls do not cover, the pass fills itself: the layer's own
    // pass is confined to the columns solid for the whole layer, so nothing else reaches this ground,
    // and it is what the wall band of every pass above comes down on. wall_sublayer_prepare() has
    // already handed anything a pass would have to fill over air back to the layer's own pass, so
    // what is left here stands on material by construction.
    const Flow  support_flow  = layerm.flow(frSolidInfill, layer->wall_sub_slices.front().height);
    const float support_angle = float(Geometry::deg2rad(region_config.solid_infill_direction.value) + model_rotation_rad);
    const float support_width = float(support_flow.scaled_width());
    const double min_area     = SUBLAYER_MIN_SUPPORT_AREA * double(support_width) * double(support_width);

    const ExPolygons *below  = layer->wall_sublayer_support(0);
    ExPolygons        ground = below == nullptr ? ExPolygons() : *below;
    for (size_t k = 0; k < num_passes; ++ k) {
        if (ctx.pass_slices[k].empty()) {
            ground.clear();
            continue;
        }
        // Measured against the model at the sub-layer below, not against what was printed there: a
        // thin wall needs something under it, and material merely beside it does not hold it up.
        const ExPolygons *below = layer->wall_sublayer_support(k);
        drop_airborne_islands(layerm.sublayer_perimeters[k], below == nullptr ? ExPolygons() : *below);

        // The tread: solid at this pass, outside the column the layer's own pass may occupy, not
        // already covered by this pass's walls, and standing on what the pass below put down.
        // Repeating it pass after pass is how the staircase a sloped surface makes is filled up to
        // print_z.
        const ExPolygons tread = intersection_ex(diff_ex(diff_ex(ctx.pass_slices[k], ctx.core_region), pass_band[k]), ground);

        // Ground for the pass above is the whole tread, before the quality filter below drops the
        // dabs, so that dropping one does not punch a hole and fragment every pass above it, and only
        // the band that is actually carried - the span of a lintel dropped above holds nothing up.
        ExPolygons printed = pass_band[k];
        append(printed, tread);
        ground = union_ex(printed);

        ExPolygons fill = opening_ex(union_ex(tread), support_width / 2.f);
        fill.erase(std::remove_if(fill.begin(), fill.end(),
                                  [min_area](const ExPolygon &e) { return e.area() < min_area; }),
                   fill.end());
        if (! fill.empty())
            append_sublayer_support_fill(layerm.sublayer_perimeters[k], fill, support_flow,
                                         support_angle + float((k % 2) * M_PI / 2.), *layer, region_config);
    }
}

} // namespace Slic3r
