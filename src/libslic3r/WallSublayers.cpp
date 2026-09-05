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
#include "VariableWidth.hpp"

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
// A gap fill run shorter than this is a dab: it holds nothing together and costs a travel, a stop and
// a start, which on a curved wall is repeated thousands of times over.
static constexpr double SUBLAYER_MIN_GAP_FILL = 2.;  // x solid infill width
// How far beside or under a wall the model below has to reach for the wall to be carried by it. On a
// surface rolling outward - a hull flaring out of the bilge, a gunwale, the lower lip of a hole -
// every pass lands outside the one below by design, and a wall's width of reach is what lets that
// surface keep its sub-layer resolution instead of falling back to one step per layer.
static constexpr double SUBLAYER_ANCHOR_REACH = 1.0;  // x sub-layer wall width
// Inside a void that is closing over - the ceiling of a cavity, the lip of a countersink narrowing
// with Z - that reach is far too generous: a ring stepping half a millimetre inward per pass sits
// with a few hundredths of its width on the ring below and the rest over the hole. Nothing is gained
// by printing it either, because the layer's own pass bridges that void at print_z with a bridge flow
// and the fan on, and these threads only droop into its way. Here a wall has to genuinely rest on
// what is below it: a quarter of its width, no further out than that.
static constexpr double SUBLAYER_VOID_ANCHOR_REACH = 0.25;  // x sub-layer wall width

// The passes are only a fraction of a layer tall but would otherwise keep the configured wall width,
// leaving the extrusion several times wider than it is tall - a shape that is hard to lay down evenly
// and shows up as a rippled surface. wall_sublayer_line_width narrows them back; 0 keeps the width the
// region uses for that role.
static Flow sublayer_flow(const LayerRegion &layerm, FlowRole role, coordf_t height)
{
    const Flow   base  = layerm.flow(role, height);
    const double width = layerm.region().config().wall_sublayer_line_width.get_abs_value(base.nozzle_diameter());
    return width > 0. ? base.with_width(float(width)) : base;
}


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
    // What a pass has to fill is the tread of the staircase a sloped surface makes: a long narrow ring
    // following the wall. Rectilinear crosses it in short lines with a travel between each, laid at a
    // right angle to the pass below so that consecutive passes bond - an alternation that repeats at
    // the height of one layer. Concentric follows the ring instead, in one loop per line, with no
    // direction to alternate, which on a gently sloped surface may come out smoother. Which of the two
    // suits a given model is not something the slicer can tell, so it is left to the user.
    const bool concentric = region_config.wall_sublayer_fill_pattern.value == ipConcentric;

    std::unique_ptr<Fill> filler(Fill::new_from_type(concentric ? ipConcentric : ipRectilinear));
    filler->set_bounding_box(layer.object()->bounding_box());
    filler->layer_id            = layer.id();
    filler->z                   = layer.print_z;
    filler->angle               = angle;
    filler->spacing             = flow.spacing();
    if (! concentric)
        filler->link_max_length = (coord_t) scale_(3. * filler->spacing);
    filler->print_config        = &layer.object()->print()->config();
    filler->print_object_config = &layer.object()->config();

    FillParams params;
    params.density        = 1.f;
    // Concentric closes its spacing over the real width of the ring, which a tread only a few lines
    // wide needs; rectilinear is held at the nominal spacing, as it was.
    params.dont_adjust    = ! concentric;
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

// The remainder of a tread, too narrow for a fill line to sit in. A medial axis gives it a centreline
// and a width that varies along it, so a ring a third of a line wide is printed a third of a line wide
// instead of being dropped or over-extruded.
static void append_sublayer_gap_fill(ExtrusionEntityCollection &out,
                                     const ExPolygons          &areas,
                                     const Flow                &flow,
                                     const PrintRegionConfig   &region_config)
{
    if (areas.empty())
        return;
    const double   min = 0.2 * flow.scaled_width() * (1 - INSET_OVERLAP_TOLERANCE);
    const double   max = 2. * flow.scaled_spacing();
    ThickPolylines polylines;
    for (ExPolygon &ex : diff_ex(opening_ex(areas, float(min / 2.)),
                                 offset2_ex(areas, - float(max / 2.), float(max / 2. + ClipperSafetyOffset))))
        ex.medial_axis(min, max, &polylines);

    // A gap this short carries a few hundredths of a cubic millimetre and costs a travel, a stop and a
    // start to place it - on a curved wall the strip shatters into thousands of them against the jagged
    // edge of what the pass below printed, and every one of those is somewhere a blob can form. The
    // user's own filter still applies on top where it is set higher.
    const double min_length = std::max(SUBLAYER_MIN_GAP_FILL * double(flow.scaled_width()),
                                       scale_(region_config.filter_out_gap_fill.value));
    polylines.erase(std::remove_if(polylines.begin(), polylines.end(),
                                   [min_length](const ThickPolyline &p) { return p.length() < min_length; }),
                    polylines.end());
    if (! polylines.empty())
        variable_width(polylines, erGapFill, flow, out.entities);
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
        // Back to the role the rest of the loop carries, not to an external one: an inner wall whose
        // overhang runs are demoted would otherwise print half of itself at the outer wall's speed
        // and acceleration, alternating between the two every few tenths of a millimetre. A loop that
        // is overhang end to end has no role to fall back on and no hitch to remove.
        ExtrusionRole base = erNone;
        for (const ExtrusionPath &path : loop->paths)
            if (path.role() != erOverhangPerimeter) {
                base = path.role();
                break;
            }
        if (base != erNone)
            for (ExtrusionPath &path : loop->paths)
                if (path.role() == erOverhangPerimeter && path.polyline.length() < min_overhang)
                    path.set_extrusion_role(base);
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
// Whether a wall has nothing to hold on to: the model below reaches neither under the thread nor up
// against it. Judged against the model below grown by a wall's width, because a wall that steps
// outward over the one beneath it is still carried by it - which is what an overhanging wall is.
//
// This used to ask, for a closed loop, how much of the area the loop bounds the model below fills.
// That is the same question only for a loop drawn around solid material. Around a hollow shell - the
// wall of a sphere, a vase, a cup - what a loop bounds is mostly the void it encircles, so a ring
// standing squarely on the ring below it read as 8% supported and was thrown away, taking the outer
// wall of every pass with it over a third of the height of an ellipsoid shell.
static bool stranded_over_air(const Polyline &q, const ExPolygons &below)
{
    double air = 0.;
    for (const Polyline &r : diff_pl(Polylines{q}, below))
        air += r.length();
    return air > 0.9 * q.length();
}

// Per wall, not per island. An island holds the loops around its contour and the loops around each of
// its holes, and they do not stand or fall together: where a cavity closes over - the crown of a dome,
// the inside of a shell - the loop around the closing hole hangs in mid air while the loop around the
// contour beside it is carried by the wall below. Judged as one island the airborne loop rode along
// with its supported neighbour and was printed over nothing.
static void drop_airborne_walls(ExtrusionEntityCollection &entities, const ExPolygons &anchored, Polygons &dropped)
{
    ExtrusionEntitiesPtr kept;
    kept.reserve(entities.entities.size());
    for (ExtrusionEntity *entity : entities.entities) {
        if (auto *nested = dynamic_cast<ExtrusionEntityCollection*>(entity)) {
            drop_airborne_walls(*nested, anchored, dropped);
            if (nested->empty())
                delete entity;
            else
                kept.emplace_back(entity);
            continue;
        }
        Polylines pl;
        entity->collect_polylines(pl);
        bool stranded = ! pl.empty();
        for (const Polyline &q : pl)
            if (! stranded_over_air(q, anchored)) {
                stranded = false;
                break;
            }
        if (stranded) {
            entity->polygons_covered_by_width(dropped, 10.f);
            delete entity;
        } else
            kept.emplace_back(entity);
    }
    entities.entities = std::move(kept);
}

// The voids the model below encloses, as areas: the holes of an ExPolygon, taken on their own.
static ExPolygons enclosed_voids(const ExPolygons &src)
{
    ExPolygons out;
    for (const ExPolygon &expoly : src)
        for (const Polygon &hole : expoly.holes) {
            out.emplace_back(hole);
            out.back().contour.reverse();
        }
    return out;
}

// What the walls that were dropped would have covered, so that the passes above do not come down on
// a wall that was never printed.
static ExPolygons drop_airborne_islands(ExtrusionEntityCollection &entities, const ExPolygons &below,
                                        float reach, float void_reach)
{
    if (below.empty())
        return {};
    // The generous reach applies where the wall hangs over open space beside the model. Over a void
    // the model below encloses it does not: there only a wall that still rests on the material around
    // the void is carried by it. See SUBLAYER_ANCHOR_REACH and SUBLAYER_VOID_ANCHOR_REACH.
    ExPolygons anchored = reach > 0.f ? offset_ex(below, reach) : below;
    if (void_reach < reach) {
        // What stands inside a void is not part of it: a box hollowed out around a pillar has the
        // pillar inside its hole, and the pillar carries its own walls.
        const ExPolygons voids = diff_ex(enclosed_voids(below), below);
        anchored = diff_ex(anchored, offset_ex(voids, - void_reach));
    }
    Polygons dropped;
    drop_airborne_walls(entities, anchored, dropped);
    return union_ex(dropped);
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
    // The layer's own run drops band_walls of its own loops at the region's full width, so the band has
    // to cover that same strip with however many of its own - possibly narrower - loops that takes.
    const double normal_width = layerm.flow(frExternalPerimeter, layer->height).width();
    const double band_width_1 = sublayer_flow(layerm, frExternalPerimeter, layer->height).width();
    ctx.band_loops = band_width_1 > 0. ? std::max(1, (int) std::lround(ctx.band_walls * normal_width / band_width_1))
                                       : ctx.band_walls;

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
    const coord_t band_width = coord_t(ctx.band_loops * sublayer_flow(layerm, frExternalPerimeter, layer->height).scaled_width());
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

    // Where the walls the layer keeps for itself begin: the passes have to reach this, or the outer
    // wall stack stands free of the rest of the layer with a void between them.
    const coord_t core_strip = coord_t(ctx.band_walls * layerm.flow(frExternalPerimeter, layer->height).scaled_spacing());
    // Only where the layer's own run can actually lay a wall. On a feature too thin to hold one - a
    // railing, a mast, the wall of a chimney - every loop it would have made is one the band took over,
    // so it prints nothing there and what is left in the middle is not interior at all. Standing the
    // band off it would leave that strip a void down the centre of the feature, which is the feature
    // splitting into two walls with a gap between them.
    const float core_min = float(layerm.flow(frPerimeter, layer->height).scaled_width());
    const ExPolygons core_interior = opening_ex(offset_ex(ctx.core_region, - float(core_strip)), core_min / 2.f);
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
            &band_slices, &compatible_regions, sub.height, sub.slice_z, sublayer_flow(layerm, frPerimeter, sub.height),
            &region_config, &object_config, &print_config, spiral_mode, model_rotation_rad,
            &layerm.sublayer_perimeters[k], &band_gap_fill, &band_fill_surfaces, &band_fill_no_overlap);

        bg.lower_slices = layer->wall_sublayer_support(k);
        if (layer->upper_layer != nullptr) {
            bg.upper_slices             = &layer->upper_layer->lslices;
            bg.upper_slices_same_region = &layer->upper_layer->get_region(region_id)->slices;
        }
        bg.layer_id            = (int) layer->id();
        bg.sublayer_band_walls = ctx.band_loops;
        bg.ext_perimeter_flow  = sublayer_flow(layerm, frExternalPerimeter, sub.height);
        // Not a bridge flow: that is a thread as thick as the nozzle, which inside a pass a fraction
        // of a layer high would stand proud of the pass and the one above would plough through it.
        // The overhang role is kept, so these still print at bridge speed with the part cooling fan.
        bg.overhang_flow     = sublayer_flow(layerm, frPerimeter, sub.height);
        bg.solid_infill_flow = sublayer_flow(layerm, frSolidInfill, sub.height);

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
    const Flow   support_flow  = sublayer_flow(layerm, frSolidInfill, layer->wall_sub_slices.front().height);
    const float  support_angle = float(Geometry::deg2rad(region_config.solid_infill_direction.value) + model_rotation_rad);
    const float  support_width = float(support_flow.scaled_width());
    const double min_area      = SUBLAYER_MIN_SUPPORT_AREA * double(support_width) * double(support_width);
    // A concentric fill has no direction, so there is nothing to turn between passes.
    const float  pass_turn     = region_config.wall_sublayer_fill_pattern.value == ipConcentric ? 0.f : float(M_PI / 2.);

    const ExPolygons *below  = layer->wall_sublayer_support(0);
    ExPolygons        ground = below == nullptr ? ExPolygons() : *below;
    ExPolygons        unprinted;
    for (size_t k = 0; k < num_passes; ++ k) {
        if (ctx.pass_slices[k].empty()) {
            ground.clear();
            unprinted.clear();
            continue;
        }
        // Measured against the model at the sub-layer below, less whatever the pass below was not
        // allowed to print there: a thin wall needs something under it, and material merely beside it
        // does not hold it up. Without the correction the ceiling of a closing cavity comes out as a
        // stair of rings, each one standing on the ring the pass below had dropped as airborne - the
        // model says every step is carried, and every step is in fact hanging in the air behind the
        // one before it. The whole ceiling is then left to the layer above, which bridges it in one
        // pass at print_z from the material the layer below ended with.
        const ExPolygons *below      = layer->wall_sublayer_support(k);
        const float       wall_width = float(sublayer_flow(layerm, frExternalPerimeter, layer->height).scaled_width());
        ExPolygons        support    = below == nullptr ? ExPolygons() : *below;
        if (! unprinted.empty() && ! support.empty())
            support = diff_ex(support, unprinted);
        const ExPolygons  dropped    = drop_airborne_islands(layerm.sublayer_perimeters[k], support,
                                                             float(SUBLAYER_ANCHOR_REACH * wall_width),
                                                             float(SUBLAYER_VOID_ANCHOR_REACH * wall_width));
        unprinted = dropped;
        // A dropped wall is not ground. Left in, the pass above would fill the ceiling of a closing
        // cavity pass by pass on the strength of a band that was never laid down.
        if (! dropped.empty())
            pass_band[k] = diff_ex(pass_band[k], dropped);

        // Everything this pass owes and has not covered: solid at this pass, short of the walls the
        // layer keeps for itself, not already under this pass's own walls, and standing on what the
        // pass below put down.
        // Where a wall was refused, nothing else is laid either: filling behind an airborne wall only
        // moves the problem from the wall to the fill.
        ExPolygons bare = intersection_ex(diff_ex(diff_ex(ctx.pass_slices[k], core_interior), pass_band[k]), ground);
        if (! dropped.empty())
            bare = diff_ex(bare, dropped);

        // The tread, outside the column the layer's own pass may occupy: the staircase a sloped
        // surface makes, filled pass after pass up to print_z.
        const ExPolygons tread = diff_ex(bare, ctx.core_region);

        // And the strip between the band and the walls the layer keeps, which is only ever a fraction
        // of a line wide: the flare over one sub-layer, plus whatever a wall_sublayer_line_width that
        // does not divide the strip evenly leaves over. Gap fill, never solid fill - a solid line
        // would either be dropped as too narrow or laid down at full width into a third of a line of
        // room. No loop count can span it, because it is a fraction of a line and it varies with Z.
        const ExPolygons join = intersection_ex(bare, ctx.core_region);

        // Ground for the pass above is the whole tread, before the quality filter below drops the
        // dabs, so that dropping one does not punch a hole and fragment every pass above it, and only
        // the band that is actually carried - the span of a lintel dropped above holds nothing up.
        ExPolygons printed = pass_band[k];
        append(printed, tread);
        append(printed, join);
        ground = union_ex(printed);

        ExPolygons fill = opening_ex(union_ex(tread), support_width / 2.f);
        fill.erase(std::remove_if(fill.begin(), fill.end(),
                                  [min_area](const ExPolygon &e) { return e.area() < min_area; }),
                   fill.end());
        if (! fill.empty())
            append_sublayer_support_fill(layerm.sublayer_perimeters[k], fill, support_flow,
                                         support_angle + float(k % 2) * pass_turn, *layer, region_config);

        // The join, plus whatever of the tread was too narrow for a fill line to sit in. Solid fill
        // either drops those or lays a full-width bead into them; gap fill gives them a medial axis and
        // a width that follows the room there actually is.
        ExPolygons thin = diff_ex(union_ex(tread), fill);
        append(thin, join);
        append_sublayer_gap_fill(layerm.sublayer_perimeters[k], union_ex(thin), support_flow, region_config);
    }
}

} // namespace Slic3r
