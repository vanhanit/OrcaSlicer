#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MeshBoolean.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// The layer at this Z is the last one of the base, so its top surface is the ledge.
const double ledge_z = 5.0;

// The first layer, at initial_layer_print_height.
const double first_layer_z = 0.2;

// TestMesh::step scaled 3x in X/Y: a 60x60x5 base carrying a 54x54 column up to z=10, leaving a 3mm
// top ledge around a feature that keeps rising. That is the geometry both only_one_wall_top and the
// top surface expansion act on. The ledge has to stay wider than the wall band plus two top-infill
// lines, or the expansion discards it as a sliver and the tests below assert nothing.
TriangleMesh step_with_ledge()
{
    TriangleMesh m = Slic3r::Test::mesh(TestMesh::step);
    m.scale(Vec3f(3.f, 3.f, 1.f));
    return m;
}

// Every setting the assertions depend on, so none of them rests on a default.
DynamicPrintConfig base_config(const char *wall_generator)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "wall_generator",             wall_generator },
        { "layer_height",               0.2 },  // puts a layer boundary exactly on ledge_z
        { "initial_layer_print_height", 0.2 },
        { "wall_loops",                 3 },
        { "sparse_infill_density",      "15%" },
        { "top_shell_layers",           3 },
        { "bottom_shell_layers",        3 },
        { "top_surface_density",        "100%" },
        { "top_surface_expansion",      0.0 },
        { "only_one_wall_top",          false },
        { "only_one_wall_first_layer",  false },
        // Do not let the one-wall threshold discard the 3mm ledge before the feature sees it.
        { "min_width_top_surface",      0.0 },
    });
    return config;
}

double collection_length(const ExtrusionEntityCollection &coll)
{
    double len = 0.;
    for (const ExtrusionEntity *entity : coll.flatten().entities)
        if (! entity->is_collection())
            len += entity->length();
    return len;
}

// Extruded length per layer. Two slices are compared through this rather than through their G-code,
// because the G-code carries a config block that differs whenever any setting differs.
struct SliceLengths {
    std::vector<double> perimeters;
    std::vector<double> fills;
};

SliceLengths slice_lengths(const Print &print)
{
    SliceLengths out;
    for (const Layer *layer : print.objects().front()->layers()) {
        double perimeters = 0., fills = 0.;
        for (const LayerRegion *region : layer->regions()) {
            perimeters += collection_length(region->perimeters);
            fills      += collection_length(region->fills);
        }
        out.perimeters.push_back(perimeters);
        out.fills.push_back(fills);
    }
    return out;
}

double perimeter_length_at(const Print &print, double print_z)
{
    for (const Layer *layer : print.objects().front()->layers())
        if (std::abs(layer->print_z - print_z) < 1e-4) {
            double len = 0.;
            for (const LayerRegion *region : layer->regions())
                len += collection_length(region->perimeters);
            return len;
        }
    return 0.;
}

// Largest per-layer difference between two series; a negative result means they are not comparable.
double max_difference(const std::vector<double> &a, const std::vector<double> &b)
{
    if (a.size() != b.size() || a.empty())
        return -1.;
    double worst = 0.;
    for (size_t i = 0; i < a.size(); ++ i)
        worst = std::max(worst, std::abs(a[i] - b[i]));
    return worst;
}

} // namespace

// The expansion only retypes area as top solid infill, so it can do nothing where there is no top
// fill to begin with: zero top shell layers retypes the top surfaces as internal, and a top surface
// density of 0% leaves the top layer with walls only. The last section is the control - the same
// expansion on the same model does change the slice once a top fill exists - without which the two
// equality checks above it would hold for an unrelated reason.
TEST_CASE("Top surface expansion only acts where there is a top fill", "[Perimeters]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    CAPTURE(wall_generator);

    auto lengths_for = [wall_generator](int top_shell_layers, const char *top_surface_density, double expansion) {
        DynamicPrintConfig config = base_config(wall_generator);
        config.set_deserialize_strict({
            { "top_shell_layers",      top_shell_layers },
            { "top_surface_density",   top_surface_density },
            { "top_surface_expansion", expansion },
        });
        Print print;
        init_and_process_print({ step_with_ledge() }, print, config);
        REQUIRE_FALSE(print.objects().empty());
        return slice_lengths(print);
    };

    SECTION("no top shell layers") {
        const SliceLengths off = lengths_for(0, "100%", 0.0);
        const SliceLengths on  = lengths_for(0, "100%", 2.0);
        REQUIRE(off.perimeters.size() == on.perimeters.size());
        CHECK_THAT(max_difference(off.perimeters, on.perimeters), Catch::Matchers::WithinAbs(0., 1.0));
        CHECK_THAT(max_difference(off.fills,      on.fills),      Catch::Matchers::WithinAbs(0., 1.0));
    }

    SECTION("zero top surface density") {
        const SliceLengths off = lengths_for(3, "0%", 0.0);
        const SliceLengths on  = lengths_for(3, "0%", 2.0);
        REQUIRE(off.perimeters.size() == on.perimeters.size());
        CHECK_THAT(max_difference(off.perimeters, on.perimeters), Catch::Matchers::WithinAbs(0., 1.0));
        CHECK_THAT(max_difference(off.fills,      on.fills),      Catch::Matchers::WithinAbs(0., 1.0));
    }

    SECTION("with a top fill the same expansion does change the slice") {
        const SliceLengths off = lengths_for(3, "100%", 0.0);
        const SliceLengths on  = lengths_for(3, "100%", 2.0);
        REQUIRE(off.fills.size() == on.fills.size());
        CHECK(max_difference(off.fills, on.fills) > scale_(0.5));
    }
}

// With no top shell the top surfaces are retyped as internal, so the top surface density has nothing
// left to control: there is no top fill, and only_one_wall_top - the one route from the density to the
// perimeters - is itself switched off for want of a top surface to act on.
TEST_CASE("Top surface density does not affect a slice without a top shell", "[Perimeters]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    CAPTURE(wall_generator);

    auto lengths_for = [wall_generator](const char *top_surface_density) {
        DynamicPrintConfig config = base_config(wall_generator);
        config.set_deserialize_strict({
            { "top_shell_layers",    0 },
            { "only_one_wall_top",   true },
            { "top_surface_density", top_surface_density },
        });
        Print print;
        init_and_process_print({ step_with_ledge() }, print, config);
        REQUIRE_FALSE(print.objects().empty());
        return slice_lengths(print);
    };

    const SliceLengths solid = lengths_for("100%");
    const SliceLengths none  = lengths_for("0%");
    REQUIRE(solid.perimeters.size() == none.perimeters.size());
    CHECK_THAT(max_difference(solid.perimeters, none.perimeters), Catch::Matchers::WithinAbs(0., 1.0));
    CHECK_THAT(max_difference(solid.fills,      none.fills),      Catch::Matchers::WithinAbs(0., 1.0));
}

// On the ledge layer the inner walls are given up to the top fill, so that layer loses wall length.
// The handover needs a top fill that reaches the freed space: at a top surface density of 0% there is
// no top fill at all, and without top_surface_expansion the fill never grows over the walls. Either
// way the feature still runs, through the original generation, which keeps the inner walls up to the
// top boundary - putting that layer back between the plain and the one-wall slice.
TEST_CASE("Only one wall on top surfaces drops inner walls only where a top fill replaces them", "[Perimeters]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    CAPTURE(wall_generator);

    auto ledge_perimeters_for = [wall_generator](bool only_one_wall_top, const char *top_surface_density, double expansion) {
        DynamicPrintConfig config = base_config(wall_generator);
        config.set_deserialize_strict({
            { "only_one_wall_top",     only_one_wall_top },
            { "top_surface_density",   top_surface_density },
            { "top_surface_expansion", expansion },
        });
        Print print;
        init_and_process_print({ step_with_ledge() }, print, config);
        REQUIRE_FALSE(print.objects().empty());
        return perimeter_length_at(print, ledge_z);
    };

    const double plain              = ledge_perimeters_for(false, "100%", 2.0);
    const double one_wall           = ledge_perimeters_for(true,  "100%", 2.0);
    const double one_wall_no_fill   = ledge_perimeters_for(true,  "0%",   2.0);
    const double one_wall_no_expand = ledge_perimeters_for(true,  "100%", 0.0);

    REQUIRE(plain > 0.);
    CHECK(one_wall < plain);
    // Both fall back to the original generation, which cuts the walls back to the top boundary but not past it.
    CHECK(one_wall_no_fill > one_wall);
    CHECK(one_wall_no_fill < plain);
    CHECK(one_wall_no_expand > one_wall);
    CHECK(one_wall_no_expand < plain);
}

// The bottom counterpart: the first layer is thinned to a single wall only where a bottom shell fills the
// space behind it. With no bottom shell layers the bottom surfaces are retyped as internal, so that wall
// would ring sparse infill on the bed - the option is switched off instead, and the GUI hides it in that
// state so a profile that left it enabled cannot act behind a hidden checkbox.
TEST_CASE("Only one wall on the first layer needs a bottom shell", "[Perimeters]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    CAPTURE(wall_generator);

    auto first_layer_perimeters_for = [wall_generator](bool only_one_wall_first_layer, int bottom_shell_layers) {
        DynamicPrintConfig config = base_config(wall_generator);
        config.set_deserialize_strict({
            { "only_one_wall_first_layer", only_one_wall_first_layer },
            { "bottom_shell_layers",       bottom_shell_layers },
        });
        Print print;
        init_and_process_print({ step_with_ledge() }, print, config);
        REQUIRE_FALSE(print.objects().empty());
        return perimeter_length_at(print, first_layer_z);
    };

    const double plain             = first_layer_perimeters_for(false, 3);
    const double one_wall          = first_layer_perimeters_for(true,  3);
    // Both at zero bottom shell layers, so everything else that setting changes cancels out between them.
    const double plain_no_shell    = first_layer_perimeters_for(false, 0);
    const double one_wall_no_shell = first_layer_perimeters_for(true,  0);

    REQUIRE(plain > 0.);
    CHECK(one_wall < plain);
    // No bottom shell: the option is inert, down to the same walls an unchecked box gives.
    CHECK_THAT(one_wall_no_shell, Catch::Matchers::WithinAbs(plain_no_shell, 1.0));
}

// A cavity whose ceiling closes over: a cone standing inside a block, subtracted from it, so each
// layer's slice has a hole a couple of millimetres narrower than the layer below. The walls around
// that hole have nothing at all under them - a ring drawn in mid air - which is what the option is
// for. The same shape a dome, a countersink or the crown of a hollow sphere makes.
TriangleMesh block_with_closing_cavity(double cone_height = 2.)
{
    TriangleMesh block = make_cube(34., 34., 8.);
    TriangleMesh cone  = make_cone(14., cone_height);
    cone.translate(17., 17., 3.);
    MeshBoolean::cgal::minus(block, cone);
    return block;
}

// Extruded wall length whose thread has no part of itself over the layer below.
double airborne_wall_length(const Print &print)
{
    double len = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->lower_layer == nullptr)
            continue;
        const Polygons below = to_polygons(layer->lower_layer->lslices);
        for (const LayerRegion *region : layer->regions())
            for (const ExtrusionEntity *entity : region->perimeters.flatten().entities) {
                if (entity->is_collection())
                    continue;
                const Polygons covered = entity->polygons_covered_by_width(10.f);
                if (! covered.empty() && intersection(covered, below).empty())
                    len += unscale<double>(entity->length());
            }
    }
    return len;
}

// Length of extrusion that nothing holds: a closed loop with no part of itself over material already
// there, or an open line with an end in mid air. Material already there is the layer below plus what
// this layer laid down before it. A straight bridge line across a hole is held at both ends and does
// not count; the same line drawn out into a hole that is still closing over has its far end over
// nothing and does, and so does a wall drawn round that hole. A ring worked inward from the rim is a
// closed loop lying on the ring before it. Only layers with something over air at all are walked.
double unheld_extrusion_length(const Print &print)
{
    double len = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->lower_layer == nullptr || diff_ex(layer->lslices, layer->lower_layer->lslices).empty())
            continue;
        for (const LayerRegion *region : layer->regions()) {
            const coord_t reach = region->flow(frExternalPerimeter).scaled_width();
            ExPolygons    laid  = layer->lower_layer->lslices;
            std::function<void(const ExtrusionEntity *)> lay = [&](const ExtrusionEntity *entity) {
                if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity)) {
                    for (const ExtrusionEntity *child : collection->entities)
                        lay(child);
                    return;
                }
                const Polygons covered = entity->polygons_covered_by_width(10.f);
                const auto     reaches = [&laid, reach](const Point &pt) {
                    Polygon around;
                    around.points = { Point(pt.x() - reach, pt.y() - reach), Point(pt.x() + reach, pt.y() - reach),
                                      Point(pt.x() + reach, pt.y() + reach), Point(pt.x() - reach, pt.y() + reach) };
                    return ! intersection(Polygons{ around }, to_polygons(laid)).empty();
                };
                const bool held = entity->is_loop() ? ! intersection(covered, to_polygons(laid)).empty()
                                                    : reaches(entity->first_point()) && reaches(entity->last_point());
                if (held)
                    // Only what is held holds the next thing: a raft of lines cantilevered side by side
                    // over a hole props up nothing, however solid it looks in the slice preview.
                    laid = union_ex(laid, covered);
                else
                    len += unscaled(entity->length());
            };
            lay(&region->perimeters);
            lay(&region->fills);
        }
    }
    return len;
}

// Bridge fill on the layers whose ceiling is still closing over - the slice has a hole and something in
// the layer stands on nothing - split into what was laid as closed rings and the total. A ring is a
// thread that comes back to where it started; a straight bridge line does not.
std::pair<double, double> closing_ceiling_bridge(const Print &print)
{
    double rings = 0., total = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->lower_layer == nullptr || diff_ex(layer->lslices, layer->lower_layer->lslices).empty())
            continue;
        if (std::none_of(layer->lslices.begin(), layer->lslices.end(),
                         [](const ExPolygon &island) { return ! island.holes.empty(); }))
            continue;
        for (const LayerRegion *region : layer->regions()) {
            const double width = region->bridging_flow(frSolidInfill).width();
            for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
                if (entity->role() != erBridgeInfill)
                    continue;
                const double len = unscaled(entity->length());
                total += len;
                if (len > 4. * width && (entity->first_point() - entity->last_point()).cast<double>().norm() < scaled<double>(width))
                    rings += len;
            }
        }
    }
    return { rings, total };
}

// The ceiling of a cavity closing over is held only around its rim, so a straight bridge line laid
// across it has its far end over the hole. Rings worked inward from the rim land on what is already
// there, which is what the option asks the fill for.
TEST_CASE("A ceiling closing over is bridged with rings, not lines that run into the hole", "[Perimeters]")
{
    const auto wall_generator = GENERATE("arachne", "classic");
    // The width of the ceiling ring a layer has to cover is the layer height times the slope, and how
    // the rings divide it decides whether the last one lands on the one before it. Sweep the slope so
    // that the awkward remainders are covered too.
    const double cone_height = GENERATE(1.6, 1.8, 2.0);
    INFO("wall_generator=" << wall_generator << " cone_height=" << cone_height);

    DynamicPrintConfig config = base_config(wall_generator);
    config.set_deserialize_strict({{ "bridge_unsupported_wall", "0" }});
    Print  print_off;
    Model  model_off;
    init_print({block_with_closing_cavity(cone_height)}, print_off, model_off, config);
    print_off.process();
    const double off = unheld_extrusion_length(print_off);

    // The control: walls around the hole and the far end of every line laid across it are over air.
    INFO("extrusion nothing holds with the option off: " << off << "mm");
    REQUIRE(off > 20.);

    config.set_deserialize_strict({{ "bridge_unsupported_wall", "1" }});
    Print  print_on;
    Model  model_on;
    init_print({block_with_closing_cavity(cone_height)}, print_on, model_on, config);
    print_on.process();
    const double on = unheld_extrusion_length(print_on);

    INFO("extrusion nothing holds with the option on: " << on << "mm");
    CHECK(on < 0.05 * off);

    // And the ceiling is laid down as rings, not as lines clipped to the annulus.
    const auto [rings_off, bridge_off] = closing_ceiling_bridge(print_off);
    const auto [rings_on,  bridge_on]  = closing_ceiling_bridge(print_on);
    INFO("ceiling bridge in rings: " << rings_off << "/" << bridge_off << "mm -> " << rings_on << "/" << bridge_on << "mm");
    REQUIRE(bridge_on > 10.);
    CHECK(rings_off < 0.1 * bridge_off);
    CHECK(rings_on  > 0.9 * bridge_on);
}

TEST_CASE("A wall with nothing under it is bridged instead of drawn in mid air", "[Perimeters]")
{
    const auto wall_generator = GENERATE("arachne", "classic");
    INFO("wall_generator=" << wall_generator);

    DynamicPrintConfig config = base_config(wall_generator);
    config.set_deserialize_strict({{ "bridge_unsupported_wall", "0" }});

    Print print_off;
    Model model_off;
    init_print({block_with_closing_cavity()}, print_off, model_off, config);
    print_off.process();
    const SliceLengths off = slice_lengths(print_off);
    const double       air_off = airborne_wall_length(print_off);

    // The control: the ceiling of the cavity is walled in mid air without the option.
    INFO("airborne wall with the option off: " << air_off << "mm");
    REQUIRE(air_off > 50.);

    config.set_deserialize_strict({{ "bridge_unsupported_wall", "1" }});
    Print print_on;
    Model model_on;
    init_print({block_with_closing_cavity()}, print_on, model_on, config);
    print_on.process();
    const SliceLengths on = slice_lengths(print_on);
    const double       air_on = airborne_wall_length(print_on);

    INFO("airborne wall with the option on: " << air_on << "mm");
    CHECK(air_on < 0.02 * air_off);

    // What the walls gave up, the fill takes over - it is not simply left unprinted.
    double fill_off = 0., fill_on = 0.;
    for (double len : off.fills) fill_off += len;
    for (double len : on.fills)  fill_on  += len;
    INFO("fill " << unscale<double>(fill_off) << "mm -> " << unscale<double>(fill_on) << "mm");
    CHECK(fill_on > fill_off);

    // And nothing changes where every wall has something under it: the layers below the cavity are
    // untouched.
    for (size_t i = 0; i < off.perimeters.size() && i < on.perimeters.size(); ++ i) {
        if (print_off.objects().front()->layers()[i]->print_z > 3.)
            break;
        INFO("layer " << i);
        CHECK(off.perimeters[i] == Catch::Approx(on.perimeters[i]));
    }
}
