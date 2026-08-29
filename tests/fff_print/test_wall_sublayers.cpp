#include <catch2/catch_all.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/MeshBoolean.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// The extruding Z values of one layer, in the order they are printed, split on the layer change tag
// the G-code carries once per layer. Consecutive duplicates are collapsed, so each entry is a Z the
// nozzle actually moved to in order to print something.
std::vector<std::vector<double>> extruding_zs_per_layer(const std::string &gcode)
{
    std::vector<std::vector<double>> layers;
    GCodeReader                      reader;
    reader.parse_buffer(gcode, [&layers](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos) {
            layers.emplace_back();
            return;
        }
        if (layers.empty() || ! line.extruding(self))
            return;
        const double z = self.z();
        if (layers.back().empty() || layers.back().back() != z)
            layers.back().emplace_back(z);
    });
    return layers;
}

// Distinct Z values of one layer, ascending.
std::vector<double> distinct_sorted(std::vector<double> zs)
{
    std::sort(zs.begin(), zs.end());
    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
    return zs;
}

// Smallest X reached while extruding at each Z, which tracks how far the outer wall has moved inward.
std::map<double, double> min_x_by_z(const std::string &gcode)
{
    std::map<double, double> min_x;
    GCodeReader              reader;
    reader.parse_buffer(gcode, [&min_x](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self) || line.dist_XY(self) <= 0)
            return;
        const double z = self.z();
        auto         it = min_x.find(z);
        if (it == min_x.end())
            min_x.emplace(z, self.x());
        else
            it->second = std::min(it->second, double(self.x()));
    });
    return min_x;
}


// Distinct Z heights at which something is extruded under the given ;TYPE: tag.
std::set<double> zs_of_type(const std::string &gcode, const std::string &type)
{
    std::set<double> zs;
    std::string      current;
    GCodeReader      reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        const size_t           tag = comment.find("TYPE:");
        if (tag != std::string_view::npos)
            current = std::string(comment.substr(tag + 5));
        else if (current == type && line.extruding(self) && line.dist_XY(self) > 0)
            zs.insert(self.z());
    });
    return zs;
}

// The slowest feedrate used for the given ;TYPE: tag at each Z of a layer, one entry per layer.
// Layers printing nothing of that type are left out.
std::vector<std::map<double, double>> slowest_feedrate_of_type_per_layer_z(const std::string &gcode, const std::string &type)
{
    std::vector<std::map<double, double>> layers;
    std::string                           current;
    GCodeReader                           reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos) {
            layers.emplace_back();
            return;
        }
        const size_t tag = comment.find("TYPE:");
        if (tag != std::string_view::npos) {
            current = std::string(comment.substr(tag + 5));
            return;
        }
        if (layers.empty() || current != type || ! line.extruding(self) || line.dist_XY(self) <= 0)
            return;
        auto [it, inserted] = layers.back().emplace(self.z(), self.f());
        if (! inserted)
            it->second = std::min(it->second, double(self.f()));
    });
    return layers;
}

// The XY bounding box of everything extruded under the given ;TYPE: tag, per Z.
std::map<double, BoundingBoxf> bbox_of_type_by_z(const std::string &gcode, const std::string &type)
{
    std::map<double, BoundingBoxf> boxes;
    std::string                    current;
    GCodeReader                    reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        const size_t           tag = comment.find("TYPE:");
        if (tag != std::string_view::npos) {
            current = std::string(comment.substr(tag + 5));
            return;
        }
        if (current != type || ! line.extruding(self) || line.dist_XY(self) <= 0)
            return;
        boxes[self.z()].merge(Vec2d(self.x(), self.y()));
    });
    return boxes;
}

// The same cone stood on its head, so the model gains area as Z rises instead of losing it. Each
// pass's wall then lands outside everything below it, with nothing at all underneath - a hole's
// lower lip, where a contour appears part way up the layer.
TriangleMesh inverted_shallow_cone()
{
    TriangleMesh m = make_cone(20.0, 2.0);
    m.mirror_z();
    m.translate(0., 0., 2.0);
    return m;
}

// A cone 2mm tall over a 40mm base: its surface rises 1mm for every 10mm it moves inward, so within
// one 0.2mm layer the contour sweeps 2mm sideways - far more than a wall is wide. Every sub-layer
// wall pass therefore lands well inside the pass below it, which is the shape of the problem a
// Benchy's hull makes around the hawseholes.
TriangleMesh shallow_cone() { return make_cone(20.0, 2.0); }

// 0.2mm layers with z_hop off, so a recorded Z is always a printing Z.
DynamicPrintConfig base_config(const char *sublayer_height, const char *wall_generator)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"layer_height", "0.2"},
                                   {"initial_layer_print_height", "0.2"},
                                   {"wall_loops", "3"},
                                   {"z_hop", "0"},
                                   {"skirt_loops", "0"},
                                   {"wall_generator", wall_generator},
                                   {"wall_sublayer_height", sublayer_height}});
    return config;
}

} // namespace

TEST_CASE("Sub-layered walls leave the G-code unchanged when disabled", "[WallSublayers]")
{
    // The regression guarantee: with the feature off, every move must be exactly what it was before
    // the feature existed. Comments carrying the export timestamp and the process-wide object id
    // counter differ between any two slices, so compare the commands only.
    auto commands = [](const std::string &gcode) {
        std::vector<std::string> out;
        GCodeReader              reader;
        reader.parse_buffer(gcode, [&out](GCodeReader &, const GCodeReader::GCodeLine &line) {
            if (! line.cmd().empty())
                out.emplace_back(line.raw());
        });
        return out;
    };

    const std::string with_defaults = slice({cube(20)}, {{"layer_height", "0.2"}, {"z_hop", "0"}});
    const std::string explicitly_off =
        slice({cube(20)}, {{"layer_height", "0.2"}, {"z_hop", "0"}, {"wall_sublayer_height", "0"}, {"wall_sublayer_loops", "1"}});

    REQUIRE(! with_defaults.empty());
    CHECK(commands(with_defaults) == commands(explicitly_off));
}

TEST_CASE("Sub-layered walls print each layer at several ascending Z heights", "[WallSublayers]")
{
    const auto wall_generator = GENERATE("arachne", "classic");
    // 0.2mm layers split by a 0.05mm sub-layer height give 4 passes of 0.05mm each.
    const std::string gcode = slice({cube(20)}, base_config("0.05", wall_generator));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);

    INFO("wall_generator=" << wall_generator);
    // The first layer is bonded to the bed and is never subdivided.
    CHECK(distinct_sorted(layers.front()).size() == 1);

    const std::vector<double> zs = distinct_sorted(layers[1]);
    REQUIRE(zs.size() == 4);
    for (int k = 0; k < 4; ++ k)
        REQUIRE_THAT(zs[k], Catch::Matchers::WithinAbs(0.2 + 0.05 * (k + 1), 1e-6));
    // The topmost pass lands exactly on the layer's own print_z, so the walls finish flush with the
    // inner walls and the infill printed there.
    REQUIRE_THAT(zs.back(), Catch::Matchers::WithinAbs(0.4, 1e-6));
}

TEST_CASE("Sub-layered walls never move the nozzle back down within a layer", "[WallSublayers]")
{
    // The collision-avoidance contract: Z is monotonically non-decreasing across a whole layer,
    // supports included, so the nozzle never crosses material it has already laid.
    const std::string gcode = slice({TestMesh::overhang}, {{"layer_height", "0.2"},
                                                           {"initial_layer_print_height", "0.2"},
                                                           {"wall_loops", "3"},
                                                           {"z_hop", "0"},
                                                           {"enable_support", "1"},
                                                           {"wall_sublayer_height", "0.1"}});
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    for (size_t i = 0; i < layers.size(); ++ i) {
        for (size_t k = 1; k < layers[i].size(); ++ k) {
            INFO("layer " << i << " step " << k << ": " << layers[i][k - 1] << " -> " << layers[i][k]);
            REQUIRE(layers[i][k] >= layers[i][k - 1] - EPSILON);
        }
    }
}

TEST_CASE("Sub-layered walls report the sub-layer height rather than the layer height", "[WallSublayers]")
{
    // The height tag drives the preview and the time estimate, so a thinner pass has to say so.
    const std::string gcode = slice({cube(20)}, base_config("0.05", "arachne"));

    REQUIRE(! gcode.empty());
    CHECK(gcode.find(";HEIGHT:0.05") != std::string::npos);
    CHECK(gcode.find(";HEIGHT:0.2") != std::string::npos);
}

TEST_CASE("Sub-layered walls follow the model between sub-layers on a slope", "[WallSublayers]")
{
    // The point of re-slicing: on a sloped surface each pass must sit at its own contour rather than
    // repeat the layer's. A pyramid narrows with height, so within one layer each successive pass has
    // to start further inward than the one below it.
    const std::string gcode = slice({TestMesh::pyramid}, base_config("0.05", "arachne"));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    const auto min_x  = min_x_by_z(gcode);
    REQUIRE(layers.size() > 4);

    // Take a layer from the middle of the slope, away from the tip where the walls merge.
    const std::vector<double> zs = distinct_sorted(layers[layers.size() / 3]);
    REQUIRE(zs.size() == 4);

    bool any_moved_inward = false;
    for (size_t k = 1; k < zs.size(); ++ k) {
        const auto lower = min_x.find(zs[k - 1]);
        const auto upper = min_x.find(zs[k]);
        REQUIRE(lower != min_x.end());
        REQUIRE(upper != min_x.end());
        INFO("z " << zs[k - 1] << " min_x " << lower->second << " -> z " << zs[k] << " min_x " << upper->second);
        // Never wider than the pass below it, and at least one pass is measurably narrower.
        REQUIRE(upper->second >= lower->second - EPSILON);
        any_moved_inward |= upper->second > lower->second + 0.001;
    }
    CHECK(any_moved_inward);
}

TEST_CASE("Sub-layered walls honor a sub-layer height given as a percentage", "[WallSublayers]")
{
    // The percentage resolves against the real layer height, so the same setting yields the same
    // number of passes whatever that height is.
    for (const char *layer_height : {"0.2", "0.1"}) {
        DYNAMIC_SECTION("layer height " << layer_height)
        {
            const std::string gcode = slice({cube(20)}, {{"layer_height", layer_height},
                                                        {"initial_layer_print_height", layer_height},
                                                        {"wall_loops", "3"},
                                                        {"z_hop", "0"},
                                                        {"skirt_loops", "0"},
                                                        {"wall_sublayer_height", "25%"}});
            REQUIRE(! gcode.empty());
            const auto layers = extruding_zs_per_layer(gcode);
            REQUIRE(layers.size() > 2);
            CHECK(distinct_sorted(layers[1]).size() == 4);
        }
    }
}

TEST_CASE("Sub-layered walls subdivide as many wall loops as asked for", "[WallSublayers]")
{
    // Two sub-layered loops means each pass prints an outer and an inner wall, and the layer's own
    // pass is left with the walls beyond those two.
    Print print;
    Model model;
    init_print({cube(20)}, print, model, {{"layer_height", "0.2"},
                                          {"initial_layer_print_height", "0.2"},
                                          {"wall_loops", "4"},
                                          {"wall_sublayer_height", "0.1"},
                                          {"wall_sublayer_loops", "2"}});
    print.process();

    const Layer *layer = print.objects().front()->layers()[2];
    REQUIRE(layer->wall_sub_slices.size() == 2);

    const LayerRegion *layerm = layer->regions().front();
    REQUIRE(layerm->sublayer_perimeters.size() == 2);
    for (const ExtrusionEntityCollection &pass : layerm->sublayer_perimeters)
        CHECK(! pass.empty());

    // Every loop the band passes print is gone from the layer's own perimeters, and the walls beyond
    // them are still there. Counting the survivors matters: dropping the whole set would satisfy the
    // inset check vacuously.
    int kept = 0;
    for (const ExtrusionEntity *ee : layerm->perimeters.entities)
        for (const ExtrusionEntity *loop : static_cast<const ExtrusionEntityCollection *>(ee)->entities) {
            CHECK(loop->inset_idx >= 2);
            ++ kept;
        }
    CHECK(kept == 2);
}

TEST_CASE("Sub-layered walls span the layer and end at its print_z", "[WallSublayers]")
{
    Print print;
    Model model;
    init_print({cube(20)}, print, model, {{"layer_height", "0.2"},
                                          {"initial_layer_print_height", "0.2"},
                                          {"wall_sublayer_height", "0.05"}});
    print.process();

    const auto &layers = print.objects().front()->layers();
    REQUIRE(layers.size() > 2);
    // The first layer is excluded from the feature.
    CHECK(layers.front()->wall_sub_slices.empty());

    const Layer *layer = layers[2];
    REQUIRE(layer->wall_sub_slices.size() == 4);
    REQUIRE_THAT(layer->wall_sub_slices.back().print_z, Catch::Matchers::WithinAbs(layer->print_z, 1e-9));
    REQUIRE_THAT(layer->wall_sub_slices.front().print_z - layer->wall_sub_slices.front().height,
                 Catch::Matchers::WithinAbs(layer->bottom_z(), 1e-9));
    for (const WallSubSlice &sub : layer->wall_sub_slices) {
        REQUIRE_THAT(sub.height, Catch::Matchers::WithinAbs(0.05, 1e-9));
        CHECK(! sub.merged.empty());
    }
}

TEST_CASE("Sub-layered walls of every object are printed before the next pass starts", "[WallSublayers]")
{
    // Printing by layer, the passes have to be grouped across objects: if one object climbed to its
    // layer top while another was still on its first pass, the nozzle would descend into it.
    const std::string gcode = slice_two_cubes_apart(10, {{"layer_height", "0.2"},
                                                         {"initial_layer_print_height", "0.2"},
                                                         {"wall_loops", "3"},
                                                         {"z_hop", "0"},
                                                         {"skirt_loops", "0"},
                                                         {"wall_sublayer_height", "0.1"}});
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    // Two objects at two sub-layer heights: had the passes not been grouped, the layer would show Z
    // rising and falling repeatedly. Grouped, each Z is visited exactly once per layer.
    for (size_t i = 1; i < layers.size(); ++ i) {
        INFO("layer " << i << " visits " << layers[i].size() << " Z values");
        REQUIRE(layers[i].size() == distinct_sorted(layers[i]).size());
    }
}

TEST_CASE("Sub-layered walls are rejected together with spiral vase mode", "[WallSublayers]")
{
    // Spiral vase rewrites a layer around one continuous Z ramp and cannot express several passes.
    // The GUI blocks this, but a per-object override reaches the slicer unchecked.
    Print print;
    Model model;
    init_print({cube(20)}, print, model, {{"spiral_mode", "1"},
                                          {"wall_loops", "1"},
                                          {"top_shell_layers", "0"},
                                          {"sparse_infill_density", "0"},
                                          {"wall_sublayer_height", "0.05"}});

    const auto err = print.validate();
    INFO("validate: " << err.string);
    CHECK(! err.string.empty());
    CHECK(err.opt_key == "wall_sublayer_height");
}

TEST_CASE("Sub-layered walls still print when they replace every wall", "[WallSublayers]")
{
    // With as many sub-layered loops as there are walls, the layer's own perimeters end up empty and
    // the passes are the region's only wall output. Nothing downstream may drop the object for that.
    const std::string gcode = slice({cube(20)}, {{"layer_height", "0.2"},
                                                 {"initial_layer_print_height", "0.2"},
                                                 {"wall_loops", "1"},
                                                 {"z_hop", "0"},
                                                 {"skirt_loops", "0"},
                                                 {"wall_sublayer_height", "0.1"},
                                                 {"wall_sublayer_loops", "1"}});
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    CHECK(distinct_sorted(layers[1]).size() == 2);
    CHECK(gcode.find(";HEIGHT:0.1") != std::string::npos);
}

TEST_CASE("Sub-layered walls hold up under every wall ordering", "[WallSublayers]")
{
    // The core pass drops its outermost loops after wall_sequence has ordered them, and
    // inner-outer-inner reasons about the very inset indices being dropped. Infill-first moves the
    // layer's own walls after the infill, which must not pull the passes along with them.
    const auto wall_sequence  = GENERATE("inner wall/outer wall", "outer wall/inner wall", "inner-outer-inner wall");
    const auto is_infill_first = GENERATE("0", "1");
    INFO("wall_sequence=" << wall_sequence << " is_infill_first=" << is_infill_first);

    const std::string gcode = slice({cube(20)}, {{"layer_height", "0.2"},
                                                 {"initial_layer_print_height", "0.2"},
                                                 {"wall_loops", "3"},
                                                 {"z_hop", "0"},
                                                 {"skirt_loops", "0"},
                                                 {"wall_sequence", wall_sequence},
                                                 {"is_infill_first", is_infill_first},
                                                 {"wall_sublayer_height", "0.1"}});
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    // Still two passes per layer, still ascending, and the walls the passes do not cover survive.
    CHECK(distinct_sorted(layers[1]).size() == 2);
    for (size_t i = 0; i < layers.size(); ++ i)
        for (size_t k = 1; k < layers[i].size(); ++ k)
            REQUIRE(layers[i][k] >= layers[i][k - 1] - EPSILON);
    CHECK(gcode.find(";TYPE:Inner wall") != std::string::npos);
}

TEST_CASE("Sub-layered walls carry features that have nothing else to print", "[WallSublayers]")
{
    // A 1 mm plate is walls all the way through: no sparse infill, no solid fill, and only enough
    // room for the outermost loop. Once every loop it has is sub-layered, the region's own
    // perimeters are empty and its whole content lives in the sub-layer passes - which the layer,
    // the tool ordering and the instance collection all have to notice. This is the shape of a 3D
    // Benchy's cabin wall and smoke stack.
    const auto wall_generator = GENERATE("arachne", "classic");
    auto plate_gcode = [wall_generator](const char *sublayer_loops) {
        return slice({make_cube(1.0, 20.0, 5.0)}, {{"layer_height", "0.2"},
                                                   {"initial_layer_print_height", "0.2"},
                                                   {"wall_loops", "2"},
                                                   {"z_hop", "0"},
                                                   {"skirt_loops", "0"},
                                                   {"wall_generator", wall_generator},
                                                   {"wall_sublayer_height", "0.1"},
                                                   {"wall_sublayer_loops", sublayer_loops}});
    };

    INFO("wall_generator=" << wall_generator);
    // Asking for more sub-layered loops than the feature has room for prints the ones it does have,
    // so the two cases differ in nothing that reaches the plate.
    const std::string fewer_loops = plate_gcode("1");
    const std::string every_loop  = plate_gcode("2");
    REQUIRE(! every_loop.empty());

    // 5 mm of 0.2 mm layers, all but the first subdivided into two passes.
    const auto zs = zs_of_type(every_loop, "Outer wall");
    CHECK(zs.size() >= 48);
    CHECK(zs_of_type(fewer_loops, "Outer wall").size() == zs.size());
    CHECK_THAT(max_z(every_loop), Catch::Matchers::WithinAbs(5.0, 1e-6));

    // And the passes still only ever move up within a layer.
    for (const auto &layer : extruding_zs_per_layer(every_loop))
        for (size_t k = 1; k < layer.size(); ++ k)
            REQUIRE(layer[k] >= layer[k - 1] - EPSILON);
}

TEST_CASE("Sub-layered wall passes are preview layers of their own", "[WallSublayers]")
{
    // Every pass prints at its own Z, so the preview lists it as its own layer instead of burying it
    // in the middle of the layer's moves. The passes below print_z each open one; the last pass ends
    // at print_z and shares that layer with the walls and infill printed there.
    const std::string gcode = slice({cube(20)}, base_config("0.1", "arachne"));
    REQUIRE(! gcode.empty());

    // One marker per pass below print_z, and never one on the un-subdivided first layer.
    std::vector<int> markers_per_layer;
    GCodeReader      reader;
    reader.parse_buffer(gcode, [&markers_per_layer](GCodeReader &, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos)
            markers_per_layer.emplace_back(0);
        else if (! markers_per_layer.empty() && comment == GCodeProcessor::Sub_Layer_Tag)
            ++ markers_per_layer.back();
    });
    REQUIRE(markers_per_layer.size() > 2);
    CHECK(markers_per_layer.front() == 0);
    CHECK(markers_per_layer[1] == 1);

    // The preview builds one layer per marker, so every marker has to be followed by something to
    // print - an empty preview layer would leave a gap the viewer cannot represent - and each of
    // those layers is printed at a single, rising Z.
    std::vector<std::vector<double>> preview_layers;
    GCodeReader                      preview_reader;
    preview_reader.parse_buffer(gcode, [&preview_layers](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos || comment == GCodeProcessor::Sub_Layer_Tag)
            preview_layers.emplace_back();
        else if (! preview_layers.empty() && line.extruding(self) && line.dist_XY(self) > 0)
            preview_layers.back().emplace_back(self.z());
    });

    // Twice as many preview layers as the printer prints, bar the first layer which is not split.
    CHECK(preview_layers.size() == 2 * markers_per_layer.size() - 1);
    double previous_z = 0.;
    for (size_t i = 0; i < preview_layers.size(); ++ i) {
        INFO("preview layer " << i);
        REQUIRE(! preview_layers[i].empty());
        const std::vector<double> zs = distinct_sorted(preview_layers[i]);
        CHECK(zs.size() == 1);
        CHECK(zs.front() > previous_z);
        previous_z = zs.front();
    }
}

TEST_CASE("Sub-layered walls are supported by the pass beneath them", "[WallSublayers]")
{
    // On a surface shallow enough that the contour sweeps further than a wall is wide between two
    // passes, the wall of a pass lands inside the void of the pass below it. The pass below has to
    // fill the strip it will land on first, or the wall is extruded into thin air.
    const auto wall_generator = GENERATE("arachne", "classic");
    INFO("wall_generator=" << wall_generator);

    const std::string gcode = slice({shallow_cone()}, base_config("0.05", wall_generator));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 4);

    // Solid fill printed at a Z that is not a layer's own print_z can only come from a wall pass.
    std::set<double> layer_print_zs;
    for (const auto &layer : layers)
        if (! layer.empty())
            layer_print_zs.insert(distinct_sorted(layer).back());

    int fill_at_sublayer_z = 0;
    for (const double z : zs_of_type(gcode, "Internal solid infill"))
        if (layer_print_zs.find(z) == layer_print_zs.end())
            ++ fill_at_sublayer_z;
    CHECK(fill_at_sublayer_z > 0);
}

TEST_CASE("Sub-layered walls add no support fill where the walls stack up", "[WallSublayers]")
{
    // The no-op guarantee for the support fill: a vertical wall repeats the same contour at every
    // pass, so no pass ever lands on the void inside the one below it and nothing is filled early.
    const std::string gcode = slice({cube(20)}, base_config("0.05", "arachne"));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);

    std::set<double> layer_print_zs;
    for (const auto &layer : layers)
        if (! layer.empty())
            layer_print_zs.insert(distinct_sorted(layer).back());

    for (const double z : zs_of_type(gcode, "Internal solid infill")) {
        INFO("solid infill at z " << z);
        CHECK(layer_print_zs.find(z) != layer_print_zs.end());
    }
}

TEST_CASE("Sub-layered walls keep the layer's own infill out of their columns", "[WallSublayers]")
{
    // A pass's wall band occupies its column for the whole height of the layer, so the layer's own
    // infill - printed afterwards at print_z, at the full layer height - must not be laid over it.
    auto fill_area = [](const char *sublayer_height) {
        Print print;
        Model model;
        DynamicPrintConfig config = base_config(sublayer_height, "arachne");
        // A single wall, so the fill area reaches out far enough for the passes to sweep across it.
        // With three walls the 2mm the contour travels within a layer is swallowed by the wall stack.
        config.set_deserialize_strict({{"wall_loops", "1"}});
        init_print({shallow_cone()}, print, model, config);
        print.process();

        const auto &layers = print.objects().front()->layers();
        double      area   = 0.;
        for (size_t i = 1; i < layers.size(); ++ i)
            for (const LayerRegion *layerm : layers[i]->regions())
                for (const ExPolygon &expoly : layerm->fill_expolygons)
                    area += expoly.area();
        return area;
    };

    const double plain      = fill_area("0");
    const double sublayered = fill_area("0.05");
    REQUIRE(plain > 0.);
    // The columns the passes occupy are given up by the layer's fill, so there is strictly less of it.
    CHECK(sublayered < plain);
}

TEST_CASE("Sub-layered walls are not slowed down as overhangs", "[WallSublayers]")
{
    // Every pass of a layer is printed onto the pass right below it, the same sub-layer height down,
    // so the material under each of them is alike and they all deserve the same speed. Measured
    // instead against the layer below - which is what the estimator does for an ordinary wall - the
    // passes are between one and two layer heights above their reference, so the higher ones read as
    // hanging over further than they do and pick up an overhang slowdown, with the part cooling fan
    // behind it, that a full-height wall over the same geometry never triggers.
    DynamicPrintConfig config = base_config("0.05", "arachne");
    config.set_deserialize_strict({{"enable_overhang_speed", "1"},
                                   {"slowdown_for_curled_perimeters", "0"},
                                   {"enable_auto_cooling", "0"},
                                   // Distinct speeds per overhang band, so any misjudged pass shows
                                   // up as a feedrate that differs from the passes around it.
                                   {"overhang_1_4_speed", "50"},
                                   {"overhang_2_4_speed", "40"},
                                   {"overhang_3_4_speed", "30"},
                                   {"overhang_4_4_speed", "20"},
                                   {"outer_wall_speed", "60"}});

    // A sphere presents every wall angle there is, so it exercises both the overhanging lower half
    // and the receding upper half.
    const std::string gcode = slice({mesh(TestMesh::sphere_50mm)}, config);
    REQUIRE(! gcode.empty());

    int consistent = 0;
    int total      = 0;
    for (const auto &layer : slowest_feedrate_of_type_per_layer_z(gcode, "Outer wall")) {
        if (layer.size() < 2)
            continue;
        ++ total;
        const double first = layer.begin()->second;
        if (std::all_of(layer.begin(), layer.end(), [first](const auto &z_f) { return std::abs(z_f.second - first) < 1.; }))
            ++ consistent;
    }
    REQUIRE(total > 100);

    // The sphere's poles genuinely change shape from one pass to the next, so a few layers differ on
    // their own account. Measured against the layer below instead, a third of them do.
    INFO(consistent << " of " << total << " layers print all their passes at one speed");
    CHECK(consistent > total * 9 / 10);
}


TEST_CASE("Sub-layered walls support a wall line that stands over nothing", "[WallSublayers]")
{
    // Where a contour appears part way up a layer - the lower lip of a hole - the next pass's wall
    // lands outside everything below it rather than inside the void of the pass beneath, so there is
    // nothing for the pass below to fill *within* its own contour. It has to reach past that contour
    // and lay the material down anyway, or the wall is printed in mid air.
    const auto wall_generator = GENERATE("arachne", "classic");
    INFO("wall_generator=" << wall_generator);

    const std::string gcode = slice({inverted_shallow_cone()}, base_config("0.05", wall_generator));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 4);

    std::set<double> layer_print_zs;
    for (const auto &layer : layers)
        if (! layer.empty())
            layer_print_zs.insert(distinct_sorted(layer).back());

    int fill_at_sublayer_z = 0;
    for (const double z : zs_of_type(gcode, "Internal solid infill"))
        if (layer_print_zs.find(z) == layer_print_zs.end())
            ++ fill_at_sublayer_z;
    CHECK(fill_at_sublayer_z > 0);
}

TEST_CASE("Sub-layered walls keep the layer's own walls inside the topmost pass", "[WallSublayers]")
{
    // The core run and the topmost pass both print at print_z, so the contour the layer presents
    // there is the topmost pass's. Built from the layer's mid-height slice instead, the walls the
    // core run is left with sit outside that contour on any slope shallower than about 13 degrees,
    // hanging off the edge of the model.
    const std::string gcode = slice({shallow_cone()}, base_config("0.05", "arachne"));
    REQUIRE(! gcode.empty());

    const auto outer = bbox_of_type_by_z(gcode, "Outer wall");
    const auto inner = bbox_of_type_by_z(gcode, "Inner wall");
    REQUIRE(! outer.empty());

    int compared = 0;
    for (const auto &[z, inner_box] : inner) {
        const auto outer_box = outer.find(z);
        if (outer_box == outer.end())
            continue;
        INFO("at z " << z << " inner walls span " << inner_box.min.x() << ".." << inner_box.max.x()
             << " and the outer wall " << outer_box->second.min.x() << ".." << outer_box->second.max.x());
        // An inner wall reaching further out than the outer wall is outside the printed surface.
        CHECK(inner_box.max.x() <= outer_box->second.max.x() + EPSILON);
        CHECK(inner_box.min.x() >= outer_box->second.min.x() - EPSILON);
        ++ compared;
    }
    CHECK(compared > 4);
}

// A block bored through by a round horizontal hole - the shape of a Benchy's hawsehole. Where the
// bore closes over, the contour sweeps far enough between sub-layers to expose what the passes and
// the layer's own full-height phase disagree about. A wall inset of three loops at 0.2mm hides it,
// so this is deliberately sliced thick and thin-walled.
TriangleMesh bored_block()
{
    TriangleMesh block = make_cube(30., 24., 20.);
    TriangleMesh bore  = make_cylinder(5., 40., 2. * PI / 60.);
    bore.rotate_x(float(-PI / 2.));   // axis along Y, through the middle of the block
    bore.translate(15., -8., 10.);
    MeshBoolean::cgal::minus(block, bore);
    return block;
}

TEST_CASE("The layer's full-height phase stays out of what the passes own", "[WallSublayers]")
{
    // Everything the core run emits is printed at print_z over the whole layer height, so it may
    // only occupy ground that is solid for that entire height and that no pass has printed on.
    // Measured against the layer's mid-height slice instead, it comes down on a column a pass has
    // already laid a wall on, and reaches into ground that is a hole at some sub-layer.
    Print print;
    Model model;
    init_print({bored_block()}, print, model, {{"layer_height", "0.3"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
        {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    print.process();

    double worst_on_band = 0., worst_outside = 0., worst_z = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->wall_sub_slices.size() < 2)
            continue;
        // The union of the passes' wall bands, and the ground solid for the whole layer height.
        ExPolygons bands, common = layer->wall_sub_slices.front().merged;
        for (const WallSubSlice &sub : layer->wall_sub_slices) {
            append(bands, diff_ex(sub.merged, offset_ex(sub.merged, - scale_(0.42))));
            common = intersection_ex(common, sub.merged);
        }
        // Shrunk, so merely touching a band's edge is not counted as standing on it.
        const Polygons bands_p = to_polygons(offset_ex(union_ex(bands), - scale_(0.15)));

        for (const LayerRegion *layerm : layer->regions()) {
            Polylines walls;
            for (const ExtrusionEntity *island : layerm->perimeters.entities)
                for (const ExtrusionEntity *loop : static_cast<const ExtrusionEntityCollection*>(island)->entities)
                    walls.push_back(loop->as_polyline());
            double on_band = 0.;
            for (const Polyline &p : intersection_pl(walls, bands_p))
                on_band += unscale<double>(p.length());

            double outside = 0.;
            for (const ExPolygon &e : diff_ex(layerm->fill_expolygons, common))
                outside += unscale<double>(unscale<double>(e.area()));

            if (on_band > worst_on_band || outside > worst_outside)
                worst_z = layer->print_z;
            worst_on_band = std::max(worst_on_band, on_band);
            worst_outside = std::max(worst_outside, outside);
        }
    }
    INFO("worst layer at z " << worst_z);
    // Before the core run was confined, the layer where the bore closes over showed 1.86mm of wall
    // standing on a pass's band and 24mm2 of infill over ground that is a hole at some sub-layer.
    CHECK(worst_on_band < 0.01);
    CHECK(worst_outside < 0.01);
}

// Every solid-infill path a pass laid down, however deeply the fillers nested it.
static void collect_pass_fill(const ExtrusionEntity *ee, Polylines &out)
{
    if (const auto *coll = dynamic_cast<const ExtrusionEntityCollection*>(ee)) {
        for (const ExtrusionEntity *child : coll->entities)
            collect_pass_fill(child, out);
    } else if (ee->role() == erSolidInfill) {
        out.push_back(ee->as_polyline());
    }
}

TEST_CASE("Sub-layered wall support fill is not sprayed over overhangs", "[WallSublayers]")
{
    // A cone stood on its head rolls outward with Z exactly as a Benchy's gunwale does, so every
    // pass lands a little further out than the one below. A wall that still comes down on most of
    // the wall beneath it is an ordinary overhang and needs nothing: propping each one up buried
    // the print in tiny fill fragments that held up nothing.
    TriangleMesh cone = make_cone(20.0, 2.0);
    cone.mirror_z();
    cone.translate(0., 0., 2.0);

    Print print;
    Model model;
    init_print({cone}, print, model, {{"layer_height", "0.3"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
        {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    print.process();

    double length = 0.;
    int    paths = 0, fragments = 0;
    for (const Layer *layer : print.objects().front()->layers())
        for (size_t k = 0; k < layer->wall_sub_slices.size(); ++ k) {
            Polylines fill;
            for (const LayerRegion *layerm : layer->regions())
                if (k < layerm->sublayer_perimeters.size())
                    for (const ExtrusionEntity *ee : layerm->sublayer_perimeters[k].entities)
                        collect_pass_fill(ee, fill);
            paths += (int) fill.size();
            for (const Polyline &p : fill) {
                const double l = unscale<double>(p.length());
                length += l;
                if (l < 6.)
                    ++ fragments;
            }
        }

    INFO("support fill " << length << "mm over " << paths << " paths, " << fragments << " under 6mm");
    // Before the overhang rule, this cone drew 643 paths of which 547 were these stubs; now 107/28.
    CHECK(fragments < 100);
    // And the staircase it does have to build is still built - the fix must not simply emit nothing.
    CHECK(length > 1000.);
}
