
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MeshBoolean.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/libslic3r.h"

#include "test_helpers.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
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


// Calls `visit` with the reader positioned at the start of every extruding XY move printed under the
// given ;TYPE: tag, and with the tool printing it, and `on_layer`, when given, at every layer change tag.
void for_each_move_of_type(const std::string &gcode, const std::string &type,
                           const std::function<void(const GCodeReader &, int)> &visit,
                           const std::function<void()> &on_layer = {})
{
    std::string current;
    int         tool = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (const std::string_view cmd = line.cmd(); cmd.size() > 1 && cmd[0] == 'T' && std::isdigit(static_cast<unsigned char>(cmd[1]))) {
            tool = std::stoi(std::string(cmd.substr(1)));
            return;
        }
        const std::string_view comment = line.comment();
        if (on_layer && comment.find("LAYER_CHANGE") != std::string_view::npos) {
            on_layer();
            return;
        }
        if (const size_t tag = comment.find("TYPE:"); tag != std::string_view::npos) {
            current = std::string(comment.substr(tag + 5));
            return;
        }
        if (current == type && line.extruding(self) && line.dist_XY(self) > 0)
            visit(self, tool);
    });
}

// Distinct Z heights at which something is extruded under the given ;TYPE: tag.
std::set<double> zs_of_type(const std::string &gcode, const std::string &type)
{
    std::set<double> zs;
    for_each_move_of_type(gcode, type, [&zs](const GCodeReader &self, int) { zs.insert(self.z()); });
    return zs;
}

// The slowest feedrate used for the given ;TYPE: tag at each Z of a layer, one entry per layer.
// Layers printing nothing of that type are left out.
std::vector<std::map<double, double>> slowest_feedrate_of_type_per_layer_z(const std::string &gcode, const std::string &type)
{
    std::vector<std::map<double, double>> layers;
    for_each_move_of_type(
        gcode, type,
        [&layers](const GCodeReader &self, int) {
            if (layers.empty())
                return;
            auto [it, inserted] = layers.back().emplace(self.z(), self.f());
            if (! inserted)
                it->second = std::min(it->second, double(self.f()));
        },
        [&layers]() { layers.emplace_back(); });
    return layers;
}

// The XY bounding box of everything extruded under the given ;TYPE: tag, per Z.
std::map<double, BoundingBoxf> bbox_of_type_by_z(const std::string &gcode, const std::string &type)
{
    std::map<double, BoundingBoxf> boxes;
    for_each_move_of_type(gcode, type, [&boxes](const GCodeReader &self, int) { boxes[self.z()].merge(Vec2d(self.x(), self.y())); });
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

TEST_CASE("Sub-layered walls leave the layers elephant foot compensation shrinks whole", "[WallSublayers]")
{
    // Elephant foot compensation shrinks the slices of the first elefant_foot_compensation_layers
    // layers, and sub-slices taken from the model would not follow it. With 3 compensated layers those
    // print at one Z each, and the fourth 0.2mm layer is split by 0.05mm into 4 passes.
    DynamicPrintConfig config = base_config("0.05", "arachne");
    config.set_deserialize_strict({{"raft_layers", "0"},
                                   {"elefant_foot_compensation", "0.2"},
                                   {"elefant_foot_compensation_layers", "3"}});
    const std::string gcode = slice({cube(20)}, config);
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 4);
    for (size_t i = 0; i < 3; ++ i) {
        INFO("layer " << i);
        CHECK(distinct_sorted(layers[i]).size() == 1);
    }
    CHECK(distinct_sorted(layers[3]).size() == 4);
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
    REQUIRE(layer->wall_sub_slices().size() == 2);

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
    CHECK(layers.front()->wall_sub_slices().empty());

    const Layer *layer = layers[2];
    REQUIRE(layer->wall_sub_slices().size() == 4);
    REQUIRE_THAT(layer->wall_sub_slices().back().print_z, Catch::Matchers::WithinAbs(layer->print_z, 1e-9));
    REQUIRE_THAT(layer->wall_sub_slices().front().print_z - layer->wall_sub_slices().front().height,
                 Catch::Matchers::WithinAbs(layer->bottom_z(), 1e-9));
    for (const WallSubSlice &sub : layer->wall_sub_slices()) {
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


TEST_CASE("A sub-layered wall over nothing is an overhang, not something to fill under", "[WallSublayers]")
{
    // Where the contour rolls outward - the lower lip of a hole, a Benchy's gunwale - each pass's
    // wall lands outside everything below it. A wall may do that: the generator classifies it as an
    // overhang and prints it as one, which is what an ordinary layer does too. What a pass must not
    // do is fill under it, because that fill would stand on air itself.
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

    // The wall is still printed at the sub-layer heights - the surface is not left with a hole. Every
    // pass here lands outside the one below, so the wall carries the overhang role for most of its
    // length and both tags have to be counted.
    std::set<double> wall_zs = zs_of_type(gcode, "Outer wall");
    for (const double z : zs_of_type(gcode, "Overhang wall"))
        wall_zs.insert(z);
    int wall_at_sublayer_z = 0;
    for (const double z : wall_zs)
        if (layer_print_zs.find(z) == layer_print_zs.end())
            ++ wall_at_sublayer_z;
    CHECK(wall_at_sublayer_z > 0);

    // And the overhang is carried by the overhang role rather than by fill beneath it.
    CHECK(gcode.find(";TYPE:Overhang wall") != std::string::npos);
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
        if (layer->wall_sub_slices().size() < 2)
            continue;
        // The union of the passes' wall bands, and the ground solid for the whole layer height.
        ExPolygons bands, common = layer->wall_sub_slices().front().merged;
        for (const WallSubSlice &sub : layer->wall_sub_slices()) {
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
        for (size_t k = 0; k < layer->wall_sub_slices().size(); ++ k) {
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
    // Nothing at all: every pass here lands outside the one below, so there is no ground to fill on.
    // The wall prints as the overhang it is, and whatever needs filling belongs to the layer's own
    // pass at print_z. Earlier designs chased these overhangs and drew 643 paths, 547 of them stubs
    // of under 6mm, which showed on the print as blobs on an outward-rolling surface.
    CHECK(length < 10.);
    CHECK(fragments == 0);
}

// A box hollowed from the bed up to a ceiling that lands part way through a layer rather than on one
// of its boundaries, so the roof closes over between two sub-layers.
TriangleMesh roofed_box()
{
    TriangleMesh box    = make_cube(20., 20., 12.);
    TriangleMesh cavity = make_cube(12., 12., 8.15);
    cavity.translate(4., 4., -0.5);   // open to the bed, ceiling at 8.15
    MeshBoolean::cgal::minus(box, cavity);
    return box;
}

TEST_CASE("Sub-layered wall support fill never stands on a void", "[WallSublayers]")
{
    // A pass stands on what the pass below it actually printed, not on that sub-layer's slice: the
    // slice says only where the model is solid, so once a ceiling closes over part way up a layer it
    // claims ground the pass below left as air. Reported from a 3DBenchy, whose cabin roof came out
    // as solid infill laid at full speed over the cabin and then collided with the hot end.
    const double layer_height = 0.3, first_layer = 0.2;
    const std::string gcode   = slice({roofed_box()}, {{"layer_height", "0.3"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
        {"z_hop", "0"}, {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    REQUIRE(! gcode.empty());

    // The object is centred on the bed, so the cavity is the middle 12mm of its 20mm footprint.
    BoundingBoxf bbox;
    GCodeReader  probe;
    probe.parse_buffer(gcode, [&bbox](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            bbox.merge(Vec2d(self.x(), self.y()));
    });
    const Vec2d  centre = bbox.center();
    const double reach  = 5.;   // 1mm inside the 6mm half-width of the cavity

    // Solid infill printed at a Z that is not a layer's print_z is a pass's support fill.
    double       over_void = 0.;
    std::string  type;
    GCodeReader  reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (const size_t tag = comment.find("TYPE:"); tag != std::string_view::npos) {
            type = std::string(comment.substr(tag + 5));
            return;
        }
        if (type != "Internal solid infill" || ! line.extruding(self) || line.dist_XY(self) <= 0)
            return;
        const double above_first = self.z() - first_layer;
        if (std::abs(above_first / layer_height - std::round(above_first / layer_height)) < 1e-4)
            return;   // a layer's own print_z, where the roof is bridged normally
        if (std::abs(self.x() - centre.x()) < reach && std::abs(self.y() - centre.y()) < reach)
            over_void += line.dist_XY(self);
    });

    INFO("support fill over the cavity: " << over_void << "mm");
    CHECK(over_void < 1.);
}

// A thin plate with a window cut through it, the window's top edge landing part way through a layer.
// Above the opening the plate's wall crosses it, anchored on the jambs at either end.
TriangleMesh windowed_plate()
{
    TriangleMesh plate  = make_cube(20., 1., 12.);
    TriangleMesh window = make_cube(10., 3., 4.15);
    window.translate(5., -1., 4.);   // opening X 5..15, right through the thickness, top at 8.15
    MeshBoolean::cgal::minus(plate, window);
    return plate;
}

TEST_CASE("A sub-layered pass keeps a wall that crosses an opening", "[WallSublayers]")
{
    // The wall above a window is part of the plate's own perimeter, anchored at both ends, and has to
    // be printed - the overhang handling is what carries it. Dropping stranded material must never
    // take a whole perimeter with it: an earlier rule judged this island by the longest unsupported
    // run anywhere in it and deleted the plate's entire outline, which the tests of the day did not
    // notice because they only asked whether anything was left over the opening.
    const std::string gcode = slice({windowed_plate()}, {{"layer_height", "0.3"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
        {"z_hop", "0"}, {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    REQUIRE(! gcode.empty());

    BoundingBoxf bbox;
    GCodeReader  probe;
    probe.parse_buffer(gcode, [&bbox](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            bbox.merge(Vec2d(self.x(), self.y()));
    });
    const double mid_x = bbox.center().x();

    const double layer_height = 0.3, first_layer = 0.2;
    double       over_opening = 0.;
    GCodeReader  reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self) || line.dist_XY(self) <= 0)
            return;
        const double above_first = self.z() - first_layer;
        if (std::abs(above_first / layer_height - std::round(above_first / layer_height)) < 1e-4)
            return;
        if (self.z() > 8.15 && self.z() < 8.3 && std::abs(self.x() - mid_x) < 3.)
            over_opening += line.dist_XY(self);
    });

    INFO("wall crossing the opening at a sub-layer height: " << over_opening << "mm");
    CHECK(over_opening > 3.);
}

// A box with a shallow square recess in its underside, shallower than one layer, so it closes over
// between two sub-layers rather than on a layer boundary - the engraved text on a 3DBenchy's hull.
TriangleMesh engraved_box()
{
    TriangleMesh box = make_cube(20., 20., 6.);
    for (const Vec3d &at : {Vec3d(7., 7., -0.3), Vec3d(13., 9., -0.35), Vec3d(9., 13., -0.28)}) {
        TriangleMesh engrave = make_cylinder(1.6, 0.6);
        engrave.translate(at.x(), at.y(), at.z());   // open to the bed, ceiling just above 0.25
        MeshBoolean::cgal::minus(box, engrave);
    }
    return box;
}

TEST_CASE("Sub-layers do not trace a recess the layer closes over", "[WallSublayers]")
{
    // The layer's own slice is taken at its mid-height, where the recess has already closed, so the
    // layer covers it in one bridge. The sub-slices below the ceiling still see it open, and taking
    // the core from their intersection punched the recess back into a layer that has no such hole:
    // every character of a 3DBenchy's bottom text came out as a wall, drawn in mid-air over the
    // recess and buried by the bridge a moment later.
    const double layer_height = 0.3, first_layer = 0.2;
    const std::string gcode = slice({engraved_box()}, {{"layer_height", "0.3"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
        {"z_hop", "0"}, {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    REQUIRE(! gcode.empty());

    BoundingBoxf bbox;
    GCodeReader  probe;
    probe.parse_buffer(gcode, [&bbox](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            bbox.merge(Vec2d(self.x(), self.y()));
    });
    const Vec2d centre = bbox.center();

    // Wall printed around the recess, anywhere between the first layer and the layer that covers it.
    double      around_recess = 0.;
    std::string type;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (const size_t tag = comment.find("TYPE:"); tag != std::string_view::npos) {
            type = std::string(comment.substr(tag + 5));
            return;
        }
        if (type.find("wall") == std::string::npos || ! line.extruding(self) || line.dist_XY(self) <= 0)
            return;
        if (self.z() <= first_layer + EPSILON || self.z() > first_layer + layer_height + EPSILON)
            return;
        if (std::abs(self.x() - centre.x()) < 4.5 && std::abs(self.y() - centre.y()) < 4.5)
            around_recess += line.dist_XY(self);
    });

    INFO("wall drawn around the closed-over recess: " << around_recess << "mm");
    CHECK(around_recess < 1.);
}

// A hollow box with a pillar standing in the middle of the hollow, so every layer above the floor
// slices as a ring with an island inside its hole - the topology of the box behind a 3DBenchy's
// cabin, which stands inside the hull.
TriangleMesh pillared_box()
{
    TriangleMesh box    = make_cube(20., 20., 12.);
    TriangleMesh cavity = make_cube(14., 14., 12.);
    cavity.translate(3., 3., 2.);   // hollow from z=2 up, open at the top
    TriangleMesh pillar = make_cube(4., 4., 12.);
    pillar.translate(8., 8., 1.);   // rooted in the floor, standing inside the hollow
    MeshBoolean::cgal::minus(cavity, pillar);
    MeshBoolean::cgal::minus(box, cavity);
    return box;
}

TEST_CASE("A sub-layer pass keeps an island that stands inside a layer's hole", "[WallSublayers]")
{
    // Dropping the voids a layer does not have must never take material with it. Deciding it by
    // subtracting the layer's holes from the sub-slice deleted every island that stands inside one,
    // because such an island lies wholly within the hole it sits in: on a 3DBenchy the cabin and the
    // box behind it vanished for two millimetres of Z and the walls above them started in mid air.
    const double layer_height = 0.3, first_layer = 0.2;
    const std::string gcode = slice({pillared_box()}, {{"layer_height", "0.3"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
        {"z_hop", "0"}, {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    REQUIRE(! gcode.empty());

    BoundingBoxf bbox;
    GCodeReader  probe;
    probe.parse_buffer(gcode, [&bbox](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            bbox.merge(Vec2d(self.x(), self.y()));
    });
    const Vec2d centre = bbox.center();

    // Sub-layer Z values between 6 and 8mm at which the pillar has something printed.
    std::set<double> pass_zs, pillar_zs;
    GCodeReader      reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (! line.extruding(self) || line.dist_XY(self) <= 0 || self.z() < 6. || self.z() > 8.)
            return;
        const double above_first = self.z() - first_layer;
        if (std::abs(above_first / layer_height - std::round(above_first / layer_height)) < 1e-4)
            return;   // a layer's own print_z
        pass_zs.insert(self.z());
        if (std::abs(self.x() - centre.x()) < 2.5 && std::abs(self.y() - centre.y()) < 2.5)
            pillar_zs.insert(self.z());
    });

    REQUIRE(! pass_zs.empty());
    INFO("pillar printed at " << pillar_zs.size() << " of " << pass_zs.size() << " sub-layer heights");
    CHECK(pillar_zs.size() == pass_zs.size());
}

// All non-zero part cooling fan speeds the G-code sets, as raw S values.
static std::set<int> fan_speeds(const std::string &gcode)
{
    std::set<int> speeds;
    GCodeReader   reader;
    reader.parse_buffer(gcode, [&speeds](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.cmd() != "M106")
            return;
        float s = 0.f;
        if (line.has_value('S', s) && s > 0.f)
            speeds.insert((int) std::lround(s));
    });
    return speeds;
}

// The extrusion widths reported by ;WIDTH: at each Z, which is how wide the thread laid there is.
static std::map<double, std::set<double>> widths_by_z(const std::string &gcode)
{
    std::map<double, std::set<double>> widths;
    double                             current = 0.;
    GCodeReader                        reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        const size_t           tag     = comment.find("WIDTH:");
        if (tag != std::string_view::npos) {
            current = atof(std::string(comment.substr(tag + 6)).c_str());
            return;
        }
        if (current > 0. && line.extruding(self) && line.dist_XY(self) > 0)
            widths[self.z()].insert(current);
    });
    return widths;
}

TEST_CASE("Sub-layered walls take their own speed", "[WallSublayers]")
{
    // A pass lays a fraction of a layer, so slowing one down costs a fraction of what slowing the
    // outer wall everywhere costs - which is the point of having a speed of its own. With one banded
    // loop the passes own every "Outer wall" extrusion and the layer's own run owns every "Inner
    // wall" one, so the two are told apart by their type tag.
    auto config = [](const char *sublayer_speed) {
        DynamicPrintConfig c = base_config("0.05", "arachne");
        c.set_deserialize_strict({{"wall_sublayer_loops", "1"},
                                  {"slow_down_for_layer_cooling", "0"},
                                  {"enable_overhang_speed", "0"},
                                  {"slowdown_for_curled_perimeters", "0"},
                                  {"outer_wall_speed", "60"},
                                  {"inner_wall_speed", "60"},
                                  {"wall_sublayer_speed", sublayer_speed}});
        return c;
    };

    // Skipping the first layers, which print at the initial layer speed whatever else is set.
    auto slowest = [](const std::string &gcode, const char *type) {
        double     f      = std::numeric_limits<double>::max();
        const auto layers = slowest_feedrate_of_type_per_layer_z(gcode, type);
        for (size_t i = 3; i < layers.size(); ++ i)
            for (const auto &[z, feedrate] : layers[i])
                f = std::min(f, feedrate);
        return f;
    };
    auto fastest = [](const std::string &gcode, const char *type) {
        double f = 0.;
        for (const auto &layer : slowest_feedrate_of_type_per_layer_z(gcode, type))
            for (const auto &[z, feedrate] : layer)
                f = std::max(f, feedrate);
        return f;
    };

    const std::string full = slice({cube(20)}, config("100%"));
    const std::string half = slice({cube(20)}, config("50%"));
    REQUIRE(! full.empty());
    REQUIRE(! half.empty());

    // 100% is the speed the extrusion type would have used anyway.
    CHECK_THAT(fastest(full, "Outer wall"), Catch::Matchers::WithinRel(60. * 60., 0.02));
    // 50% halves the passes.
    CHECK_THAT(fastest(half, "Outer wall"), Catch::Matchers::WithinRel(30. * 60., 0.02));
    // and leaves the walls the layer still prints at its own height alone: every inner wall keeps the
    // feedrate it had, whatever the volumetric limit and the initial layers made of it.
    const auto full_inner = slowest_feedrate_of_type_per_layer_z(full, "Inner wall");
    const auto half_inner = slowest_feedrate_of_type_per_layer_z(half, "Inner wall");
    REQUIRE(full_inner.size() > 10);
    CHECK(full_inner == half_inner);
    CHECK(slowest(full, "Inner wall") > 0.);
}

TEST_CASE("Sub-layered walls are given a minimum layer time per pass", "[WallSublayers]")
{
    // The nozzle comes back to the same spot once per pass, so the layer has to last as many minimum
    // layer times as it has passes for each of those intervals to be one. Splitting a layer into more
    // passes therefore has to slow it down further, not less - even though more passes also means
    // more material and so a longer layer to begin with.
    auto slowest_feedrate = [](const char *sublayer_height) {
        DynamicPrintConfig config = base_config(sublayer_height, "arachne");
        config.set_deserialize_strict({{"wall_loops", "2"},
                                       {"slow_down_for_layer_cooling", "1"},
                                       {"slow_down_layer_time", "5"},
                                       {"slow_down_min_speed", "1"},
                                       {"enable_overhang_speed", "0"}});
        // Small enough that a layer is well under the minimum layer time and the regulator has to act.
        const std::string gcode = slice({cube(5)}, config);
        REQUIRE(! gcode.empty());
        double f = std::numeric_limits<double>::max();
        const auto layers = slowest_feedrate_of_type_per_layer_z(gcode, "Outer wall");
        REQUIRE(layers.size() > 4);
        // Skip the first layers, whose speeds are dictated by the initial layer settings.
        for (size_t i = 3; i < layers.size(); ++ i)
            for (const auto &[z, feedrate] : layers[i])
                f = std::min(f, feedrate);
        return f;
    };

    const double two_passes  = slowest_feedrate("0.1");
    const double four_passes = slowest_feedrate("0.05");
    INFO("2 passes slowest " << two_passes << ", 4 passes slowest " << four_passes);
    CHECK(four_passes < two_passes);
}

TEST_CASE("Sub-layered walls take their own fan speed", "[WallSublayers]")
{
    // With every other fan source pinned to zero, any fan the print turns on is the sub-layer one.
    auto config = [](const char *sublayer_fan) {
        DynamicPrintConfig c = base_config("0.05", "arachne");
        c.set_deserialize_strict({{"slow_down_for_layer_cooling", "0"},
                                  {"fan_min_speed", "0"},
                                  {"fan_max_speed", "0"},
                                  {"reduce_fan_stop_start_freq", "0"},
                                  {"close_fan_the_first_x_layers", "0"},
                                  {"enable_overhang_bridge_fan", "0"},
                                  {"additional_cooling_fan_speed", "0"},
                                  {"initial_layer_fan_speed", "-1"},
                                  {"part_cooling_fan_min_pwm", "0"},
                                  {"wall_sublayer_fan_speed", sublayer_fan}});
        return c;
    };

    const std::set<int> off = fan_speeds(slice({cube(20)}, config("-1")));
    const std::set<int> on  = fan_speeds(slice({cube(20)}, config("40")));

    INFO("off: " << off.size() << " speeds, on: " << on.size() << " speeds");
    CHECK(off.empty());
    REQUIRE(on.size() == 1);
    // 40% of the fan's range.
    CHECK(*on.begin() == (int) std::lround(40. * 255. / 100.));
}

TEST_CASE("A narrower sub-layer line still covers the wall it replaces", "[WallSublayers]")
{
    // A pass a fraction of a layer tall but a full wall wide is far flatter than it is wide and hard
    // to lay down evenly. Narrowing it has to leave the wall as thick as it was, so the passes take
    // proportionally more loops to cover the same strip - otherwise the band and the walls the layer
    // still prints itself part company and leave a groove.
    auto config = [](const char *line_width) {
        DynamicPrintConfig c = base_config("0.05", "arachne");
        c.set_deserialize_strict({{"wall_loops", "4"},
                                  {"wall_sublayer_loops", "2"},
                                  {"outer_wall_line_width", "0.45"},
                                  {"inner_wall_line_width", "0.45"},
                                  {"wall_sublayer_line_width", line_width}});
        return c;
    };

    const std::string wide   = slice({cube(20)}, config("0"));
    const std::string narrow = slice({cube(20)}, config("0.3"));
    REQUIRE(! wide.empty());
    REQUIRE(! narrow.empty());

    // A pass Z is any Z that is not a layer boundary; on this cube only the band prints there.
    const auto pass_z = [](double z) { return std::abs(std::fmod(z + 1e-6, 0.2)) > 2e-6; };

    const auto narrow_widths = widths_by_z(narrow);
    int        pass_zs = 0;
    for (const auto &[z, widths] : narrow_widths) {
        if (z < 1. || ! pass_z(z))
            continue;
        ++ pass_zs;
        for (const double w : widths) {
            INFO("z " << z << " width " << w);
            REQUIRE_THAT(w, Catch::Matchers::WithinAbs(0.3, 0.05));
        }
    }
    REQUIRE(pass_zs > 10);

    // The layer's own walls are untouched by the setting. They print at the layer's print_z, which the
    // topmost pass also finishes on, so that Z carries both widths.
    int boundary_zs = 0;
    for (const auto &[z, widths] : narrow_widths)
        if (z > 1. && ! pass_z(z)) {
            ++ boundary_zs;
            INFO("z " << z);
            CHECK(std::any_of(widths.begin(), widths.end(), [](double w) { return w > 0.35; }));
        }
    REQUIRE(boundary_zs > 10);

    // And the band still reaches as far in as it did at the full width, so nothing is left bare
    // between it and the first wall the layer prints itself.
    const auto wide_x   = min_x_by_z(wide);
    const auto narrow_x = min_x_by_z(narrow);
    double     worst    = 0.;
    for (const auto &[z, x] : wide_x) {
        if (z < 1. || ! pass_z(z))
            continue;
        const auto it = narrow_x.find(z);
        REQUIRE(it != narrow_x.end());
        worst = std::max(worst, std::abs(it->second - x));
    }
    INFO("innermost band extent moved by at most " << worst << "mm");
    CHECK(worst < 0.25);
}

TEST_CASE("A concentric sub-layer fill follows the surface it fills", "[WallSublayers]")
{
    // What a pass has to fill is the tread of the staircase a sloped surface makes: a long narrow
    // ring following the wall. Filled with straight lines it comes out as stubs across the ring;
    // filled concentrically it is one loop per line. Which suits a given model is the user's call, so
    // the pattern is an option and rectilinear stays the default.
    auto closed_fraction = [](const char *pattern) {
        Print print;
        Model model;
        init_print({shallow_cone()}, print, model, {{"layer_height", "0.2"},
            {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"skirt_loops", "0"},
            {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"},
            {"wall_sublayer_fill_pattern", pattern}});
        print.process();

        int closed = 0, open = 0;
        for (const Layer *layer : print.objects().front()->layers())
            for (size_t k = 0; k < layer->wall_sub_slices().size(); ++ k) {
                Polylines fill;
                for (const LayerRegion *layerm : layer->regions())
                    if (k < layerm->sublayer_perimeters.size())
                        for (const ExtrusionEntity *ee : layerm->sublayer_perimeters[k].entities)
                            collect_pass_fill(ee, fill);
                for (const Polyline &p : fill) {
                    if (p.size() < 3)
                        continue;
                    // A loop comes back to where it started; a line across the ring does not.
                    const double span = unscale<double>((p.last_point() - p.first_point()).cast<double>().norm());
                    (span < 0.5 ? closed : open) ++;
                }
            }
        INFO(pattern << ": " << closed << " closed fill paths, " << open << " open ones");
        REQUIRE(closed + open > 20);
        return double(closed) / double(closed + open);
    };

    CHECK(closed_fraction("concentric") > 0.75);
    CHECK(closed_fraction("rectilinear") < 0.25);
}

TEST_CASE("A scarf seam inside a pass ramps within that pass", "[WallSublayers]")
{
    // The passes own the outer wall, so disabling scarf seams inside them silently dropped the
    // setting from the only walls it applies to. A scarf ramps over the height of the path it is on,
    // and a pass path is a sub-layer tall, so it stays inside its own pass instead of cutting into
    // the one beneath - the same relationship an ordinary scarf has with the layer below it.
    DynamicPrintConfig config = base_config("0.05", "arachne");
    config.set_deserialize_strict({{"seam_slope_type", "external"}, {"seam_slope_start_height", "50%"}});
    const std::string gcode = slice({cube(20)}, config);
    REQUIRE(!gcode.empty());

    // Z values seen while extruding, per layer, without collapsing duplicates.
    std::vector<std::vector<double>> layers;
    GCodeReader                      reader;
    reader.parse_buffer(gcode, [&layers](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos) {
            layers.emplace_back();
            return;
        }
        if (!layers.empty() && line.extruding(self) && line.dist_XY(self) > 0)
            layers.back().emplace_back(self.z());
    });
    REQUIRE(layers.size() > 5);

    // A scarf has to actually be emitted, or the test proves nothing: Z takes values off the
    // sub-layer grid as the ramp climbs.
    int off_grid = 0;
    for (size_t i = 3; i < layers.size(); ++i)
        for (const double z : layers[i])
            if (std::abs(std::remainder(z, 0.05)) > 1e-4)
                ++off_grid;
    INFO(off_grid << " extruding moves at a ramped Z");
    CHECK(off_grid > 0);

    // And it must never dip a whole sub-layer below the pass it is ramping within, which is what
    // would put the nozzle into the pass underneath.
    for (size_t i = 3; i < layers.size(); ++i) {
        double high = 0.;
        for (const double z : layers[i]) {
            high = std::max(high, z);
            INFO("layer " << i << " z " << z << " after reaching " << high);
            REQUIRE(z > high - 0.05 - EPSILON);
        }
    }
}

TEST_CASE("Sub-layered walls stay joined to the walls the layer keeps", "[WallSublayers]")
{
    // The layer's own walls are generated from the column solid for the whole layer height, which on a
    // wall that flares is the contour at the bottom of the layer. Every pass sits further out than
    // that by the flare over one sub-layer, so without something in between the outer wall stack
    // stands free of the rest of the layer, bonded to nothing, and prints as a rough free-standing
    // ribbon. Measured as the closest approach between the two, which must be within touching
    // distance of one extrusion for the band to be joined on at all.
    // A sphere's lower half flares outward with Z exactly as a hull does, so each pass's contour sits
    // outside the one below it by the flare over one sub-layer.
    Print print;
    Model model;
    init_print({mesh(TestMesh::sphere_50mm)}, print, model, {{"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "3"}, {"skirt_loops", "0"},
        {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"}});
    print.process();

    std::function<void(const ExtrusionEntity*, Points&)> pts =
        [&](const ExtrusionEntity *ee, Points &out) {
            if (const auto *c = dynamic_cast<const ExtrusionEntityCollection*>(ee)) {
                for (const ExtrusionEntity *ch : c->entities) pts(ch, out);
            } else {
                Polylines pl; ee->collect_polylines(pl);
                for (const Polyline &p : pl)
                    for (const Point &q : p.points) out.push_back(q);
            }
        };

    const auto &layers = print.objects().front()->layers();
    std::vector<double> gaps;
    int no_tread = 0, with_tread = 0;
    for (size_t li = 10; li < std::min<size_t>(layers.size(), 60); li += 5) {
        const Layer *layer = layers[li];
        Points core;
        for (const LayerRegion *lr : layer->regions())
            for (const ExtrusionEntity *ee : lr->perimeters.entities) pts(ee, core);
        if (core.empty()) continue;
        for (size_t k = 0; k < layer->wall_sub_slices().size(); ++ k) {
            Points band; bool tread = false;
            for (const LayerRegion *lr : layer->regions())
                if (k < lr->sublayer_perimeters.size()) {
                    for (const ExtrusionEntity *ee : lr->sublayer_perimeters[k].entities) {
                        pts(ee, band);
                        std::function<void(const ExtrusionEntity*)> chk = [&](const ExtrusionEntity *e) {
                            if (const auto *c = dynamic_cast<const ExtrusionEntityCollection*>(e)) {
                                for (const ExtrusionEntity *ch : c->entities) chk(ch);
                            } else if (e->role() == erSolidInfill) tread = true;
                        };
                        chk(ee);
                    }
                }
            if (band.empty()) continue;
            (tread ? with_tread : no_tread) ++;
            double best = std::numeric_limits<double>::max();
            for (size_t i = 0; i < band.size(); i += 3)
                for (size_t j = 0; j < core.size(); j += 3)
                    best = std::min(best, (band[i] - core[j]).cast<double>().norm());
            gaps.push_back(unscale<double>(best));
        }
    }
    std::sort(gaps.begin(), gaps.end());
    REQUIRE(! gaps.empty());
    const double median = gaps[gaps.size() / 2];
    INFO("passes sampled " << gaps.size() << " (with tread " << with_tread << ", without " << no_tread
         << ")  closest band-to-core centreline distance: min " << gaps.front()
         << "mm  median " << median << "mm  max " << gaps.back() << "mm");
    // Two lines of the default width touch at about 0.42mm between their centres. Before the join was
    // printed this was 0.58mm - a void right behind the outer wall over most of the surface.
    CHECK(median < 0.45);
    CHECK(gaps.front() < 0.42);
}

TEST_CASE("A sub-layered thin feature is covered end to end", "[WallSublayers]")
{
    // A railing, a mast, a chimney wall: too thin for the layer's own run to keep a wall of its own,
    // because the outermost loops all belong to the band. The band must then cover the feature
    // outright. Standing it off a strip of interior that nothing else prints leaves that strip as a
    // void down the middle of the feature - which comes out as the wall splitting into two loops with
    // a gap between them.
    // Swept, because the strip only appears over the range of thicknesses where what is left inside
    // the band is too narrow for a wall but not narrow enough to have closed up on its own.
    const double thickness = GENERATE(1.6, 2.0, 2.4, 2.8, 3.2);
    INFO("feature " << thickness << "mm thick");
    Print print; Model model;
    init_print({make_cube(thickness, 18., 10.)}, print, model, {{"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"}, {"wall_loops", "3"}, {"skirt_loops", "0"},
        {"wall_generator", "arachne"}, {"wall_sublayer_height", "0.05"},
        {"wall_sublayer_loops", "2"}});
    print.process();

    double worst = 0.;
    int    passes = 0;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->id() < 3 || layer->wall_sub_slices().empty())
            continue;
        Polygons core;
        for (const LayerRegion *lr : layer->regions()) {
            append(core, lr->perimeters.polygons_covered_by_width(float(SCALED_EPSILON)));
            append(core, lr->fills.polygons_covered_by_width(float(SCALED_EPSILON)));
            append(core, lr->thin_fills.polygons_covered_by_width(float(SCALED_EPSILON)));
        }
        for (size_t k = 0; k < layer->wall_sub_slices().size(); ++ k) {
            Polygons covered = core;
            for (const LayerRegion *lr : layer->regions())
                if (k < lr->sublayer_perimeters.size())
                    append(covered, lr->sublayer_perimeters[k].polygons_covered_by_width(float(SCALED_EPSILON)));
            const ExPolygons slice = layer->wall_sub_slices()[k].merged;
            if (slice.empty())
                continue;
            double total = 0., bare = 0.;
            for (const ExPolygon &e : slice) total += e.area();
            for (const ExPolygon &e : diff_ex(slice, union_ex(covered))) bare += e.area();
            if (total > 0.) { worst = std::max(worst, bare / total); ++ passes; }
        }
    }
    REQUIRE(passes > 20);
    INFO("worst uncovered fraction of a sub-slice: " << worst);
    CHECK(worst < 0.05);
}

TEST_CASE("Every object keeps printing when another is on the plate", "[WallSublayers]")
{
    // Two objects of differing height, sub-layered. Each must keep printing something on every layer
    // it reaches: an object that falls silent for a stretch of layers has vanished from the print.
    TriangleMesh shortish = make_cube(20., 20., 12.);
    TriangleMesh tall     = make_cube(20., 20., 30.);
    tall.translate(40., 0., 0.);

    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    cfg.set_deserialize_strict({{"layer_height", "0.3"}, {"initial_layer_print_height", "0.2"},
        {"wall_loops", "3"}, {"skirt_loops", "0"}, {"wall_generator", "arachne"},
        {"wall_sublayer_height", "0.075"}, {"wall_sublayer_loops", "2"}});
    std::vector<TriangleMesh> meshes; meshes.push_back(shortish); meshes.push_back(tall);
    Print print; Model model;
    init_print(std::move(meshes), print, model, cfg, nullptr, false);
    print.process();

    REQUIRE(print.objects().size() == 2);
    for (const PrintObject *obj : print.objects()) {
        int silent = 0;
        for (const Layer *layer : obj->layers()) {
            if (layer->id() < 2) continue;
            size_t n = 0;
            for (const LayerRegion *lr : layer->regions()) {
                n += lr->perimeters.entities.size() + lr->thin_fills.entities.size();
                for (const auto &pass : lr->sublayer_perimeters) n += pass.entities.size();
            }
            if (n == 0) ++ silent;
        }
        INFO("an object has " << silent << " layers printing nothing");
        CHECK(silent == 0);
    }
}

TEST_CASE("Sub-layered walls open and close their object labels in pairs", "[WallSublayers]")
{
    // Each pass of each instance writes a "printing object" marker of its own. Left unmatched, every
    // one of them opens an object that is never closed, and a reader of the G-code hands everything
    // that follows to whichever object was opened last.
    DynamicPrintConfig config = base_config("0.05", "arachne");
    config.set_deserialize_strict({{"gcode_label_objects", "1"}});
    const std::string gcode = slice({cube(20)}, config);
    REQUIRE(! gcode.empty());

    int open = 0, close = 0, worst = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        const std::string_view c = line.comment();
        if (c.rfind(" printing object", 0) == 0) { ++ open; worst = std::max(worst, open - close); }
        else if (c.rfind(" stop printing object", 0) == 0) ++ close;
    });
    INFO(open << " opened, " << close << " closed, deepest " << worst);
    REQUIRE(open > 10);
    CHECK(open == close);
    // Never more than one object open at a time.
    CHECK(worst == 1);
}

TEST_CASE("A sub-layered hollow shell keeps its walls all the way up", "[WallSublayers]")
{
    // The wall of a sphere is a ring standing on the ring below it, and it is supported: there is
    // material directly under the thread. Judged instead by how much of the area the ring bounds is
    // filled, it reads as airborne - what a ring encircles is mostly the void inside the shell - and
    // the pass's outer wall is thrown away. It cost the outer wall of every pass over a third of the
    // height of an ellipsoid shell, which showed in the preview as the shell reduced to a bare rim.
    TriangleMesh outer = make_sphere(20., 1.);
    TriangleMesh inner = make_sphere(18.3, 1.);
    MeshBoolean::cgal::minus(outer, inner);
    outer.translate(0., 0., 20.);

    Print print; Model model;
    init_print({outer}, print, model, {{"layer_height", "0.3"}, {"initial_layer_print_height", "0.2"},
        {"wall_loops", "3"}, {"skirt_loops", "0"}, {"wall_generator", "arachne"},
        {"wall_sublayer_height", "0.075"}, {"wall_sublayer_loops", "2"}});
    print.process();

    std::vector<double> band;
    for (const Layer *layer : print.objects().front()->layers()) {
        // The sphere's crown and its footprint shrink to nothing, so only the body is measured.
        if (layer->print_z < 6. || layer->print_z > 34. || layer->wall_sub_slices().empty()) continue;
        double len = 0.;
        for (const LayerRegion *lr : layer->regions())
            for (const auto &pass : lr->sublayer_perimeters) {
                Polylines pl; pass.collect_polylines(pl);
                for (const Polyline &q : pl) len += unscale<double>(q.length());
            }
        band.push_back(len);
    }
    REQUIRE(band.size() > 30);
    std::vector<double> sorted = band;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted[sorted.size() / 2];
    REQUIRE(median > 100.);
    // No layer may lose most of its band: the shell's wall is supported at every height.
    const double worst = *std::min_element(band.begin(), band.end());
    INFO("median band " << median << "mm, worst layer " << worst << "mm");
    CHECK(worst > 0.4 * median);
}

// Sub-layer wall length with no part of its thread over anything that was there when it printed: the
// layer below for the first pass, and after that whatever the passes under it actually laid down. The
// model re-sliced at a pass's own height is not that - it has the whole ceiling of a closing cavity in
// it, which nothing covers until the layer's own run at print_z, after every pass has gone.
double airborne_pass_wall_length(const Print &print)
{
    double len = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->lower_layer == nullptr || layer->wall_sub_slices().empty())
            continue;
        Polygons laid = to_polygons(layer->lower_layer->lslices);
        for (size_t k = 0; k < layer->wall_sub_slices().size(); ++ k) {
            Polygons pass_covered;
            std::function<void(const ExtrusionEntity *)> walk = [&](const ExtrusionEntity *entity) {
                if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity)) {
                    for (const ExtrusionEntity *child : collection->entities)
                        walk(child);
                    return;
                }
                const Polygons covered = entity->polygons_covered_by_width(10.f);
                if (! covered.empty() && intersection(covered, laid).empty())
                    len += unscaled(entity->length());
                append(pass_covered, covered);
            };
            for (const LayerRegion *region : layer->regions())
                if (k < region->sublayer_perimeters.size())
                    walk(&region->sublayer_perimeters[k]);
            append(laid, pass_covered);
        }
    }
    return len;
}

// A cavity closing over fast enough that a pass's ring clears the ring below it: the hole shrinks a
// wall's width and more per pass, so no part of a pass's thread is over anything a pass below laid
// down. Taking the model re-sliced at the pass's own height for material carried these rings up the
// ceiling one pass at a time, each hanging behind the one before it. Reported from the crown of a
// dome, where the passes drew circles in mid air and the layer bridged onto them.
TEST_CASE("A sub-layer pass stands on what the pass below printed, not on the model", "[WallSublayers]")
{
    const auto wall_generator = GENERATE("arachne", "classic");
    INFO("wall_generator=" << wall_generator);

    TriangleMesh block = make_cube(34., 34., 8.);
    TriangleMesh cone  = make_cone(14., 2.);
    cone.translate(17., 17., 3.);
    MeshBoolean::cgal::minus(block, cone);

    Print print;
    Model model;
    init_print({block}, print, model, {{"layer_height", "0.3"}, {"initial_layer_print_height", "0.2"},
        {"wall_loops", "3"}, {"skirt_loops", "0"}, {"wall_generator", wall_generator},
        {"wall_sublayer_height", "0.075"}, {"wall_sublayer_loops", "2"}});
    print.process();

    const double airborne = airborne_pass_wall_length(print);
    INFO("sub-layer wall over nothing: " << airborne << "mm");
    CHECK(airborne < 1.);
}

TEST_CASE("A sub-layered pass does not ring a cavity that is closing over", "[WallSublayers]")
{
    // A conical void closing under a ceiling. Its wall rises 2mm while the hole closes 14, so the
    // hole shrinks 0.52mm per 0.075mm pass - more than a wall is wide, so each pass's ring is clear
    // of the ring below it with no part of the thread over material at all. A wall's width of reach
    // still read it as carried. The layer's own pass bridges that void at print_z, with a bridge flow
    // and the fan on; a pass cannot, and the rings it drew were left drooping in the bridge's way.
    // Reported from the crown of a dome, where every pass drew a circle in mid air.
    TriangleMesh block = make_cube(34., 34., 8.);
    TriangleMesh cone  = make_cone(14., 2.);
    cone.translate(17., 17., 3.);
    MeshBoolean::cgal::minus(block, cone);

    Print print; Model model;
    init_print({block}, print, model, {{"layer_height", "0.3"}, {"initial_layer_print_height", "0.2"},
        {"wall_loops", "3"}, {"skirt_loops", "0"}, {"wall_generator", "arachne"},
        {"wall_sublayer_height", "0.075"}, {"wall_sublayer_loops", "2"}});
    print.process();

    const float width = float(print.objects().front()->layers().front()->regions().front()->flow(frExternalPerimeter).scaled_width());
    const Vec2d centre = unscale(get_extents(print.objects().front()->layers().front()->lslices).center());
    double over_model = 0., over_below = 0., total = 0., crept = 0.;
    std::vector<double> reach;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->print_z < 3.2 || layer->print_z > 5. || layer->wall_sub_slices().empty())
            continue;
        for (size_t k = 0; k < layer->wall_sub_slices().size(); ++ k) {
            const ExPolygons *support = layer->wall_sublayer_support(layer->wall_sub_slices().size(), k);
            if (support == nullptr)
                continue;
            // The voids the model below encloses.
            ExPolygons voids;
            for (const ExPolygon &expoly : *support)
                for (const Polygon &hole : expoly.holes) {
                    voids.emplace_back(hole);
                    voids.back().contour.reverse();
                }
            Polylines walls;
            for (const LayerRegion *region : layer->regions())
                if (k < region->sublayer_perimeters.size())
                    region->sublayer_perimeters[k].collect_polylines(walls);
            for (const Polyline &q : walls)
                total += unscale<double>(q.length());
            // Half a wall's width inside the void: past that, no part of the thread is over material
            // at all.
            for (const Polyline &q : intersection_pl(walls, offset_ex(voids, - 0.5f * width)))
                over_model += unscale<double>(q.length());

            // And the same against what the layer below left: its own run reaches the columns solid
            // for its whole height, and no pass may come down inside the void that leaves. Judged
            // per layer, so that refusing the ceiling does not simply start the stair again one
            // layer up.
            if (layer->lower_layer != nullptr) {
                ExPolygons core;
                bool       core_set = false;
                for (const WallSubSlice &sub : layer->lower_layer->wall_sub_slices()) {
                    if (sub.merged.empty()) continue;
                    core = core_set ? intersection_ex(core, sub.merged) : sub.merged;
                    core_set = true;
                }
                ExPolygons below_voids;
                for (const ExPolygon &expoly : core_set ? core : layer->lower_layer->lslices)
                    for (const Polygon &hole : expoly.holes) {
                        below_voids.emplace_back(hole);
                        below_voids.back().contour.reverse();
                    }
                for (const Polyline &q : intersection_pl(walls, offset_ex(below_voids, - 0.5f * width)))
                    over_below += unscale<double>(q.length());
            }

            // How far into the cone each pass reaches. A ring that steps inward pass by pass is the
            // stair a closing ceiling makes: every step lands on the step the pass below was not
            // allowed to print, and the model says every one of them is carried.
            double closest = std::numeric_limits<double>::max();
            for (const Polyline &q : walls)
                for (const Point &pt : q.points)
                    closest = std::min(closest, (unscale(pt) - centre).norm());
            reach.resize(std::max(reach.size(), k + 1), std::numeric_limits<double>::max());
            reach[k] = std::min(reach[k], closest);
        }
        for (size_t k = 1; k < reach.size(); ++ k)
            crept = std::max(crept, reach.front() - reach[k]);
        reach.clear();
    }

    // The ceiling the passes refuse is the layer's own to print, walls and all. Measured at the edge
    // of the cone: with the band dropped there and the layer still handing its outermost walls over,
    // the first thread sits two wall spacings inside the contour and the ceiling comes out as one
    // ring with a gap around it.
    std::vector<double> edge_gaps;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->print_z < 3.2 || layer->print_z > 5. || layer->wall_sub_slices().empty())
            continue;
        Polylines threads;
        for (const LayerRegion *region : layer->regions()) {
            region->perimeters.collect_polylines(threads);
            for (const auto &pass : region->sublayer_perimeters)
                pass.collect_polylines(threads);
        }
        if (threads.empty())
            continue;
        // The contour the layer actually prints to is its own run's, which is confined to the columns
        // solid for the whole layer height - the intersection of the sub-slices - not the mid-height
        // slice.
        ExPolygons core;
        bool       core_set = false;
        for (const WallSubSlice &sub : layer->wall_sub_slices()) {
            if (sub.merged.empty()) continue;
            core = core_set ? intersection_ex(core, sub.merged) : sub.merged;
            core_set = true;
        }
        double worst = 0.;
        for (const ExPolygon &expoly : core)
            for (const Polygon &hole : expoly.holes)
                for (size_t i = 0; i < hole.points.size(); i += 8) {
                    double best = std::numeric_limits<double>::max();
                    for (const Polyline &q : threads)
                        for (const Point &pt : q.points)
                            best = std::min(best, (pt - hole.points[i]).cast<double>().norm());
                    worst = std::max(worst, unscale<double>(best));
                }
        edge_gaps.push_back(worst);
    }
    std::sort(edge_gaps.begin(), edge_gaps.end());
    REQUIRE(! edge_gaps.empty());
    const double edge_gap = edge_gaps[edge_gaps.size() / 2];
    // Half a wall's width is the outermost thread sitting on the contour; a wall and a half is the
    // two the band was handed and never printed.
    INFO("furthest the cone's edge is from a thread, median over layers: " << edge_gap << "mm");
    CHECK(edge_gap < 1.5 * unscale<double>(width));

    // The walls of the block itself are still printed at every pass - the cone must not take the
    // feature with it.
    INFO(over_model << "mm of " << total << "mm of sub-layer wall over the model's void, " << over_below
         << "mm inside the void the layer below left; the passes creep " << crept << "mm into the cone "
         "within a layer");
    REQUIRE(total > 500.);
    CHECK(over_model < 0.01 * total);
    CHECK(over_below < 0.01 * total);
    // One wall's width of movement is the band itself following the model; a stair is three times that
    // by the end of a layer.
    CHECK(crept < unscale<double>(width));
}




// -----------------------------------------------------------------------------------------------
// Sub-layered walls together with other features: modifiers, layer ranges, negative volumes,
// painting, wall filaments, seams, z-hop and sequential printing.

namespace {

struct ExtraVolume
{
    TriangleMesh                                mesh;
    ModelVolumeType                             type;
    std::vector<ConfigBase::SetDeserializeItem> config;
};

// One object made of `base` plus further volumes with settings of their own. `prepare` runs on the
// object before it is applied, to paint it or to give it layer ranges.
void init_object(Print &print, Model &model, const DynamicPrintConfig &config, TriangleMesh base,
                 std::vector<ExtraVolume> extra = {}, const std::function<void(ModelObject &)> &prepare = {})
{
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    full.apply(config);
    full.set_key_value("gcode_comments", new ConfigOptionBool(true));
    ModelObject *object = model.add_object();
    object->name = "object.stl";
    object->add_volume(std::move(base));
    for (ExtraVolume &v : extra) {
        ModelVolume       *volume = object->add_volume(std::move(v.mesh), v.type);
        DynamicPrintConfig overrides;
        for (const ConfigBase::SetDeserializeItem &item : v.config)
            overrides.set_deserialize_strict(item.opt_key, item.opt_value);
        volume->config.apply(overrides);
    }
    object->add_instance();
    if (prepare)
        prepare(*object);
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, full);
    print.set_status_silent();
}

// 0.2mm layers with z_hop and the skirt off, so a recorded Z is always a printing Z.
DynamicPrintConfig plain_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config;
    config.set_deserialize_strict({{"layer_height", "0.2"},
                                   {"initial_layer_print_height", "0.2"},
                                   {"wall_loops", "3"},
                                   {"z_hop", "0"},
                                   {"skirt_loops", "0"}});
    config.set_deserialize_strict(extra);
    return config;
}

// Whether z is the print_z of a layer of the given height, rather than a pass below it.
bool on_layer_top(double z, double layer_height)
{
    const double steps = z / layer_height;
    return std::abs(steps - std::round(steps)) < 1e-3;
}

// Paint the facets of the volume's mesh lying in its +X face with `state`, or every facet with
// `whole` set.
void paint(ModelVolume &volume, FacetsAnnotation &annotation, EnforcerBlockerType state, bool whole)
{
    const indexed_triangle_set &its   = volume.mesh().its;
    const float                 max_x = volume.mesh().bounding_box().max.x();
    TriangleSelector            selector(volume.mesh());
    for (int i = 0; i < int(its.indices.size()); ++ i) {
        bool on_face = true;
        for (int k = 0; k < 3; ++ k)
            on_face = on_face && its.vertices[its.indices[i][k]].x() > max_x - 1e-3f;
        if (whole || on_face)
            selector.set_facet(i, state);
    }
    annotation.set(selector);
}

} // namespace

TEST_CASE("A modifier that sub-layers part of an object leaves the rest of the object alone", "[WallSublayers]")
{
    // The object asks for no sub-layers. A modifier over its left 4mm splits its 0.2mm layers by 0.05mm
    // into 4 passes there. Only walls inside the modifier may print at a pass height.
    TriangleMesh modifier = make_cube(6., 24., 6.);
    modifier.translate(-2., -2., -1.);
    Print print;
    Model model;
    init_object(print, model, plain_config(), make_cube(20., 20., 4.),
                {{std::move(modifier), ModelVolumeType::PARAMETER_MODIFIER, {{"wall_sublayer_height", "0.05"}}}});
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE(! gcode.empty());

    BoundingBoxf object_box;
    double       pass_max_x = -1e10;
    size_t       pass_moves = 0;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) {
        object_box.merge(Vec2d(self.x(), self.y()));
        if (! on_layer_top(self.z(), 0.2)) {
            pass_max_x = std::max(pass_max_x, double(self.x()));
            ++ pass_moves;
        }
    });
    // The modifier's own walls are sub-layered.
    REQUIRE(pass_moves > 0);
    INFO("object spans x " << object_box.min.x() << " .. " << object_box.max.x() << ", passes reach x " << pass_max_x);
    CHECK(pass_max_x < object_box.min.x() + 4. + 1.);
}

TEST_CASE("A region gets the sub-layer height it asks for rather than another region's", "[WallSublayers]")
{
    // The object splits its 0.2mm layers by 0.05mm into 4 passes; a modifier over its left 4mm asks for
    // 0.1mm, so 2 passes, or for 0.025mm, so 8. Each prints its outer wall at exactly its own pass
    // heights, and the two interleave without the nozzle ever moving down within a layer.
    const auto [modifier_height, modifier_step] = GENERATE(std::pair{"0.1", 0.1}, std::pair{"0.025", 0.025});
    INFO("modifier wall_sublayer_height " << modifier_height);
    TriangleMesh modifier = make_cube(6., 24., 6.);
    modifier.translate(-2., -2., -1.);
    Print print;
    Model model;
    init_object(print, model, plain_config({{"wall_sublayer_height", "0.05"}}), make_cube(20., 20., 4.),
                {{std::move(modifier), ModelVolumeType::PARAMETER_MODIFIER, {{"wall_sublayer_height", modifier_height}}}});
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE(! gcode.empty());

    BoundingBoxf object_box;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) { object_box.merge(Vec2d(self.x(), self.y())); });
    std::set<double> modifier_zs, object_zs;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) {
        if (self.z() < 0.2 + EPSILON)
            return;
        if (self.x() < object_box.min.x() + 3.)
            modifier_zs.insert(self.z());
        else if (self.x() > object_box.min.x() + 6.)
            object_zs.insert(self.z());
    });
    REQUIRE(! modifier_zs.empty());
    REQUIRE(! object_zs.empty());
    for (const double z : modifier_zs) {
        INFO("outer wall inside the modifier at z " << z);
        CHECK(on_layer_top(z, modifier_step));
    }
    for (const double z : object_zs) {
        INFO("outer wall outside the modifier at z " << z);
        CHECK(on_layer_top(z, 0.05));
    }
    // Both splits are actually used: more pass heights inside the finer region than the coarser has.
    CHECK(modifier_zs.size() != object_zs.size());

    for (const auto &layer : extruding_zs_per_layer(gcode))
        for (size_t k = 1; k < layer.size(); ++ k) {
            INFO("step " << k << ": " << layer[k - 1] << " -> " << layer[k]);
            REQUIRE(layer[k] >= layer[k - 1] - EPSILON);
        }
}

TEST_CASE("Sub-layered walls follow a layer range whose boundary falls inside a layer", "[WallSublayers]")
{
    // Two layer ranges differing only in infill density, split at 2.1mm - the middle of the 2.0 .. 2.2
    // layer. Every pass of every layer above the first still prints the whole 20mm outline.
    const DynamicPrintConfig config = plain_config({{"wall_sublayer_height", "0.05"}});
    Print print;
    Model model;
    init_object(print, model, config, make_cube(20., 20., 4.), {}, [](ModelObject &object) {
        for (const auto &[range, density] : {std::pair{t_layer_height_range{0., 2.1}, "15%"}, std::pair{t_layer_height_range{2.1, 4.}, "40%"}}) {
            DynamicPrintConfig range_config;
            range_config.set_deserialize_strict({{"layer_height", "0.2"}, {"sparse_infill_density", density}});
            object.layer_config_ranges[range].assign_config(std::move(range_config));
        }
    });
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 10);
    for (size_t i = 1; i < layers.size(); ++ i) {
        std::ostringstream zs;
        for (const double z : distinct_sorted(layers[i]))
            zs << z << " ";
        INFO("layer " << i << " prints at z " << zs.str());
        CHECK(distinct_sorted(layers[i]).size() == 4);
    }
    for (const auto &[z, box] : bbox_of_type_by_z(gcode, "Outer wall")) {
        INFO("outer wall at z " << z);
        CHECK(box.size().x() > 19.);
    }
}

TEST_CASE("Sub-layered walls do not print inside a negative volume", "[WallSublayers]")
{
    // A 6mm square shaft cut through the middle of the 20mm block by a negative volume.
    TriangleMesh shaft = make_cube(6., 6., 6.);
    shaft.translate(7., 7., -1.);
    Print print;
    Model model;
    init_object(print, model, plain_config({{"wall_sublayer_height", "0.05"}}), make_cube(20., 20., 4.),
                {{std::move(shaft), ModelVolumeType::NEGATIVE_VOLUME, {}}});
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE(! gcode.empty());

    BoundingBoxf object_box;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) { object_box.merge(Vec2d(self.x(), self.y())); });
    // The shaft, 1mm in from its sides: no line of any kind runs there.
    const BoundingBoxf inside(object_box.min + Vec2d(8., 8.), object_box.min + Vec2d(12., 12.));
    size_t pass_moves = 0, inside_moves = 0;
    for (const char *type : {"Outer wall", "Inner wall", "Gap infill", "Internal solid infill", "Sparse infill"})
        for_each_move_of_type(gcode, type, [&](const GCodeReader &self, int) {
            if (! on_layer_top(self.z(), 0.2))
                ++ pass_moves;
            if (inside.contains(Vec2d(self.x(), self.y())))
                ++ inside_moves;
        });
    REQUIRE(pass_moves > 0);
    CHECK(inside_moves == 0);
}

TEST_CASE("Painted fuzzy skin reaches the sub-layered outer wall", "[WallSublayers]")
{
    // The +X face of the block is painted fuzzy. Fuzzy skin breaks that face's outer wall into a line
    // per fuzzy_skin_point_distance, so painted, a pass's outer wall takes many more moves.
    const auto pass_moves = [](const char *sublayer_height, bool painted, bool at_pass_heights) {
        Print print;
        Model model;
        // fuzzy_skin none is "Painted only"; the default disables painting as well.
        init_object(print, model, plain_config({{"wall_sublayer_height", sublayer_height}, {"fuzzy_skin", "none"}}), make_cube(20., 20., 4.), {},
                    [painted](ModelObject &object) {
                        if (painted)
                            paint(*object.volumes.front(), object.volumes.front()->fuzzy_skin_facets, EnforcerBlockerType::FUZZY_SKIN, false);
                    });
        size_t moves = 0;
        for_each_move_of_type(Slic3r::Test::gcode(print), "Outer wall", [&](const GCodeReader &self, int) {
            if (self.z() > 0.2 + EPSILON && on_layer_top(self.z(), 0.2) != at_pass_heights)
                ++ moves;
        });
        return moves;
    };
    // The painting itself works: without sub-layers it multiplies the outer wall's moves.
    const size_t plain_off = pass_moves("0", false, false), painted_off = pass_moves("0", true, false);
    INFO("full-height outer wall moves: " << plain_off << " plain, " << painted_off << " painted");
    REQUIRE(painted_off > 2 * plain_off);

    const size_t plain_on = pass_moves("0.05", false, true), painted_on = pass_moves("0.05", true, true);
    INFO("pass outer wall moves: " << plain_on << " plain, " << painted_on << " painted");
    REQUIRE(plain_on > 0);
    CHECK(painted_on > 2 * plain_on);
}

TEST_CASE("Colour painting does not block sub-layered walls on a single-filament printer", "[WallSublayers]")
{
    // With one filament the colour painting is never applied - slicing skips the segmentation - so
    // there is nothing for sub-layered walls to conflict with.
    Print print;
    Model model;
    init_object(print, model, plain_config({{"wall_sublayer_height", "0.05"}, {"layer_change_gcode", "G92 E0"}}), make_cube(20., 20., 4.), {},
                [](ModelObject &object) {
                    paint(*object.volumes.front(), object.volumes.front()->mmu_segmentation_facets, EnforcerBlockerType::Extruder2, true);
                });
    REQUIRE(print.config().filament_diameter.size() == 1);
    REQUIRE(model.objects.front()->is_mm_painted());
    const StringObjectException error = print.validate();
    INFO("validate: " << error.string);
    CHECK(error.string.empty());
}

TEST_CASE("Colour painted outer walls in another filament print whole after the passes", "[WallSublayers]")
{
    // The +X face of the block is painted in filament 2. Only one filament can print below a layer's
    // print_z without the nozzle coming down beside something already standing a full layer high, so the
    // object's own filament 1 prints its outer wall in passes, and wherever the painting reaches the
    // outer wall that wall prints once in filament 2, at the full layer height, after them. Which layers
    // the painting reaches is the colour segmentation's business, so it is taken from the same print
    // without sub-layers: per layer, the painted face's outer wall prints in the same filaments.
    const auto slice_painted = [](const char *sublayer_height, std::vector<StringObjectException> *warnings) {
        Print print;
        Model model;
        init_object(print, model,
                    multifilament_config(2, {{"layer_height", "0.2"},
                                             {"initial_layer_print_height", "0.2"},
                                             {"wall_loops", "3"},
                                             {"z_hop", "0"},
                                             {"skirt_loops", "0"},
                                             {"enable_prime_tower", "0"},
                                             {"layer_change_gcode", "G92 E0"},
                                             {"wall_sublayer_height", sublayer_height}}),
                    make_cube(20., 20., 4.), {}, [](ModelObject &object) {
                        paint(*object.volumes.front(), object.volumes.front()->mmu_segmentation_facets, EnforcerBlockerType::Extruder2, false);
                    });
        const StringObjectException error = print.validate(warnings);
        INFO("validate: " << error.string);
        REQUIRE(error.string.empty());
        return Slic3r::Test::gcode(print);
    };
    // The filaments printing the painted face's outer wall, by the top of the layer they belong to.
    const auto painted_face_tools = [](const std::string &gcode) {
        BoundingBoxf object_box;
        for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) { object_box.merge(Vec2d(self.x(), self.y())); });
        std::map<long, std::set<int>> tools;
        for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int tool) {
            if (self.x() > object_box.max.x() - 0.3)
                tools[std::lround(std::ceil(self.z() / 0.2 - 1e-3))].insert(tool);
        });
        return tools;
    };

    const std::string plain = slice_painted("0", nullptr);
    REQUIRE(! plain.empty());
    const auto plain_tools = painted_face_tools(plain);
    // The painting reaches the outer wall on most of the 20 layers of 0.2mm in 4mm.
    REQUIRE(std::count_if(plain_tools.begin(), plain_tools.end(), [](const auto &l) { return l.second.count(1) > 0; }) > 10);

    std::vector<StringObjectException> warnings;
    const std::string gcode = slice_painted("0.05", &warnings);
    REQUIRE(! gcode.empty());
    CHECK(std::any_of(warnings.begin(), warnings.end(), [](const StringObjectException &w) {
        return w.opt_key == "wall_sublayer_height" && w.string.find("painted") != std::string::npos;
    }));
    const auto tools = painted_face_tools(gcode);
    for (const auto &[layer, plain_layer_tools] : plain_tools) {
        INFO("layer " << layer);
        CHECK(tools.count(layer) == 1);
        if (tools.count(layer) == 1)
            CHECK(tools.at(layer) == plain_layer_tools);
    }

    // The passes are all the object's own filament, and stay off the face wherever it is painted.
    BoundingBoxf object_box;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) { object_box.merge(Vec2d(self.x(), self.y())); });
    std::set<int> pass_tools;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int tool) {
        if (on_layer_top(self.z(), 0.2))
            return;
        pass_tools.insert(tool);
        if (self.x() > object_box.max.x() - 0.3) {
            const long layer = std::lround(std::ceil(self.z() / 0.2 - 1e-3));
            INFO("pass on the painted face at z " << self.z());
            CHECK(plain_tools.count(layer) == 1);
            if (plain_tools.count(layer) == 1)
                CHECK(plain_tools.at(layer).count(1) == 0);
        }
    });
    CHECK(pass_tools == std::set<int>{0});

    for (const auto &layer : extruding_zs_per_layer(gcode))
        for (size_t k = 1; k < layer.size(); ++ k) {
            INFO("step " << k << ": " << layer[k - 1] << " -> " << layer[k]);
            REQUIRE(layer[k] >= layer[k - 1] - EPSILON);
        }
}

TEST_CASE("Sub-layered walls keep Z rising within a layer when the walls use two filaments", "[WallSublayers]")
{
    // Outer walls with filament 2, everything else with filament 1. Whichever filament prints first in a
    // layer, nothing printed after it may come down below the layer's print_z it already reached.
    const auto loops = GENERATE("1", "2");
    const std::string gcode = slice({cube(20)}, multifilament_config(2, {{"layer_height", "0.2"},
                                                                          {"initial_layer_print_height", "0.2"},
                                                                          {"wall_loops", "3"},
                                                                          {"z_hop", "0"},
                                                                          {"skirt_loops", "0"},
                                                                          {"enable_prime_tower", "0"},
                                                                          {"outer_wall_filament_id", "2"},
                                                                          {"inner_wall_filament_id", "1"},
                                                                          {"sparse_infill_filament_id", "1"},
                                                                          {"solid_infill_filament_id", "1"},
                                                                          {"wall_sublayer_height", "0.05"},
                                                                          {"wall_sublayer_loops", loops}}));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    for (size_t i = 0; i < layers.size(); ++ i)
        for (size_t k = 1; k < layers[i].size(); ++ k) {
            INFO("wall_sublayer_loops " << loops << ", layer " << i << " step " << k << ": " << layers[i][k - 1] << " -> " << layers[i][k]);
            REQUIRE(layers[i][k] >= layers[i][k - 1] - EPSILON);
        }
}

TEST_CASE("Inner walls keep the inner wall filament when the outer walls are sub-layered", "[WallSublayers]")
{
    // wall_sublayer_loops 2 asks the passes to take the first inner wall too. The passes print with the
    // outer wall filament (tool 1), so with a separate inner wall filament (tool 0) that inner wall has
    // to stay out of them: every inner wall prints with tool 0, every pass's outer wall with tool 1.
    const std::string gcode = slice({cube(20)}, multifilament_config(2, {{"layer_height", "0.2"},
                                                                          {"initial_layer_print_height", "0.2"},
                                                                          {"wall_loops", "3"},
                                                                          {"z_hop", "0"},
                                                                          {"skirt_loops", "0"},
                                                                          {"enable_prime_tower", "0"},
                                                                          {"outer_wall_filament_id", "2"},
                                                                          {"inner_wall_filament_id", "1"},
                                                                          {"sparse_infill_filament_id", "1"},
                                                                          {"solid_infill_filament_id", "1"},
                                                                          {"wall_sublayer_height", "0.05"},
                                                                          {"wall_sublayer_loops", "2"}}));
    REQUIRE(! gcode.empty());

    std::set<int> inner_tools, pass_outer_tools;
    for_each_move_of_type(gcode, "Inner wall", [&](const GCodeReader &, int tool) { inner_tools.insert(tool); });
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int tool) {
        if (! on_layer_top(self.z(), 0.2))
            pass_outer_tools.insert(tool);
    });
    REQUIRE(! pass_outer_tools.empty());
    CHECK(pass_outer_tools == std::set<int>{1});
    CHECK(inner_tools == std::set<int>{0});
}

TEST_CASE("One object's sub-layered walls in two filaments are rejected", "[WallSublayers]")
{
    // Two touching parts of one object, their outer walls in filaments 1 and 2, both sub-layered.
    // Whichever filament goes second would bring its passes down beside the first part's walls, already
    // printed a full layer high, so this cannot be printed safely.
    TriangleMesh second = make_cube(20., 20., 4.);
    second.translate(20., 0., 0.);
    Print print;
    Model model;
    init_object(print, model, multifilament_config(2, {{"layer_height", "0.2"}, {"layer_change_gcode", "G92 E0"}, {"wall_sublayer_height", "0.05"}}),
                make_cube(20., 20., 4.), {{std::move(second), ModelVolumeType::MODEL_PART, {{"outer_wall_filament_id", "2"}}}});
    REQUIRE(model.objects.front()->volumes.size() == 2);
    const StringObjectException error = print.validate();
    INFO("validate: " << error.string);
    CHECK(error.string.find("Sub-layered") != std::string::npos);
}

namespace {

// Where an outer wall run starts - its seam.
struct SeamPoint
{
    double z, x, y;
};

// Every outer wall run's seam, in printing order.
std::vector<SeamPoint> outer_wall_starts(const std::string &gcode)
{
    std::vector<SeamPoint> starts;
    bool               in_outer = false, started = false;
    GCodeReader        reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (const size_t tag = line.comment().find("TYPE:"); tag != std::string_view::npos) {
            in_outer = line.comment().substr(tag + 5) == "Outer wall";
            started  = false;
            return;
        }
        if (in_outer && ! started && line.extruding(self) && line.dist_XY(self) > 0) {
            starts.push_back({self.z(), self.x(), self.y()});
            started = true;
        }
    });
    return starts;
}

} // namespace

TEST_CASE("Every sub-layered pass puts its seam where seam_position asks", "[WallSublayers]")
{
    // seam_position back: every outer wall loop, pass or not, starts on the back (+Y) face of the block.
    const std::string gcode = slice({cube(20)}, plain_config({{"wall_sublayer_height", "0.05"}, {"seam_position", "back"}}));
    REQUIRE(! gcode.empty());

    const std::map<double, BoundingBoxf> boxes = bbox_of_type_by_z(gcode, "Outer wall");
    size_t pass_starts = 0;
    for (const SeamPoint &start : outer_wall_starts(gcode)) {
        if (start.z <= 0.2 + EPSILON || on_layer_top(start.z, 0.2))
            continue;
        ++ pass_starts;
        INFO("outer wall pass at z " << start.z << " starts at y " << start.y << ", back face at y " << boxes.at(start.z).max.y());
        CHECK(start.y > boxes.at(start.z).max.y() - 1.);
    }
    REQUIRE(pass_starts > 0);
}

TEST_CASE("Every sub-layered pass puts its seam on a painted seam enforcer", "[WallSublayers]")
{
    // The +X face of the block is painted as a seam enforcer. The seam placer knows only the layer's own
    // walls; a pass's outer wall takes its seam from them, and still has to start on the painted face,
    // as the full-height outer wall does without sub-layers.
    const auto seam_starts = [](const char *sublayer_height) {
        Print print;
        Model model;
        init_object(print, model, plain_config({{"wall_sublayer_height", sublayer_height}}), make_cube(20., 20., 4.), {},
                    [](ModelObject &object) {
                        paint(*object.volumes.front(), object.volumes.front()->seam_facets, EnforcerBlockerType::ENFORCER, false);
                    });
        const std::string gcode = Slic3r::Test::gcode(print);
        BoundingBoxf object_box;
        for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) { object_box.merge(Vec2d(self.x(), self.y())); });
        std::vector<std::pair<double, bool>> out; // (z, starts on the painted face)
        for (const SeamPoint &start : outer_wall_starts(gcode))
            if (start.z > 0.2 + EPSILON)
                out.emplace_back(start.z, start.x > object_box.max.x() - 1.);
        return out;
    };
    // The control: without sub-layers the enforcer holds the outer wall's seam.
    for (const auto &[z, on_face] : seam_starts("0")) {
        INFO("full-height outer wall at z " << z);
        REQUIRE(on_face);
    }
    size_t pass_starts = 0;
    for (const auto &[z, on_face] : seam_starts("0.05")) {
        INFO("outer wall at z " << z);
        CHECK(on_face);
        pass_starts += ! on_layer_top(z, 0.2);
    }
    REQUIRE(pass_starts > 0);
}

TEST_CASE("A prime tower does not bring the nozzle down onto the object within a layer", "[WallSublayers]")
{
    // Two filaments, outer walls in filament 2, with a prime tower. The tower prints at the layer's
    // print_z on a tool change, away from the object, and the passes come down to their heights only
    // over the object - where, within a layer, Z still never drops.
    DynamicPrintConfig config = multifilament_config(2, {{"layer_height", "0.2"},
                                                         {"initial_layer_print_height", "0.2"},
                                                         {"wall_loops", "3"},
                                                         {"z_hop", "0"},
                                                         {"skirt_loops", "0"},
                                                         {"enable_prime_tower", "1"},
                                                         {"wipe_tower_x", "120"}, // inside the 200x200 test bed
                                                         {"wipe_tower_y", "120"},
                                                         {"outer_wall_filament_id", "2"},
                                                         {"inner_wall_filament_id", "1"},
                                                         {"sparse_infill_filament_id", "1"},
                                                         {"internal_solid_filament_id", "1"},
                                                         {"wall_sublayer_height", "0.05"}});
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    Print print;
    Model model;
    init_print({cube(20)}, print, model, config);
    // The first apply counts the filaments in use before it has built the regions that carry the
    // per-feature filaments, sees one, and turns the tower off. Applying again, as the GUI does on
    // the next change, counts both.
    print.apply(model, config);
    const std::string gcode = Slic3r::Test::gcode(print);
    // The passes are on: the outer wall lands at the 4 pass heights of each 0.2mm layer above the first.
    const std::set<double> wall_zs = zs_of_type(gcode, "Outer wall");
    REQUIRE(std::any_of(wall_zs.begin(), wall_zs.end(), [](double z) { return std::abs(z - 0.25) < 1e-4; }));
    // The tower prints at whole layer heights only, never at a pass height.
    const std::set<double> tower_zs = zs_of_type(gcode, "Prime tower");
    REQUIRE(! tower_zs.empty());
    for (double z : tower_zs) {
        INFO("prime tower at z " << z);
        CHECK_THAT(z - 0.2 * std::round(z / 0.2), Catch::Matchers::WithinAbs(0., 1e-4));
    }

    BoundingBoxf object_box;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int) { object_box.merge(Vec2d(self.x(), self.y())); });
    object_box.offset(1.);
    std::vector<std::vector<double>> layers;
    GCodeReader                      reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos)
            layers.emplace_back();
        else if (! layers.empty() && line.extruding(self) && line.dist_XY(self) > 0 && object_box.contains(Vec2d(self.x(), self.y())))
            layers.back().emplace_back(self.z());
    });
    REQUIRE(layers.size() > 2);
    for (size_t i = 0; i < layers.size(); ++ i)
        for (size_t k = 1; k < layers[i].size(); ++ k) {
            INFO("layer " << i << " step " << k << ": " << layers[i][k - 1] << " -> " << layers[i][k]);
            REQUIRE(layers[i][k] >= layers[i][k - 1] - EPSILON);
        }
}

TEST_CASE("Sub-layered walls extrude only at their pass heights with z-hop on", "[WallSublayers]")
{
    // z_hop lifts the nozzle for travel; every extrusion still lands at a pass height, 4 of them in a
    // 0.2mm layer split by 0.05mm.
    const std::string gcode = slice({cube(20)}, plain_config({{"wall_sublayer_height", "0.05"}, {"z_hop", "0.4"}}));
    REQUIRE(! gcode.empty());
    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    const std::vector<double> zs = distinct_sorted(layers[1]);
    REQUIRE(zs.size() == 4);
    for (int k = 0; k < 4; ++ k)
        CHECK_THAT(zs[k], Catch::Matchers::WithinAbs(0.2 + 0.05 * (k + 1), 1e-6));
}

TEST_CASE("Sub-layered walls work when objects print one after another", "[WallSublayers]")
{
    // print_sequence by object: each 20mm cube prints all its 100 layers of 0.2mm before the other
    // starts, and every layer but each object's first splits into 4 passes.
    const std::string gcode = slice_two_cubes_apart(80., {{"layer_height", "0.2"},
                                                          {"initial_layer_print_height", "0.2"},
                                                          {"z_hop", "0"},
                                                          {"skirt_loops", "0"},
                                                          {"print_sequence", "by object"},
                                                          {"wall_sublayer_height", "0.05"}});
    REQUIRE(! gcode.empty());
    size_t split = 0, whole = 0, other = 0;
    for (const auto &layer : extruding_zs_per_layer(gcode)) {
        const size_t n = distinct_sorted(layer).size();
        (n == 4 ? split : n == 1 ? whole : other) += 1;
    }
    CHECK(split == 2 * 99);
    CHECK(whole == 2);
    CHECK(other == 0);
}

// Full Spectrum mixed filaments: a virtual filament blending two physical ones, either by alternating
// whole layers or by splitting each layer into a sub-layer per component.

namespace {

// Every region role on the mixed slot blending filaments 1 and 2 60/40, with 0.2mm layers split by
// 0.05mm into 4 wall passes.
DynamicPrintConfig mixed_sublayered_config(bool mixed_color_sublayers)
{
    DynamicPrintConfig config = mixed_config(mixed_color_sublayers);
    config.set_deserialize_strict({{"layer_height", "0.2"},
                                   {"initial_layer_print_height", "0.2"},
                                   {"wall_loops", "3"},
                                   {"z_hop", "0"},
                                   {"skirt_loops", "0"},
                                   {"wall_sublayer_height", "0.05"}});
    return config;
}

// Z never drops within a layer, and every layer prints an outer wall.
void check_layers_rise_and_keep_their_outer_wall(const std::string &gcode)
{
    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 4);
    for (size_t i = 0; i < layers.size(); ++ i)
        for (size_t k = 1; k < layers[i].size(); ++ k) {
            INFO("layer " << i << " step " << k << ": " << layers[i][k - 1] << " -> " << layers[i][k]);
            REQUIRE(layers[i][k] >= layers[i][k - 1] - EPSILON);
        }
    std::vector<size_t> outer_wall_moves;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &, int) {
        if (! outer_wall_moves.empty())
            ++ outer_wall_moves.back();
    }, [&]() { outer_wall_moves.emplace_back(0); });
    for (size_t i = 0; i < outer_wall_moves.size(); ++ i) {
        INFO("layer " << i);
        CHECK(outer_wall_moves[i] > 0);
    }
}

} // namespace

TEST_CASE("Sub-layered walls work with a mixed filament alternating whole layers", "[WallSublayers]")
{
    // Each layer prints with one of the mixed slot's two components, outer wall passes included, so
    // across the print both components print passes - each layer above the first in 4 of them.
    const std::string gcode = slice({cube(20)}, mixed_sublayered_config(false));
    REQUIRE(! gcode.empty());
    check_layers_rise_and_keep_their_outer_wall(gcode);

    const auto layers = extruding_zs_per_layer(gcode);
    for (size_t i = 1; i < layers.size(); ++ i) {
        INFO("layer " << i);
        CHECK(distinct_sorted(layers[i]).size() == 4);
    }
    std::set<int> pass_tools;
    for_each_move_of_type(gcode, "Outer wall", [&](const GCodeReader &self, int tool) {
        if (! on_layer_top(self.z(), 0.2))
            pass_tools.insert(tool);
    });
    CHECK(pass_tools == std::set<int>{0, 1});
}

TEST_CASE("Sub-layered walls and mixed color sub-layers print a sound layer together", "[WallSublayers]")
{
    // Mixed color sub-layers print the mixed region once per component, each at a sub-height of its
    // own - a split of the layer in Z just like the wall passes. Whatever the two make of it, a layer
    // still never moves the nozzle down and still has its outer wall.
    const std::string gcode = slice({cube(20)}, mixed_sublayered_config(true));
    REQUIRE(! gcode.empty());
    check_layers_rise_and_keep_their_outer_wall(gcode);

    // The color sub-layers already split the layer, so the outer wall is not split again - and the user
    // is told why their sub-layer setting has no effect there.
    Print print;
    Model model;
    init_print({cube(20)}, print, model, mixed_sublayered_config(true));
    std::vector<StringObjectException> warnings;
    print.validate(&warnings);
    CHECK(std::any_of(warnings.begin(), warnings.end(), [](const StringObjectException &w) {
        return w.opt_key == "wall_sublayer_height" && w.string.find("mixed") != std::string::npos;
    }));
}

TEST_CASE("Sub-layered walls on a belt printer slice their passes around the layer's own plane", "[WallSublayers][belt]")
{
    // A belt printer slices the cube tilted 45 degrees and moves every layer's print_z afterwards, while
    // its geometry stays where it was sliced. The passes have to be sliced around the layer's slicing
    // plane: the second and third of 4 passes are 1/8 of the layer below and above it, so where the
    // tilted cross-section grows or shrinks the layer's own outline lies between theirs.
    Model model;
    Print print;
    init_print({cube(20)}, print, model, {{"belt_printer", 1},
                                          {"belt_slice_rotation", "x"},
                                          {"belt_slice_rotation_angle", 45},
                                          {"layer_height", 0.2},
                                          {"initial_layer_print_height", 0.2},
                                          {"skirt_loops", 0},
                                          {"elefant_foot_compensation", 0},
                                          {"wall_sublayer_height", "0.05"}});
    print.process();
    const PrintObject *object = print.objects().front();

    // The belt moved print_z off the slicing plane, or this would not test anything.
    bool shifted = false;
    for (const Layer *layer : object->layers())
        shifted = shifted || std::abs(layer->print_z - 0.5 * layer->height - layer->slice_z) > 0.01;
    REQUIRE(shifted);

    const auto area = [](const ExPolygons &expolygons) {
        double out = 0.;
        for (const ExPolygon &expolygon : expolygons)
            out += expolygon.area();
        return out;
    };
    size_t checked = 0;
    for (const Layer *layer : object->layers()) {
        const std::vector<WallSubSlice> &passes = layer->wall_sub_slices();
        if (passes.size() != 4)
            continue;
        const double below = area(passes[1].merged), above = area(passes[2].merged), own = area(layer->lslices);
        // Only where the cross-section changes measurably across the layer.
        if (std::abs(above - below) < 0.01 * own)
            continue;
        ++ checked;
        INFO("layer at slice_z " << layer->slice_z << ": passes " << below << " / " << above << ", layer " << own);
        CHECK(own > std::min(below, above) - 0.002 * own);
        CHECK(own < std::max(below, above) + 0.002 * own);
    }
    REQUIRE(checked > 10);
}

TEST_CASE("Sub-layered walls follow the beams interlocking two parts", "[WallSublayers]")
{
    // Two touching blocks in filaments 1 and 2, interlocked by beams along their interface; only the
    // first is sub-layered. The beams reshape both parts' slices after slicing, so the first part's
    // passes have to follow its interlocked outline rather than the straight interface of the model,
    // or they print through the second part's beams.
    TriangleMesh second = make_cube(20., 20., 4.);
    second.translate(20., 0., 0.);
    Print print;
    Model model;
    init_object(print, model,
                multifilament_config(2, {{"layer_height", "0.2"},
                                         {"initial_layer_print_height", "0.2"},
                                         {"wall_loops", "2"},
                                         {"z_hop", "0"},
                                         {"skirt_loops", "0"},
                                         {"enable_prime_tower", "0"},
                                         {"layer_change_gcode", "G92 E0"},
                                         {"interlocking_beam", "1"},
                                         {"wall_sublayer_height", "0.05"}}),
                make_cube(20., 20., 4.),
                {{std::move(second), ModelVolumeType::MODEL_PART,
                  {{"outer_wall_filament_id", "2"}, {"inner_wall_filament_id", "2"}, {"sparse_infill_filament_id", "2"},
                   {"internal_solid_filament_id", "2"}, {"wall_sublayer_height", "0"}}}});
    const StringObjectException error = print.validate();
    INFO("validate: " << error.string);
    REQUIRE(error.string.empty());
    print.process();

    const PrintObject *object = print.objects().front();
    int first = -1, second_id = -1;
    for (const LayerRegion *layerm : object->layers()[2]->regions())
        if (! layerm->slices.empty())
            (layerm->region().config().outer_wall_filament_id.value == 1 ? first : second_id) = layerm->region().print_object_region_id();
    REQUIRE(first >= 0);
    REQUIRE(second_id >= 0);

    size_t passes = 0;
    for (const Layer *layer : object->layers()) {
        const ExPolygons theirs = to_expolygons(layer->get_region(second_id)->slices.surfaces);
        for (const WallSubSlice &sub : layer->wall_sub_slices()) {
            ++ passes;
            double overlap = 0.;
            for (const ExPolygon &e : intersection_ex(sub.region_slices[first], theirs))
                overlap += unscaled(unscaled(e.area()));
            INFO("layer at " << layer->print_z << ", pass at " << sub.print_z);
            CHECK(overlap < 0.05);
        }
    }
    REQUIRE(passes > 0);
}

TEST_CASE("Sub-layered walls split every layer of a variable layer height by its own height", "[WallSublayers]")
{
    // Layer ranges of 0.1mm up to 2mm and 0.3mm above, split by 0.05mm: 2 passes in a thin layer, 6 in
    // a thick one, and either way the passes tile the layer from its bottom to its print_z.
    Print print;
    Model model;
    init_object(print, model, plain_config({{"wall_sublayer_height", "0.05"}}), make_cube(20., 20., 4.), {}, [](ModelObject &object) {
        for (const auto &[range, height] : {std::pair{t_layer_height_range{0., 2.}, "0.1"}, std::pair{t_layer_height_range{2., 4.}, "0.3"}}) {
            DynamicPrintConfig range_config;
            range_config.set_deserialize_strict({{"layer_height", height}});
            object.layer_config_ranges[range].assign_config(std::move(range_config));
        }
    });
    print.process();

    std::set<size_t> pass_counts;
    for (const Layer *layer : print.objects().front()->layers()) {
        if (layer->id() == 0)
            continue;
        INFO("layer " << layer->id() << " at z " << layer->print_z << ", " << layer->height << "mm");
        const std::vector<WallSubSlice> &subs = layer->wall_sub_slices();
        REQUIRE(subs.size() == size_t(std::lround(layer->height / 0.05)));
        double bottom = layer->bottom_z();
        for (const WallSubSlice &sub : subs) {
            CHECK_THAT(sub.print_z - sub.height, Catch::Matchers::WithinAbs(bottom, 1e-9));
            CHECK_THAT(sub.height, Catch::Matchers::WithinAbs(0.05, 1e-9));
            bottom = sub.print_z;
        }
        CHECK_THAT(bottom, Catch::Matchers::WithinAbs(layer->print_z, 1e-9));
        pass_counts.insert(subs.size());
    }
    CHECK(pass_counts == std::set<size_t>{2, 6});
}

TEST_CASE("Sub-layered walls leave the first object layer on a raft whole", "[WallSublayers]")
{
    // 2 raft layers, then the object. Like the first layer on the bed, the first object layer is laid
    // onto the raft at one height; the 0.2mm layers above it are split by 0.05mm into 4 passes.
    Print print;
    Model model;
    init_object(print, model, plain_config({{"wall_sublayer_height", "0.05"}, {"raft_layers", "2"}}), make_cube(20., 20., 2.));
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 4);
    for (size_t i = 0; i < 3; ++ i) {
        INFO("printed layer " << i << " (raft, raft, first object layer)");
        CHECK(distinct_sorted(layers[i]).size() == 1);
    }
    CHECK(distinct_sorted(layers[3]).size() == 4);
}

TEST_CASE("Sub-layered walls are lifted by the Z offset with the rest of the layer", "[WallSublayers]")
{
    // z_offset 0.3: every pass of the second 0.2mm layer lands 0.3mm above where it would without it.
    const std::string gcode = slice({cube(20)}, plain_config({{"wall_sublayer_height", "0.05"}, {"z_offset", "0.3"}}));
    REQUIRE(! gcode.empty());

    const auto layers = extruding_zs_per_layer(gcode);
    REQUIRE(layers.size() > 2);
    const std::vector<double> zs = distinct_sorted(layers[1]);
    REQUIRE(zs.size() == 4);
    for (int k = 0; k < 4; ++ k)
        CHECK_THAT(zs[k], Catch::Matchers::WithinAbs(0.3 + 0.2 + 0.05 * (k + 1), 1e-6));
}
