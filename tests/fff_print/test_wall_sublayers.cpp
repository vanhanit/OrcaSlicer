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
            for (size_t k = 0; k < layer->wall_sub_slices.size(); ++ k) {
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
