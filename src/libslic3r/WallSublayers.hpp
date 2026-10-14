#pragma once

#include "ExPolygon.hpp"
#include "libslic3r.h"

#include <vector>

namespace Slic3r {

class LayerRegion;
class PrintObject;
class PrintRegion;
struct WallSubSlice;
class PrintConfig;
class PrintRegionConfig;
using LayerRegionPtrs = std::vector<LayerRegion*>;

// Orca: sub-layered outer walls. A layer is split into at most this many wall passes, and a pass is
// never thinner than this, so a small wall_sublayer_height cannot explode the G-code or ask the
// extruder for an unprintable amount of material.
static constexpr int      MAX_WALL_SUBLAYERS       = 10;
static constexpr coordf_t MIN_WALL_SUBLAYER_HEIGHT = 0.02;

// How many sub-layers a layer of the given height is split into for this region of the object, 1
// meaning the walls print at the full layer height as usual. wall_sublayer_height is resolved against
// the real layer height rather than the layer_height setting, so adaptive and per-range heights are
// honored. Two kinds of walls are never sub-layered, see the predicates below.
int wall_sublayer_count(const PrintObject &object, const PrintRegion &region, coordf_t layer_height);
// Walls in a mixed filament that enable_mixed_color_sublayer splits into color sub-layers: that split
// already divides their layer in Z, once per component filament.
bool wall_in_mixed_color_sublayers(const PrintRegionConfig &config, const PrintConfig &print_config);
// Walls colour painted in a filament other than their volume's. A layer's passes go first, before
// anything is printed at its print_z, and only one filament can go first without the nozzle coming down
// beside something already standing a full layer high - see ToolOrdering::put_sublayer_wall_extruders_first().
// The object's own filament prints its walls in passes, the painted colours after them at full height.
bool wall_painted_in_other_filament(const PrintObject &object, const PrintRegion &region);

// The areas a layer's sub-layered walls are generated from, and the column left to the layer's own
// full-height pass. band_walls == 0 means the feature is off for this region and the rest is empty.
struct WallSublayerContext
{
    // How many outermost loops the passes take over from the layer's own perimeter run, at the width
    // that run uses. This is what the layer's own run drops.
    int                     band_walls = 0;
    // How many loops the passes actually print to cover that strip. Equal to band_walls unless
    // wall_sublayer_line_width makes the passes narrower than the loops they replace.
    int                     band_loops = 0;
    // The split of the layer this region's passes are printed in, see Layer::wall_sublayer_split().
    const std::vector<WallSubSlice> *passes = nullptr;
    // Per pass, lowest first: this layer re-sliced at that sub-layer's height, over all the regions
    // sharing this perimeter run.
    std::vector<ExPolygons> pass_slices;
    // Where the model is solid for the whole layer height, plus anything no pass can build on, which
    // the layer's own full-height pass bridges at print_z. Unset when no pass has any area.
    ExPolygons              core_region;
    // Only the solid-for-the-whole-height part. That column carries a pass standing on it; the rest
    // of core_region is printed at print_z, above the passes, so it is air while they run.
    ExPolygons              core_base;
    bool                    core_region_set = false;
    // Where the passes cannot print at all: the part of the layer that hangs inside a void the layer
    // below closed over. The layer's own run keeps its outermost walls there - the band that would
    // have printed them is refused as airborne - and bridges the rest at print_z.
    ExPolygons              ceiling;
};

// Compute the above for one region. Cheap and inert when the feature is off.
WallSublayerContext wall_sublayer_prepare(const LayerRegion &layerm, const LayerRegionPtrs &compatible_regions, bool spiral_mode);

// Run the wall generator once per sub-layer into layerm.sublayer_perimeters, then fill the ground the
// passes own that their walls do not cover. Call only when ctx.band_walls > 0.
void wall_sublayer_generate(LayerRegion               &layerm,
                            const LayerRegionPtrs     &compatible_regions,
                            const WallSublayerContext &ctx,
                            bool                       arachne,
                            bool                       spiral_mode,
                            double                     model_rotation_rad);

} // namespace Slic3r
