#pragma once

#include "ExPolygon.hpp"
#include "libslic3r.h"

#include <vector>

namespace Slic3r {

class LayerRegion;
class PrintRegionConfig;
using LayerRegionPtrs = std::vector<LayerRegion*>;

// Orca: sub-layered outer walls. A layer is split into at most this many wall passes, and a pass is
// never thinner than this, so a small wall_sublayer_height cannot explode the G-code or ask the
// extruder for an unprintable amount of material.
static constexpr int      MAX_WALL_SUBLAYERS       = 10;
static constexpr coordf_t MIN_WALL_SUBLAYER_HEIGHT = 0.02;

// How many sub-layers a layer of the given height is split into for this region, 1 meaning the walls
// print at the full layer height as usual. wall_sublayer_height is resolved against the real layer
// height rather than the layer_height setting, so adaptive and per-range heights are honored.
int wall_sublayer_count(const PrintRegionConfig &config, coordf_t layer_height);

// The areas a layer's sub-layered walls are generated from, and the column left to the layer's own
// full-height pass. band_walls == 0 means the feature is off for this region and the rest is empty.
struct WallSublayerContext
{
    // How many outermost loops the passes take over from the layer's own perimeter run.
    int                     band_walls = 0;
    // Per pass, lowest first: this layer re-sliced at that sub-layer's height, over all the regions
    // sharing this perimeter run.
    std::vector<ExPolygons> pass_slices;
    // Where the model is solid for the whole layer height, which is the only ground the layer's own
    // full-height pass may occupy. Unset when no pass has any area.
    ExPolygons              core_region;
    bool                    core_region_set = false;
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
