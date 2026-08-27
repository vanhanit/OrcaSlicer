# Sub-layered Outer Walls — Design Spec

Date: 2026-08-27
Branch: `feat/wall-sublayers`

## Overview

OrcaSlicer prints every feature of a layer at one Z and one height. Surface quality on sloped
geometry is therefore capped by the layer height, and the only way to improve it is to lower the
layer height for the whole print, which costs time on the inner walls and infill where the finer
resolution buys nothing.

This feature prints the outermost walls as a stack of thinner passes inside a single layer. The mesh
is **re-sliced at each sub-layer height**, so each pass follows the model's real contour at its own
height rather than repeating the layer's contour. Inner walls and infill still print once per layer
at the full layer height.

## Goals

1. Vertical resolution on outer surfaces decoupled from the layer height, at close to the print time
   of the coarser layer height.
2. **The nozzle never descends within a layer.** Z is monotonically non-decreasing from the start of
   a layer to its end, so the nozzle cannot crash into material already printed above its level.
3. Zero regression when disabled: with the default settings the emitted G-code commands are
   identical to those produced before the feature existed.
4. Works with both wall generators (classic and Arachne), with adaptive and per-range layer heights,
   and with per-object / per-modifier overrides.

## Non-Goals

- Sub-layer resolution for inner walls, infill, or top/bottom surfaces.
- Compatibility with spiral vase mode, which rewrites a layer around one continuous Z ramp.
- Multi-material painted objects (see Conflicts).

## Settings

Both live in `PrintRegionConfig` (`src/libslic3r/PrintConfig.hpp`), so they can be overridden per
object, per modifier mesh, and per height range.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `wall_sublayer_height` | `coFloatOrPercent`, `ratio_over = "layer_height"` | `0` (off) | Target height of one wall pass. |
| `wall_sublayer_loops` | `coInt`, 1..5 | 1 | How many of the outermost loops become sub-layer stacks. Clamped to `wall_loops`. |
| `has_sublayered_walls` | `coBool`, hidden | false | Derived during apply; tells `GCodeProcessor` to detect layers from the layer-change tag. |

`wall_sublayer_height` is resolved **manually against the real per-layer height**, never against the
`layer_height` setting: `ratio_over` cannot resolve automatically here because `layer_height` lives
in `PrintObjectConfig` while these keys live in `PrintRegionConfig` (see `ConfigBase::get_abs_value`,
`src/libslic3r/Config.cpp`). This is also what makes adaptive and per-range layer heights work.

`wall_sublayer_count()` (`src/libslic3r/Layer.cpp`) turns the setting into a per-layer count:
`n = clamp(round(H / target), 1, MAX_WALL_SUBLAYERS)`, then reduced while `H/n` would fall below
`MIN_WALL_SUBLAYER_HEIGHT`. The layer is always split into `n` **equal** passes, so the topmost pass
lands exactly on the layer's `print_z`.

## Architecture

Three stages, all inside `posPerimeters` and the G-code export. Nothing in `posSlice` changes, so
toggling the feature does not invalidate slicing and no other consumer of the layer slices is
affected.

### 1. Sub-slicing — `PrintObject::slice_wall_sublayers()` (`PrintObjectSlice.cpp`)

Called from `PrintObject::make_perimeters()` before the per-layer parallel loop. For each layer past
the first it computes `n` (the maximum over the layer's enabled regions), collects every sub-slice
plane into one ascending list, and slices the mesh **once** by reusing the existing
`slice_volumes_inner()` + `slices_to_regions()` pair — the same call shape as `slice_volumes()`, so
modifier meshes, negative volumes and multi-part clipping behave exactly as they do for layer slices.
The same negative XY compensation is then applied via `_shrink_contour_holes()`.

Results land in `Layer::wall_sub_slices` (`std::vector<WallSubSlice>`, `Layer.hpp`), each entry
carrying its `print_z`, `slice_z`, `height`, per-region ExPolygons and their union (`merged`).

The first layer is never subdivided — its height is dictated by bed adhesion. Raft layers are support
layers and never appear in `m_layers`, so indexing by layer position is inherently raft-aware.

### 2. Wall generation — `LayerRegion::make_perimeters()` (`LayerRegion.cpp`)

Two new inert-by-default members on `PerimeterGenerator`:

- `sublayer_band_walls` (band mode): generate only this many outermost loops, from a sub-slice.
- `sublayer_drop_walls` (core mode): drop loops with `inset_idx < this` from the output.

The layer's own generator run is unchanged except for `sublayer_drop_walls`. Then one extra
generator run per sub-slice produces that pass's outermost loops into
`LayerRegion::sublayer_perimeters[k]`, using flows built at the sub-layer height
(`LayerRegion::flow(role, height)`), with the bridge flow left alone because it is a thread diameter
rather than a layer height.

**Why "generate normally, then drop" rather than shrinking the core region:**

- It is the only formulation that works for Arachne, whose walls have variable width, so a constant
  "band width" offset is not well defined.
- Inner wall positions, infill boundaries, `only_one_wall_top` splitting and the `wall_sequence`
  ordering (including inner-outer-inner, which reasons about `inset_idx` directly) are all computed
  on the full wall set exactly as before, and only then are the band loops removed. That is what
  makes the rest of the layer byte-identical to a non-subdivided print.
- Dropping after the ordering has been applied leaves `wall_sequence` in charge of the loops that
  remain.
- The mid-layer slice the core is built from is at worst `(H/2)·tanθ` away from any sub-slice on a
  slope, half the error of retreating the core to the innermost sub-slice.

Overhangs and bridges are classified against `wall_sub_slices[k-1].merged`, or for the lowest pass
the layer below's topmost sub-slice — the material directly beneath that pass. A wall supported by
the pass under it is therefore no longer treated as an overhang of the whole layer. Classic gap fill
found behind a dropped wall is suppressed in the core run and rediscovered by each band pass from its
own sub-slice, so it is filled at the sub-layer height.

`only_one_wall_top` / `only_one_wall_first_layer`, `process_no_bridge()` and
`apply_extra_perimeters()` are all core-only: they change the wall count or the infill area of the
layer as a whole, which the band passes must not touch.

### 3. Emission — `GCode::extrude_sublayer_walls()` (`GCode.cpp`)

Runs inside the per-extruder section of `process_layer`, right after the tool change and **before**
the instance loop that emits skirt, brim and supports. That placement is what makes a single-filament
print monotonic in Z across the whole layer.

The outer loop is the pass index and the inner loop is the instances: every instance completes pass
`k` before any instance starts pass `k+1`. With two objects on the bed, ungrouped passes would make Z
rise and fall repeatedly within one layer.

Z is driven exactly like the existing mixed-color sub-layer feature: set `m_nominal_z` and
`m_need_change_layer_lift_z`, and let the next travel reach the new height lazily — `travel_to()`
already travels at the current height and applies the destination Z only at the end point. On exit
`m_nominal_z` is restored to the layer's `print_z`.

`m_sub_layer_flow_ratio` is deliberately **not** used: unlike the mixed-color feature, these paths
were generated at the sub-layer height, so their width, height and `mm3_per_mm` are already correct
and the `;HEIGHT:` tag reports the sub-height on its own.

While `m_in_sublayer_wall_pass` is set, two features are suppressed:
- **Scarf seams**, whose Z ramp spans a full layer height and would cut into the pass below.
- **wipe-before-external-loop**, which aims for a neighbouring inner wall that does not exist at that
  height yet.

`GCodeProcessor` counts layers from the layer-change tag, which is still emitted once per layer, so
the extra Z moves create no phantom preview layers. `has_sublayered_walls` is OR-ed into
`m_detect_layer_based_on_tag` so the seam detector tolerates in-layer Z changes, mirroring
`has_scarf_joint_seam`.

## Conflicts

| Setting | Resolution |
|---|---|
| `spiral_mode` | Rejected. Dialog in `ConfigManipulation`, error in `Print::validate()`. |
| Multi-material painting | Rejected in `validate()`: sub-slices bypass MMU segmentation. |
| Sub-layer height too small / too large | Clamp-and-notify dialog; `wall_sublayer_count()` also clamps. |
| `make_overhang_printable` | Warning: sub-slices come from the original model. |
| `zaa_enabled` | Warning: both refine the same surfaces. |
| Scarf seam | Applied to the layer's own walls, never to the sub-layer passes. |
| `wall_sequence`, incl. inner-outer-inner | Applies within each pass and to the remaining core walls. |
| `is_infill_first` | Unaffected: the passes always precede the layer's normal phase. |
| `only_one_wall_top` / `_first_layer` | Honored by the core run, ignored by the passes. |
| Ironing | Still last in the layer. |

## Known limitations

- On a slope, the interface between the passes and the inner walls can be off by up to `(H/2)·tanθ`,
  since the inner walls still derive from the layer's mid-height slice.
- With several filaments, a later extruder's passes can start below content an earlier extruder
  printed at `print_z` (wipe tower, supports on another filament). The lift-travel mitigates it, and
  `validate()` warns. This is the same exposure the existing mixed-color sub-layer feature accepts.
- Print time and G-code size for the outer walls grow roughly in proportion to the number of passes.
  Capped by `MAX_WALL_SUBLAYERS`.

## Verification

`tests/fff_print/test_wall_sublayers.cpp`, tag `[WallSublayers]`. The two load-bearing tests are
"never moves the nozzle back down within a layer" (the safety contract, checked with supports on) and
"follow the model between sub-layers on a slope" (proves real re-slicing rather than a repeated
contour, by measuring that each pass on a pyramid starts further inward than the one below).
