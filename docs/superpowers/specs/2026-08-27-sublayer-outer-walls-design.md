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

The layer's own generator run differs only in `sublayer_drop_walls` and in being fed the layer's
slice **clipped to the intersection of the sub-slices**. Everything the core run emits — its walls
and its infill alike — is printed at `print_z` over the full layer height, so it occupies its whole
column from `bottom_z` upwards, and is only valid where the model is solid for that entire height.
Built from the layer's mid-height slice instead it goes wrong two ways: its walls sit outside the
surface the layer actually presents once the contour recedes by more than a wall width across the
layer (`(H/2)·tanθ > w`, a slope within about 13° of horizontal), and where the contour grows it
comes down on a column a pass has already laid a wall on and reaches into ground that is a hole at
some sub-layer. The clip is applied per surface, which preserves the surface type and the
`extra_perimeters` grouping `Layer::make_perimeters` built. A pass with nothing in it does not
constrain the core, or a feature ending part way up a layer would take the whole layer's content
with it.

The intersection, rather than just the topmost sub-slice, is what makes this hold in both
directions: the topmost alone leaves the growing case — a hole closing over — untouched, since the
top sub-slice then contains the layer's own slice and the clip does nothing.

Then one extra generator run per sub-slice produces that pass's outermost loops into
`LayerRegion::sublayer_perimeters[k]`, using flows built at the sub-layer height
(`LayerRegion::flow(role, height)`) — the overhang flow included. A bridge flow is a thread as thick
as the nozzle, which is right for a wall spanning a whole layer and roughly five times too much
material inside a pass a fraction of that height: it stands proud of the pass and the one above
ploughs through it. The overhang *role* is kept, so those walls still print at the bridge speed with
the part cooling fan on.

Each pass keeps two things beyond its walls. Its **gap fill**, appended after the walls. And the
**support fill**, which is everything the passes own that their walls do not cover. Two parts:

- `((slice(k) ∖ core) ∖ band(k)) ∩ below(k)` — the **tread**: solid at this pass, outside the column
  the core run may occupy, and over material that actually exists beneath the pass. The core is confined to the intersection of the sub-slices, so nothing else in the
  layer ever reaches this ground, and it is what the wall band of every pass above comes down on.
  Repeating it pass after pass is how the staircase a sloped surface makes is filled up to `print_z`;
  a region that becomes solid at pass `k` is refilled by every pass above it without any explicit
  carrying.
That is the whole of it. A wall band that lands past its support is **not** propped up: fill placed
under it at the pass below would be sitting on air itself, and on an outward-rolling surface — a
Benchy's gunwale — it came out as material on the outside of the hull. Such a wall prints as the
overhang or bridge the wall generator has already classified it as, exactly as it would without
sub-layers.

The support fill therefore has a contract worth stating plainly: **it never leaves the model and it
never floats.** Both halves come from the clip to `below(k)`, since that is model geometry beneath
the pass.

`interior(k)` is the pass generator's `fill_no_overlap` output and `band(k)` is the rest of the
sub-slice, so the two are complementary by construction.

**Why "generate normally, then drop" rather than shrinking the core region:**

- It is the only formulation that works for Arachne, whose walls have variable width, so a constant
  "band width" offset is not well defined.
- Inner wall positions, infill boundaries, `only_one_wall_top` splitting and the `wall_sequence`
  ordering (including inner-outer-inner, which reasons about `inset_idx` directly) are all computed
  on the full wall set exactly as before, and only then are the band loops removed. With the feature
  off nothing is clipped and no band is dropped, so the layer is byte-identical to a non-subdivided
  print; with it on, the whole run moves to the clipped contour together and the walls stay nested.
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

Finally, the passes own their columns for the whole height of the layer, so the wall bands are
subtracted from the layer's `fill_surfaces` and `fill_no_overlap` before `make_perimeters` returns.
Without it the layer's own infill — printed afterwards at `print_z`, at the full layer height —
extrudes into walls that are already there. The bands are shrunk by the same infill/wall overlap the
generator grows the fill area by, so the infill still bonds to the topmost pass exactly as it would
to a full-height wall. The subtraction survives `posPrepareInfill`: `Layer::make_perimeters` copies
the result into `fill_expolygons`, and `slices_to_fill_surfaces_clipped()` rebuilds `fill_surfaces`
as `slices ∩ fill_expolygons`.

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

A pass is printed onto the pass right below it, not onto the layer below, so
`ExtrusionQualityEstimator::override_prev_layer_boundary()` points the overhang estimator at
`Layer::wall_sublayer_support(k)` — the same material the wall generator classified that pass
against — for as long as the pass is being emitted, and `restore_prev_layer_boundaries()` puts the
layer below back before the rest of the layer. Measured against the layer below, a pass sits between
one and two layer heights above its reference instead of one, so the higher passes read as hanging
over further than they do and pick up an overhang slowdown and the part cooling fan that a
full-height wall over the same geometry never triggers. The curled extrusions are deliberately left
alone: they protrude upwards from the layer below and are in the way of any pass.

While `m_in_sublayer_wall_pass` is set, two features are suppressed:
- **Scarf seams**, whose Z ramp spans a full layer height and would cut into the pass below.
- **wipe-before-external-loop**, which aims for a neighbouring inner wall that does not exist at that
  height yet.

A region whose every loop is sub-layered keeps nothing in `LayerRegion::perimeters`, and a feature
that is only walls — a thin plate, a Benchy's smoke stack — has no fills either. Three emptiness
checks decide whether such a feature is printed at all, and each reads only those two collections:
`LayerRegion::has_extrusions()`, `ToolOrdering::collect_extruders` and the island collection in
`process_layer`. All three consult `has_sublayer_walls()` as well; missing any one of them drops the
feature from the G-code without a warning.

### Preview

`GCodeProcessor` counts layers from the layer-change tag, which is still emitted once per layer, so
the printer's layer count and the time estimate are unchanged. `has_sublayered_walls` is OR-ed into
`m_detect_layer_based_on_tag` so the seam detector tolerates in-layer Z changes, mirroring
`has_scarf_joint_seam`.

The preview does need a layer per pass, or the passes are only reachable by dragging the horizontal
slider through the middle of a layer. `GCodeProcessor::Sub_Layer_Tag` marks each pass below `print_z`
and is counted into `m_sub_layer_count`, which is added to `MoveVertex::layer_id` — a field nothing
outside `GCodeViewer` reads. The last pass ends at `print_z` and so shares its preview layer with the
walls and infill printed there, which keeps one Z per preview layer. The marker is written only once
a pass is known to print something, because the viewer cannot represent an empty layer.

Sub-layered walls keep the ordinary `erExternalPerimeter` / `erPerimeter` roles, so `;TYPE:` tags,
speeds and cooling are what they would be for a full-height wall, and the preview's feature-type list
counts them under Outer wall and Inner wall rather than on a row of their own.

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

- The support fill a pass prints for the pass above it goes down with the **wall** filament, the one
  that pass is printed with, not with `sparse_infill_filament`.
- A wall band past the material below it is left as an overhang: the passes never print outside the
  model to catch one. On a hole's ceiling that means the topmost passes bridge, as the layer would.
- Confining the core run to the intersection of the sub-slices means the layer's own infill stops at
  the narrowest cross-section the layer has, rather than at its mid-height one. Everything outside it
  is carried by the passes' wall bands and their tread fill, so the material is still there — but on
  a sloped surface a larger share of the layer is printed at sub-layer height, and the print takes
  correspondingly longer.
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

Three more cover the interaction between the passes and the fill, each on a shape chosen so the
contour sweeps further within one layer than a wall is wide — a cone 2mm tall over a 40mm base, or a
sphere:

- "supported by the pass beneath them" and its no-op twin "add no support fill where the walls stack
  up": solid fill appears at sub-layer Z on the cone and nowhere on a cube.
- "keep the layer's own infill out of their columns": the layer's fill area is strictly smaller with
  the feature on.
- "support a wall line that stands over nothing": the same cone stood on its head, so every pass's
  wall lands outside everything below it. Nothing is filled without the second case above.
- "keep the layer's own walls inside the topmost pass": no `Inner wall` move at a given Z may reach
  further out than the `Outer wall` at that same Z.
- "support fill is not sprayed over overhangs": the same cone stood on its head, the one shape where
  every pass lands outside the one below it.
- "the layer's full-height phase stays out of what the passes own": a block bored through by a round
  horizontal hole, sliced at 0.3mm with two walls so the wall inset cannot hide the error. At the
  layer where the bore closes over, the unconfined core run put 1.86mm of wall on a pass's band and
  24mm² of infill over ground that is a hole at some sub-layer.
- "are not slowed down as overhangs": on a sphere, over 90% of layers print every one of their passes
  at one feedrate. Measured against the layer below instead, only about two thirds do. Asserting the
  passes of a layer agree with *each other* is what makes this independent of how the estimator
  scores any particular slope.
