# libslic3r — slicing pipeline notes

Working notes for the core slicing library. The repo-wide rules are in the root
[AGENTS.md](../../AGENTS.md); this file covers what is expensive to rediscover about the pipeline
itself.

## The pipeline and its steps

Work is organised as invalidatable steps, per object (`PrintObjectStep`) and per print (`PrintStep`).
Each step is skipped when its state is still valid, so **which step a setting invalidates decides how
much work a settings change costs.**

```
posSlice → posPerimeters → posPrepareInfill → posInfill → posIroning → posSupportMaterial …
                                                        → psSkirtBrim → psWipeTower → psGCodeExport
```

- `posSlice` — `PrintObject::slice()` / `slice_volumes()` (`PrintObjectSlice.cpp`). Cuts the mesh at
  each layer's `slice_z`, assigns the result to regions, applies XY and elephant-foot compensation,
  MMU and fuzzy-skin segmentation, conical overhangs, interlocking.
- `posPerimeters` — `PrintObject::make_perimeters()` → `Layer::make_perimeters()` →
  `LayerRegion::make_perimeters()` → `PerimeterGenerator`.
- `psGCodeExport` — `GCode::process_layers()` → `GCode::process_layer()`.

### Registering a new setting

Missing any of these produces a specific, hard-to-diagnose failure:

| Site | Consequence of forgetting it |
|---|---|
| `PrintConfig.cpp` `init_fff_params` | Key does not exist. |
| `PrintConfig.hpp` class sequence | Not readable from the typed config. |
| `Preset.cpp` `s_Preset_print_options` | Never saved to presets or 3MF, and missing from the per-object / per-modifier override tabs. |
| `PrintObject::invalidate_state_by_config_options` (or `Print::` for print-level keys) | Falls through to `invalidate_all_steps()` — a full re-slice on every edit. |
| `Layer::is_perimeter_compatible` (perimeter-affecting region keys only) | Two regions with different values are merged into one `make_perimeters()` call and silently share the first one's value. |
| `Tab.cpp`, `GUI_Factories.cpp` | Invisible in the settings tab / the per-part "Add settings" menu. |

Pick the config class by override scope: `PrintObjectConfig` = per object; `PrintRegionConfig` = per
object **and** per modifier mesh **and** per height range; `PrintConfig` / `GCodeConfig` = whole print.

Unknown keys are silently ignored by every loader, so a brand-new key needs no `handle_legacy` and no
profile changes — an old project just falls back to the option's default.

**`ratio_over` does not resolve across config classes.** `ConfigBase::get_abs_value` looks the base key
up in the same config object, so a `PrintRegionConfig` option with `ratio_over = "layer_height"` (which
lives in `PrintObjectConfig`) must be resolved by hand against the real value. Doing it by hand is also
what makes adaptive and per-range layer heights behave — see `wall_sublayer_count()` (`WallSublayers.cpp`) and
the scarf height resolution in `GCode::extrude_loop`.

## Perimeter generation

`PerimeterGenerator` has two independent implementations, `process_classic()` and `process_arachne()`
(Arachne is the default). **Anything that changes wall generation has to be done in both.**

- `ExtrusionEntity::inset_idx` is the reliable wall identifier: 0 is the outermost loop, 1 the first
  inner wall, and so on. Roles cannot tell a first inner wall from a second. Thin walls carry `-1`.
  Note it is a different field from `Arachne::ExtrusionLine::inset_idx`, which is what the Arachne
  internals and the fuzzy-skin code read; `traverse_extrusions` copies the latter onto the former.
- `LayerRegion::perimeters` is a collection of **per-island collections**; order is preserved all the
  way to the emitter. Consumers must flatten one level — `GCode::ObjectByExtruder::Island::Region::append`
  is the reference.
- `wall_sequence` (including inner-outer-inner, which reasons about `inset_idx` directly) is applied at
  the end of each generator. Post-processing that reorders or filters walls belongs **after** that
  point, so the ordering stays in charge of what remains.
- Overhang classification compares against `PerimeterGenerator::lower_slices`. Overhang and bridge
  paths live inside the same `ExtrusionLoop` as the external perimeter, so detection has to be per
  `ExtrusionPath`, not per loop.
- Gap fill goes to `LayerRegion::thin_fills`, which is copied into `fills` — it is emitted **with the
  infill**, not with the walls.
- Bridge flow is a thread diameter, not a layer height: `mm3_per_mm` ignores height and
  `Flow::with_height()` asserts on it. Never rescale a bridge flow by a height ratio.

## G-code export

`GCode::process_layer()` (`GCode.cpp`) assembles one layer: extrusions are bucketed by filament, then
by instance, then by island and region, and `extrude_perimeters` / `extrude_infill` decide the order
from the per-region `is_infill_first`. Ironing is always emitted last.

Three separate emptiness checks read `LayerRegion::perimeters` and `fills` to decide whether anything
exists to print: `LayerRegion::has_extrusions()` (skips the layer), `ToolOrdering::collect_extruders`
(claims the filament) and the island collection in `process_layer` (creates the instance). Extrusions
held anywhere else are invisible to all three, and a feature that is *only* walls — a thin plate, a
Benchy's smoke stack — has nothing in either collection to fall back on, so it is dropped silently
rather than misprinted. Anything storing extrusions outside those two has to be added to all three.

### Z within a layer

Three features already print part of a layer at a Z other than `print_z`; all of them work the same
way, and a fourth should too:

- **Scarf seams** (`seam_slope_*`) ramp Z within one extrusion via `ExtrusionPathSloped`.
- **Mixed-color sub-layers** re-run the region loop at a sub-Z with `m_sub_layer_flow_ratio` /
  `m_sub_layer_height` scaling the flow and reporting the sub-height.
- **Sub-layered walls** (`wall_sublayer_height`) print the outermost walls as a stack of passes;
  their paths already carry the right height and flow, so only Z is driven.

The mechanism is: set `m_nominal_z`, set `m_need_change_layer_lift_z`, extrude, then restore
`m_nominal_z` to `print_z + z_offset`. `travel_to()` travels at the current height and applies the
destination Z only at the final point, which is what keeps the move safe. Do **not** route an in-layer
Z change through `change_layer()` — that emits a layer-change tag and can trigger a retract.

Rules that hold for anything printing at several Z levels in one layer:

- **Z must not decrease within a layer.** The nozzle would otherwise cross material printed above its
  current level. Anything printed at a lower Z has to be emitted before everything above it, which in
  practice means before the skirt, brim and supports of that layer.
- With several objects or instances, group by Z level across all of them: complete level `k`
  everywhere before starting level `k+1`, or Z will oscillate through the layer.
- Emit exactly **one** layer-change tag per real layer. `GCodeProcessor` counts layers from that tag,
  so extra Z moves do not create phantom preview layers. The `;HEIGHT:` tag is per path and is the
  right place to report a sub-height.
- `m_layer_id` is the number of layers the *printer* prints: it is written into the G-code header as
  the total layer count and buckets the time estimate. To give the preview a layer of its own without
  disturbing either, count separately and add that to `MoveVertex::layer_id`, which nothing outside
  `GCodeViewer` reads (`m_sub_layer_count` and `GCodeProcessor::Sub_Layer_Tag` are the example). The
  viewer requires those ids to arrive in order and without gaps, so mark a sub-layer only once it is
  known to print something.
- Set a hidden derived bool (`has_scarf_joint_seam`, `has_sublayered_walls`) in `PrintApply.cpp` and
  OR it into `GCodeProcessor::m_detect_layer_based_on_tag`, otherwise the seam detector mis-reads the
  in-layer Z changes.
- Features that assume a wall spans the whole layer, or that the rest of the layer is already printed,
  must be suppressed: scarf seams (their ramp is a full layer height) and wipe-before-external-loop
  (the inner wall it aims for may not exist yet).

## Testing the pipeline

Tests live in `tests/fff_print` — see [tests/AGENTS.md](../../tests/AGENTS.md). Two traps specific to
G-code assertions:

- The output is **not byte-reproducible** across two slices in one process: the export timestamp and a
  process-wide object-id counter differ. Compare parsed commands via `GCodeReader`, not raw text.
- `tests/fff_print/test_helpers.hpp` already has `layers_with_role`, `role_passes`, `role_sequence`
  and `max_z`. Check them before writing another G-code parser.
