# Sub-layered Outer Walls — Implementation Plan

Date: 2026-08-27
Branch: `feat/wall-sublayers`
Design: [`../specs/2026-08-27-sublayer-outer-walls-design.md`](../specs/2026-08-27-sublayer-outer-walls-design.md)

Six commits, each self-contained and buildable. Stages 1–3 add machinery that nothing reads yet, so
the output is unchanged until stage 4 wires it into the emitter.

## Stage 1 — Config plumbing

`feat(config): wall_sublayer_height and wall_sublayer_loops region options`

| File | Change |
|---|---|
| `PrintConfig.cpp` | Both option definitions next to `wall_direction`; hidden `has_sublayered_walls` next to `has_scarf_joint_seam`. |
| `PrintConfig.hpp` | Both keys in the `PrintRegionConfig` sequence; `has_sublayered_walls` in `GCodeConfig`; `MAX_WALL_SUBLAYERS` / `MIN_WALL_SUBLAYER_HEIGHT`. |
| `Preset.cpp` | Both keys in `s_Preset_print_options`. |
| `Tab.cpp` | Two `append_single_option_line` in the "Walls and surfaces" group. |
| `GUI_Factories.cpp` | Both keys in `PART_CATEGORY_SETTINGS["Quality"]`. |
| `PrintObject.cpp` | Both keys in the `posPerimeters` bucket of `invalidate_state_by_config_options`. |
| `Layer.cpp` | Both keys in `is_perimeter_compatible`. |
| `PrintApply.cpp` | Derive `has_sublayered_walls`, mirroring the scarf block. |
| `GCodeProcessor.cpp` | OR it into `m_detect_layer_based_on_tag` (both `apply_config` overloads). |
| `Print.cpp` | `has_sublayered_walls` into `steps_gcode`, so the derived flag does not force a full re-slice. |

The three easiest to forget: `Preset.cpp` (without it the option is never saved and vanishes from the
override tabs), the invalidation bucket (an unlisted key falls through to `invalidate_all_steps()`),
and `is_perimeter_compatible` (without it two regions with different values silently share the first
one's).

## Stage 2 — Sub-slicing

`feat(slice): re-slice layers at sub-layer heights for sub-layered walls`

- `Layer.hpp`: `WallSubSlice`, `Layer::wall_sub_slices`, `LayerRegion::sublayer_perimeters`, and a
  forward declaration of `PrintRegionConfig` for the `wall_sublayer_count()` declaration.
- `Layer.cpp`: `wall_sublayer_count()`; clear `sublayer_perimeters` everywhere `perimeters` is cleared.
- `PrintObjectSlice.cpp`: `PrintObject::slice_wall_sublayers()`.
- `PrintObject.cpp` / `Print.hpp`: call it from `make_perimeters()`, declare it.

Reuse `slice_volumes_inner()` and `slices_to_regions()` rather than slicing by hand — they carry the
modifier-mesh, negative-volume and multi-part clipping semantics.

## Stage 3 — Wall generation

`feat(perimeter): generate walls per sub-layer, drop them from the core pass`

- `PerimeterGenerator.hpp`: `sublayer_band_walls`, `sublayer_drop_walls`, both defaulting to 0.
- `PerimeterGenerator.cpp`: `drop_sublayer_band_walls()` helper applied at both output sites (classic
  after the wall ordering, Arachne after `traverse_extrusions`); loop-count clamp in band mode in both
  generators; gap-fill suppression for the dropped onion steps; band-mode early return in
  `apply_extra_perimeters()` and `process_no_bridge()`; one-wall options gated off in band mode.
- `LayerRegion.cpp`: set `sublayer_drop_walls` on the core run, then one band run per sub-slice.

The drop helper owns the entities it removes and must `delete` them — the collection is the owner.

**Prerequisite found during implementation:** only the classic generator was writing
`ExtrusionEntity::inset_idx`. Arachne read `inset_idx` off its own `Arachne::ExtrusionLine` but never
carried it onto the emitted entity, leaving every wall at the default `-1`. Since Arachne is the
default generator, a filter keyed on `inset_idx` silently dropped *all* walls there. `traverse_extrusions`
now sets it at each append site, which is also what the classic generator has always done.

## Stage 4 — Emission

`feat(gcode): emit sub-layered wall passes at ascending Z before the layer`

- `GCode.hpp`: `m_in_sublayer_wall_pass`; `extrude_sublayer_walls()` declared **after** the
  `InstanceVisit` struct it takes.
- `GCode.cpp`: the function itself next to `extrude_perimeters`; the call right after the tool change
  and before the instance loop; the scarf and wipe-before-external-loop gates.

A pass holds one collection per island, so the emitter flattens one level before calling
`extrude_entity`, which cannot dispatch a collection.

## Stage 5 — Conflict handling

`feat(gui): guard sub-layered walls against conflicting settings`

- `ConfigManipulation.cpp`: spiral-vase dialog, sub-layer-height clamp notice, the two toggles.
- `Print.cpp`: spiral vase and MMU-painting errors, `make_overhang_printable` and `zaa_enabled`
  warnings — `validate()` is the only place per-object and per-modifier overrides are seen.

## Stage 6 — Tests

`test(fff_print): cover sub-layered walls incl. Z monotonicity and slope detail`

`tests/fff_print/test_wall_sublayers.cpp` plus its `CMakeLists.txt` entry.

## Verification

```bash
cmake --build build --config Release --target fff_print_tests -j 24
./build/tests/fff_print/Release/fff_print_tests "[WallSublayers]"
ctest --test-dir build/tests --output-on-failure
```

Two traps worth knowing about:

- G-code is **not** byte-reproducible between two slices in one process: the export timestamp and a
  process-wide object-id counter differ. Compare parsed commands, not raw text.
- A test helper must not return `std::initializer_list` — its backing array dies with the call, and
  the result is a segfault at slice time rather than at the call site.
