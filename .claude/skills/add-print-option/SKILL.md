---
name: add-print-option
description: Add a new print process setting to OrcaSlicer end to end - config definition, preset and GUI registration, step invalidation, region compatibility, conflict handling and tests. Use whenever a task adds, renames or removes a print/printer/filament setting, or when a newly added setting does not appear in the UI, is not saved to presets or 3MF, or forces a full re-slice on every edit.
---

# Adding a print option to OrcaSlicer

A setting is only "added" once every registration site knows about it. Missing one does not produce a
compile error — it produces a specific runtime symptom, listed below.

Work through the checklist in order. Skip a step only when the "when" column clearly does not apply.

## 1. Define the option — `src/libslic3r/PrintConfig.cpp`

In `PrintConfigDef::init_fff_params()`, next to the options it belongs with (that grouping is also
roughly the order of the settings tab):

```cpp
def = this->add("my_option", coFloatOrPercent);
def->label    = L("Human readable name");
def->category = L("Quality");            // buckets it in the per-part "Add settings" menu
def->tooltip  = L("What it does, and what 0 / the default means.");
def->sidetext = L("mm or %");            // "%" alone is not wrapped in L()
def->ratio_over = "layer_height";        // see the trap in step 3
def->min      = 0;
def->mode     = comAdvanced;             // comSimple / comAdvanced / comExpert / comDevelop
def->set_default_value(new ConfigOptionFloatOrPercent(0, false));
```

**Choose a default that means "off".** A feature gated by an option must not change existing behavior
when the option is at its default — that is a repo-wide rule, and it is what makes the option safe for
existing profiles and projects.

A literal `%` in a tooltip needs `// xgettext:no-c-format, no-boost-format` above it or `msgfmt` fails.

## 2. Pick the config class — `src/libslic3r/PrintConfig.hpp`

Add the key to one `PRINT_CONFIG_CLASS_DEFINE` sequence. The class decides the override scope, and
that is the whole decision:

| Class | Overridable |
|---|---|
| `PrintObjectConfig` | per object |
| `PrintRegionConfig` | per object, per modifier mesh, **and** per height range |
| `PrintConfig` / `GCodeConfig` | whole print only (`GCodeConfig` is also written into the G-code) |

## 3. Register it everywhere

| File | What | Symptom if missed |
|---|---|---|
| `src/libslic3r/Preset.cpp` | key in `s_Preset_print_options` | Never saved to presets or 3MF, and absent from every override tab |
| `src/slic3r/GUI/Tab.cpp` | `optgroup->append_single_option_line("my_option", "<wiki_page>#<anchor>")` in `TabPrint::build()` | Invisible in the settings tab |
| `src/slic3r/GUI/GUI_Factories.cpp` | entry in `PART_CATEGORY_SETTINGS` (region keys) or `OBJECT_CATEGORY_SETTINGS` | Missing from the right-click "Add settings" menu |
| `src/libslic3r/PrintObject.cpp` | key in the right bucket of `invalidate_state_by_config_options` | Falls through to `invalidate_all_steps()` — a **full re-slice on every edit** |
| `src/libslic3r/Print.cpp` | print-level keys: `steps_gcode` for G-code-only keys, else the right bucket | same |
| `src/libslic3r/Layer.cpp` | region keys that affect wall generation: compare it in `is_perimeter_compatible` | Two regions with different values are merged into one `make_perimeters()` call and **silently share the first one's value** |

Pick the invalidation bucket by the earliest pipeline step the option actually changes: `posSlice`
(slicing geometry), `posPerimeters` (wall generation), `posInfill`, or `psGCodeExport` (emission only,
invalidated directly rather than through `steps`). Being too coarse costs the user re-slices; being too
fine ships stale G-code.

**Trap: `ratio_over` does not resolve across config classes.** `ConfigBase::get_abs_value` looks the
base key up in the same config object. A `PrintRegionConfig` option with `ratio_over = "layer_height"`
(a `PrintObjectConfig` key) throws if resolved that way, so resolve it by hand against the real value —
which is also what makes adaptive and per-range layer heights work:

```cpp
const double v = config.my_option.get_abs_value(layer->height);   // real height, not the setting
```

## 4. Handle conflicts

Two layers, and you usually want both:

- `src/slic3r/GUI/ConfigManipulation.cpp`
  - `update_print_fff_config()` — a `MessageDialog` that offers to fix the conflicting setting.
    Follow the spiral-vase block: yes applies the fix, no reverts the setting that triggered it.
    For a value that is merely out of range, follow the clamp-and-notify blocks instead.
  - `toggle_print_fff_options()` — `toggle_field` (grey out) / `toggle_line` (hide) for dependents.
- `Print::validate()` in `src/libslic3r/Print.cpp` — for anything that must never reach the slicer.
  **Per-object and per-modifier overrides never pass through `ConfigManipulation`**, so a GUI dialog
  alone does not protect a region key. Return a `StringObjectException` for an error, or use the local
  `warn()` lambda for a warning; the `opt_key` deep-links the user to the setting.

## 5. Test it — `tests/fff_print/`

See [tests/AGENTS.md](../../../tests/AGENTS.md). At minimum:

- The behavior the option enables.
- **A regression guard that the default changes nothing**, since that is the compatibility promise.
  G-code is not byte-reproducible between two slices in one process — the export timestamp and a
  process-wide object-id counter differ — so compare commands parsed with `GCodeReader`, not raw text.

Check `tests/fff_print/test_helpers.hpp` before writing setup or parsing code; `slice`,
`init_and_process_print`, `layers_with_role`, `role_passes` and `role_sequence` already exist. Add the
new file to `tests/fff_print/CMakeLists.txt` in the same change.

## 6. Things you do not need

- `handle_legacy` — only for renames and value remaps, not for a new key.
- `resources/profiles/**` — the `set_default_value` is the source of truth. If you do touch a vendor
  profile, bump `version` in the sibling `resources/profiles/<Vendor>.json`.
- Editing `.po` files — the `L()` macros are the registration; the template is regenerated.
- `print_options_with_variant` — only for per-extruder-variant vector options.

Old builds silently ignore unknown keys, and new builds fall back to the default for old projects, so
a brand-new key needs no migration.

## Verify

```bash
cmake --build build --config Release --target fff_print_tests -j 24
ctest --test-dir build/tests --output-on-failure
```

Then confirm in the app that the option appears in the settings tab, in the per-object override tab,
and in the right-click "Add settings" menu, and that changing it re-slices only what it should.
