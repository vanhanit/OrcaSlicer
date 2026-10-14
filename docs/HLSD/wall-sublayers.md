# Sub-layered outer walls — High Level Design

## Purpose and scope

`wall_sublayer_height` prints the outermost wall loops of a layer as a stack of
thinner passes at ascending Z, while the inner walls, the infill and everything
else of the layer still print once at the full layer height. Every pass is
generated from the model re-sliced at its own height, so a sloped surface gains
real vertical resolution instead of repeating the layer's contour, at the cost of
print time on the outer walls only.

`wall_sublayer_loops` sets how many outermost loops the passes take over,
`wall_sublayer_line_width` narrows the pass extrusions, `wall_sublayer_speed`
and the filament's `wall_sublayer_fan_speed` give them their own speed and
cooling, and `wall_sublayer_fill_pattern` picks how a pass fills the step a
sloped surface leaves under the pass above it.

## Splitting a layer

`wall_sublayer_count()` resolves a region's sub-layer height against the real
height of each layer, so adaptive and per-range layer heights are honored, and
clamps the result to `MAX_WALL_SUBLAYERS` passes no thinner than
`MIN_WALL_SUBLAYER_HEIGHT`. The last pass always ends exactly at the layer's
`print_z`.

`PrintObject::slice_wall_sublayers()` runs at the start of `posPerimeters`. For
every layer it builds one split per pass count its regions ask for, finest
first (`Layer::wall_sublayer_splits`), so a modifier asking for 2 passes and the
object around it asking for 4 each get their own planes. A region that asks for
none keeps its walls at the full layer height even when the rest of the layer is
split. The planes of the whole object are sliced in one go and distributed over
the regions with the same `slice_volumes_inner()` / `slices_to_regions()` the
layer slices use, with the same XY hole and contour compensation. Two details
keep them in line with the layer:

- Planes are placed around the layer's own `slice_z`, not its `print_z`: a belt
  printer moves `print_z` after slicing while the geometry stays where it was
  sliced.
- Planes are distributed over the regions of the layer range the layer's own
  `slice_z` falls in, even when a range boundary lies inside the layer, so a
  pass above the boundary is not handed to regions the layer does not have.

Some steps split a layer's slices between regions after slicing, in a way the
volumes cannot reproduce: colour and fuzzy skin painting, and interlocking
beams. On such objects the sub-slices are split the way the layer is: an area
inside a region's slice of the layer goes to that region, and an area beyond the
layer's outline goes to the nearest region by wave expansion
(`Algorithm::expand_expolygons()`).

The first layer is never split, nor are the layers elephant foot compensation
shrinks, because their slices are reshaped after slicing in a way the sub-slices
would not follow.

## Generating the walls

`LayerRegion::make_perimeters()` asks `wall_sublayer_prepare()` for the areas of
a perimeter run (`WallSublayerContext`):

- The **core**: where the model is solid for the whole layer height. The
  layer's own run is clipped to it and drops the outermost `band_walls` loops,
  which the passes print instead. Anything no pass can print - fill over air,
  the ceiling of a cavity closing over - goes back to the core, which bridges it
  at `print_z` the way an ordinary layer does.
- The **band**: per pass, the region re-sliced at that pass's height.
  `wall_sublayer_generate()` runs the perimeter generator once per pass on it
  (`LayerRegion::sublayer_perimeters`), then fills the tread a sloped surface
  leaves between the band and the core with solid fill and gap fill at the
  sub-layer height.

Each pass is classified against what actually exists under it: the passes
below it and the top of the layer below, not the model re-sliced at its height.
A pass wall may overhang what is under it, but a wall with nothing under it at
all is dropped and its strip handed to the core, which bridges it. A pass prints
before the layer's infill, so `unsupported_wall_last` never holds one of its loops
back for the infill to anchor.

## Printing

`GCode::extrude_sublayer_walls()` prints the passes of a filament before
anything else of the layer: every pass height of every instance and region, in
ascending order, before the next height starts. Regions split differently
interleave by height. Z therefore only rises within a layer and the nozzle never
crosses material printed above it. Each pass height is a preview layer of its
own (`GCodeProcessor::Sub_Layer_Tag`), while the printer and the time estimate
still see one layer. The overhang estimator measures a pass against the
sub-slice below it (`Layer::wall_sublayer_support()`), and the cooling buffer
asks for the minimum layer time once per pass of the most finely split region.

## Filaments

Everything printed at a layer's `print_z` stands a full layer high once it is
down, so a filament printing passes after another filament has printed there
would bring the nozzle down beside it. Hence:

- `ToolOrdering::put_sublayer_wall_extruders_first()` moves the filament
  printing sub-layered walls to the front of its layers, over the flush
  minimizing and cyclic orders; a mixed filament's component order takes
  precedence where a layer has one.
- An object whose sub-layered walls would need two filaments is rejected by
  `Print::validate()`: only one can go first.
- With a separate inner wall filament, the passes take only the outer wall, so
  the inner walls keep their own filament.
- Walls painted in a filament other than their part's, and walls in a mixed
  filament split into colour sub-layers (`enable_mixed_color_sublayer`, which
  already divides the layer in Z per component), are not sub-layered; they print
  at the full layer height after the passes, and `Print::validate()` says so.

## Constraints

- Spiral vase mode is rejected: it rewrites a layer around a single Z ramp.
- `make_overhang_printable` and Z anti-aliasing produce a warning: the passes are
  sliced from the original model, and anti-aliasing refines the same surfaces.
- With the option off nothing of the above runs, and the G-code is unchanged.
