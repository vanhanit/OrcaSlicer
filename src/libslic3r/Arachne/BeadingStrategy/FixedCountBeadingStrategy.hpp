// Orca: fixed bead count for sub-layered walls.

#ifndef FIXED_COUNT_BEADING_STRATEGY_H
#define FIXED_COUNT_BEADING_STRATEGY_H

#include <string>

#include "BeadingStrategy.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r::Arachne
{

/*!
 * A meta-strategy that asks for the same number of beads whatever the thickness is, letting the
 * strategy underneath stretch their widths to cover it instead of adding or dropping one.
 *
 * The wall of a sub-layer pass has to meet the walls the layer prints at its own full height, and the
 * two are apart by however far the model's surface moves over one sub-layer - a fraction of a line,
 * varying along the wall and with no relation to a whole number of them. Left to choose, the strategy
 * underneath answers that with a different number of beads in different places, which changes the wall
 * count around the perimeter and overrides the width the sub-layer walls were asked to print at.
 * Holding the count instead turns the same variation into width, which is what the underlying
 * strategies vary smoothly and what RedistributeBeadingStrategy keeps off the outermost bead, so the
 * surface stays where the model puts it and the give is taken up on the inside.
 *
 * Beyond max_stretch_width per bead the region is not a wall to be stretched over any more, and the
 * count the strategy underneath asked for is used unchanged.
 */
class FixedCountBeadingStrategy : public BeadingStrategy
{
public:
    FixedCountBeadingStrategy(coord_t bead_count, coord_t max_stretch_width, BeadingStrategyPtr parent);

    ~FixedCountBeadingStrategy() override = default;

    Beading     compute(coord_t thickness, coord_t bead_count) const override;
    coord_t     getOptimalThickness(coord_t bead_count) const override;
    coord_t     getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t     getOptimalBeadCount(coord_t thickness) const override;
    coord_t     getTransitioningLength(coord_t lower_bead_count) const override;
    float       getTransitionAnchorPos(coord_t lower_bead_count) const override;
    std::string toString() const override;

protected:
    const coord_t            m_bead_count;
    const coord_t            m_max_bead_width;
    const coord_t            m_max_thickness;
    const BeadingStrategyPtr m_parent;
};

} // namespace Slic3r::Arachne
#endif // FIXED_COUNT_BEADING_STRATEGY_H
