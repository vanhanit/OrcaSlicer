// Orca: fixed bead count for sub-layered walls.

#include "FixedCountBeadingStrategy.hpp"

#include <algorithm>
#include <limits>

namespace Slic3r::Arachne
{

FixedCountBeadingStrategy::FixedCountBeadingStrategy(coord_t bead_count, coord_t max_stretch_width, BeadingStrategyPtr parent)
    : BeadingStrategy(*parent)
    , m_bead_count(std::max<coord_t>(bead_count, 1))
    , m_max_bead_width(max_stretch_width)
    , m_max_thickness(std::max<coord_t>(bead_count, 1) * max_stretch_width)
    , m_parent(std::move(parent))
{
    name = "FixedCountBeadingStrategy";
}

coord_t FixedCountBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
{
    const coord_t natural = m_parent->getOptimalBeadCount(thickness);
    // Nothing at all below the minimum feature size stays nothing, and a region too thick to be a wall
    // this band could stretch over is left to the strategy underneath.
    if (natural == 0 || thickness > m_max_thickness)
        return natural;
    return m_bead_count;
}

BeadingStrategy::Beading FixedCountBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
{
    if (bead_count == 0 || getOptimalBeadCount(thickness) != m_bead_count)
        return m_parent->compute(thickness, bead_count);

    Beading held = m_parent->compute(thickness, m_bead_count);
    // Holding the count spreads the thickness over the beads the strategy underneath favours, and with
    // a low wall_distribution_count that is one bead in the middle taking nearly all of it. A bead
    // several times the width it was asked for is not a wall any more, so where that happens the count
    // the strategy underneath wanted is used after all and the leftover goes back to being a gap.
    for (const coord_t width : held.bead_widths)
        if (width > m_max_bead_width)
            return m_parent->compute(thickness, bead_count);
    return held;
}

coord_t FixedCountBeadingStrategy::getOptimalThickness(coord_t bead_count) const
{
    return m_parent->getOptimalThickness(bead_count);
}

coord_t FixedCountBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
{
    // The count does not change with thickness while this strategy is in charge, so there is no
    // thickness at which it steps up: below the fixed count it is already there, at or above it the
    // step only happens once the region is too thick for this strategy to hold.
    if (lower_bead_count < m_bead_count)
        return 0;
    if (lower_bead_count == m_bead_count)
        return m_max_thickness;
    return m_parent->getTransitionThickness(lower_bead_count);
}

coord_t FixedCountBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
{
    return m_parent->getTransitioningLength(lower_bead_count);
}

float FixedCountBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
{
    // The base class derives this from the thicknesses either side of a step, which are equal here for
    // every step this strategy owns and would divide by zero.
    return lower_bead_count < m_bead_count ? 1.f : 0.5f;
}

std::string FixedCountBeadingStrategy::toString() const
{
    return std::string("FixedCount+") + m_parent->toString();
}

} // namespace Slic3r::Arachne
