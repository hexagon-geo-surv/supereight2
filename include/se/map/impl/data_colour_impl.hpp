/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_COLOUR_IMPL_HPP
#define SE_DATA_COLOUR_IMPL_HPP

namespace se {

inline bool ColourData<Colour::On>::update(const colour_t colour_, const std::uint8_t max_weight)
{
    // Avoid overflow if max_weight is equal to the maximum value of weight_t.
    if (weight < max_weight) {
        weight++;
    }
    // The following code relies on the implicit conversion to int or larger (if weight_t is an
    // integral type) or float or larger (if weight_t is a floating-point type) to avoid overflows.
    static_assert(sizeof(colour_t::r) < sizeof(int));
    colour.r = (colour.r * (weight - weight_t(1)) + colour_.r) / weight;
    colour.g = (colour.g * (weight - weight_t(1)) + colour_.g) / weight;
    colour.b = (colour.b * (weight - weight_t(1)) + colour_.b) / weight;
    return true;
}

} // namespace se

#endif // SE_DATA_COLOUR_IMPL_HPP
