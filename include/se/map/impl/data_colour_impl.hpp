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
    // integral type) or to float or larger (if weight_t is a floating-point type) in order to avoid
    // overflows.
    static_assert(sizeof(colour_t::r) < sizeof(int));
    static_assert(sizeof(colour_t::g) < sizeof(int));
    static_assert(sizeof(colour_t::b) < sizeof(int));
    colour.r = (colour.r * (weight - weight_t(1)) + colour_.r) / weight;
    colour.g = (colour.g * (weight - weight_t(1)) + colour_.g) / weight;
    colour.b = (colour.b * (weight - weight_t(1)) + colour_.b) / weight;
    return true;
}



inline void
ColourData<Colour::On>::setToMean(const BoundedVector<ColourData<Colour::On>, 8>& child_data)
{
    static_assert(sizeof(colour_t::r) < sizeof(int));
    static_assert(sizeof(colour_t::g) < sizeof(int));
    static_assert(sizeof(colour_t::b) < sizeof(int));
    static_assert(std::is_floating_point_v<weight_t>,
                  "The code must be modified to avoid overflows for non-floating-point weight_t.");
    const int n = child_data.size();
    int r_sum = 0;
    int g_sum = 0;
    int b_sum = 0;
    weight_t weight_sum(0);
    for (const auto d : child_data) {
        r_sum += d.colour.r;
        g_sum += d.colour.g;
        b_sum += d.colour.b;
        weight_sum += d.weight;
    }
    colour.r = r_sum / n;
    colour.g = g_sum / n;
    colour.b = b_sum / n;
    weight = std::ceil(weight_sum / n);
}

} // namespace se

#endif // SE_DATA_COLOUR_IMPL_HPP
