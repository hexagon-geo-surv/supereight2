/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_RGB_HPP
#define SE_COMMON_RGB_HPP

#include <cassert>
#include <cstdint>
#include <tuple>

namespace se {

/** A colour represented as a Red-Green-Blue tuple with 8-bits per channel. */
struct RGB {
    std::uint8_t r = 0x00;
    std::uint8_t g = 0x00;
    std::uint8_t b = 0x00;


    /** Add \p lhs and \p rhs channel-wise. Needed for interpolation.
     *
     * \warning Assumes no overflow can happen.
     */
    // Passing lhs by value helps optimize chained additions:
    // https://en.cppreference.com/w/cpp/language/operators#Binary_arithmetic_operators
    friend RGB operator+(RGB lhs, const RGB rhs)
    {
        // Asserts work due to implicit conversion of std::uint8_t to int for arithmetic operators.
        assert(lhs.r + rhs.r <= std::numeric_limits<std::uint8_t>::max() && "Avoid overflow.");
        assert(lhs.g + rhs.g <= std::numeric_limits<std::uint8_t>::max() && "Avoid overflow.");
        assert(lhs.b + rhs.b <= std::numeric_limits<std::uint8_t>::max() && "Avoid overflow.");
        lhs.r += rhs.r;
        lhs.g += rhs.g;
        lhs.b += rhs.b;
        return lhs;
    }

    /** Scale all channels of \p rhs by the factor \p t in the interval `[0, 1]` inclusive. Needed
     * for interpolation.
     */
    friend RGB operator*(RGB lhs, const float t)
    {
        assert(t >= 0.0f && "Avoid under/overflow.");
        assert(t <= 1.0f && "Avoid under/overflow.");
        lhs.r *= t;
        lhs.g *= t;
        lhs.b *= t;
        return lhs;
    }

    /** \overload */
    friend RGB operator*(const float t, RGB rhs)
    {
        return rhs * t;
    }


    friend bool operator==(const RGB lhs, const RGB rhs)
    {
        return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b;
    }

    friend bool operator!=(const RGB lhs, const RGB rhs)
    {
        return !(lhs == rhs);
    }

    friend bool operator<(const RGB lhs, const RGB rhs)
    {
        // Perform lexicographical comparison.
        return std::tie(lhs.r, lhs.g, lhs.b) < std::tie(rhs.r, rhs.g, rhs.b);
    }

    friend bool operator>(const RGB lhs, const RGB rhs)
    {
        return rhs < lhs;
    }

    friend bool operator<=(const RGB lhs, const RGB rhs)
    {
        return !(lhs > rhs);
    }

    friend bool operator>=(const RGB lhs, const RGB rhs)
    {
        return !(lhs < rhs);
    }
};

} // namespace se

#endif // SE_COMMON_RGB_HPP
