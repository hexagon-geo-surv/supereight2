/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_RGBA_HPP
#define SE_COMMON_RGBA_HPP

#include <cstdint>

namespace se {

/** A colour represented as a Red-Green-Blue-Alpha tuple with 8-bits per channel. */
struct RGBA {
    std::uint8_t r = 0x00;
    std::uint8_t g = 0x00;
    std::uint8_t b = 0x00;
    std::uint8_t a = 0xFF;
};

} // namespace se

#endif // SE_COMMON_RGBA_HPP
