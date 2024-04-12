/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_RGB_HPP
#define SE_COMMON_RGB_HPP

#include <cstdint>

namespace se {

/** A colour represented as a Red-Green-Blue tuple with 8-bits per channel. */
struct RGB {
    std::uint8_t r = 0x00;
    std::uint8_t g = 0x00;
    std::uint8_t b = 0x00;
};

} // namespace se

#endif // SE_COMMON_RGB_HPP
