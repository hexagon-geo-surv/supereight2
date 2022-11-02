/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/common/weight.hpp>

TEST(Weight, increment)
{
    se::weight_t w = 0;
    for (int i = 0; i < 130; i++) {
        se::weight::increment(w, 127);
    }
    ASSERT_EQ(w, 127);
    for (int i = 0; i < 130; i++) {
        se::weight::increment(w, UINT8_MAX);
    }
    ASSERT_EQ(w, UINT8_MAX);
}

TEST(Weight, mean)
{
    constexpr se::weight_t max_weight = UINT8_MAX;
    se::delta_weight_t d = 0;
    for (int i = 0; i < 8; i++) {
        d += max_weight;
    }
    ASSERT_EQ(d, 8 * static_cast<se::delta_weight_t>(max_weight));
    ASSERT_EQ(d / 8, static_cast<se::delta_weight_t>(max_weight));
    ASSERT_EQ(se::weight::div(d, 8), static_cast<se::delta_weight_t>(max_weight));
    d /= 8;
    ASSERT_EQ(d, static_cast<se::delta_weight_t>(max_weight));
}
