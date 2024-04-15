/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/common/rgb.hpp>

TEST(RGB, operators)
{
    const se::RGB black;
    const se::RGB gray{0x7F, 0x7F, 0x7F};
    const se::RGB white{0xFF, 0xFF, 0xFF};

    // Arithmetic
    ASSERT_EQ(black + white, white);
    ASSERT_EQ(0.5f * black, black);
    ASSERT_EQ(0.5f * white, gray);

    // Comparison
    ASSERT_TRUE(black == black);
    ASSERT_TRUE(black != white);
    ASSERT_TRUE(black < white);
    ASSERT_TRUE(white > black);
    ASSERT_TRUE(black <= white);
    ASSERT_TRUE(white >= black);
    ASSERT_TRUE(black <= black);
    ASSERT_TRUE(white >= white);

    // Linear interpolation
    const se::RGB dark_gray{0x4C, 0x4C, 0x4C};
    const se::RGB light_gray{0xB2, 0xB2, 0xB2};
    ASSERT_EQ(0.5f * black + 0.5f * white, gray);
    ASSERT_EQ(0.7f * black + 0.3f * white, dark_gray);
    ASSERT_EQ(0.3f * black + 0.7f * white, light_gray);
}
