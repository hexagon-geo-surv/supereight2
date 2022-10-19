/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/common/colour_types.hpp>

TEST(ColourTypes, scalarOperators)
{
    const se::rgb_t a{10, 20, 30};
    const se::rgb_t a055{6, 11, 17};
    const se::rgb_t a3{30, 60, 90};
    const se::rgb_t a10_overflow{100, 200, 44};

    EXPECT_EQ((a * 0.55f), a055);
    EXPECT_EQ((a * uint8_t(3)), a3);
    EXPECT_EQ((a * uint8_t(10)), a10_overflow);

    EXPECT_EQ((a3 / uint8_t(3)), a);
    EXPECT_EQ((a055 / 0.55f), a);
}

TEST(ColourTypes, rgbOperators)
{
    const se::rgb_t a{10, 20, 30};
    const se::rgb_t b{40, 50, 60};
    const se::rgb_t apb{50, 70, 90};
    const se::rgb_t apb5_overflow{210, 14, 74};

    EXPECT_EQ((a + b), apb);
    EXPECT_EQ((b + a), apb);
    EXPECT_EQ((a + 5 * b), apb5_overflow);

    EXPECT_EQ((apb - b), a);
    EXPECT_EQ((apb - a), b);
}

TEST(ColourTypes, differentSizeof)
{
    se::rgb16s_t s{0, 0, 0};
    const se::rgb16s_t s1{100, 20, 30};
    const se::rgb16s_t s2{200, 40, 60};
    const se::rgb16s_t s3{300, 60, 90};
    const se::rgb16s_t s_avg{100, 20, 30};
    const se::rgb_t a{100, 20, 30};

    s += a;
    EXPECT_EQ(s, s1);
    s += a;
    EXPECT_EQ(s, s2);
    s += a;
    EXPECT_EQ(s, s3);
    s /= 3;
    EXPECT_EQ(s, s_avg);
}
