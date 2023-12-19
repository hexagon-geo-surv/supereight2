/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/common/bounded_vector.hpp>

// Empty class just so we can define a typed test suite.
template<typename T>
class BoundedVectorTest : public ::testing::Test {
};

using BoundedVectorTestTypes = ::testing::Types<int, float>;
TYPED_TEST_SUITE(BoundedVectorTest, BoundedVectorTestTypes);

TYPED_TEST(BoundedVectorTest, test)
{
    se::BoundedVector<TypeParam, 8> v;
    constexpr size_t capacity = decltype(v)::allocator_type::capacity;

    EXPECT_TRUE(v.empty());
    EXPECT_EQ(v.size(), 0u);

    for (size_t i = 0; i < capacity; i++) {
        EXPECT_EQ(v.size(), i);
        v.push_back(i);
        EXPECT_EQ(v.size(), i + 1);
        EXPECT_FALSE(v.empty());
        EXPECT_EQ(v[i], i);
        EXPECT_EQ(v.at(i), i);
    }
    EXPECT_EQ(v.size(), capacity);
    EXPECT_EQ(v.capacity(), capacity);

    // Attempt to overfill.
    EXPECT_THROW(v.push_back(0), std::bad_alloc);

    v.clear();
    EXPECT_TRUE(v.empty());
    EXPECT_EQ(v.size(), 0u);
    EXPECT_EQ(v.capacity(), capacity);

    // Test aggregate initialization.
    v = {0, 1, 2, 3};
    EXPECT_EQ(v.size(), 4u);
    for (size_t i = 0; i < v.size(); i++) {
        EXPECT_EQ(v[i], i);
    }

    // Test that copies do not share data.
    const auto vv = v;
    ASSERT_EQ(vv.size(), v.size());
    for (size_t i = 0; i < vv.size(); i++) {
        EXPECT_EQ(vv[i], v[i]);
    }
    v = {0, 2, 4, 6};
    ASSERT_EQ(vv.size(), 4u);
    for (size_t i = 0; i < vv.size(); i++) {
        EXPECT_EQ(vv[i], i);
    }
}
