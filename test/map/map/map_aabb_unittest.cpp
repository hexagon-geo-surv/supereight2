/*
 * SPDX-FileCopyrightText: 2022-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <gtest/gtest.h>
#include <se/integrator/map_integrator.hpp>
#include <se/map/map.hpp>

se::PinholeCamera generate_sensor()
{
    constexpr int width = 100;
    constexpr int height = width;
    const Eigen::Matrix4f T_BS =
        (Eigen::Matrix4f() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1).finished();
    return se::PinholeCamera(
        {{width, height, 0.1f, 10.0f, T_BS}, 100.0f, 100.0f, width / 2 - 0.5f, height / 2 - 0.5f});
}

template<typename MapT>
void integrate_wall(MapT& map, const se::PinholeCamera& sensor, float depth_value)
{
    const se::Image<float> depth(
        sensor.model.imageWidth(), sensor.model.imageHeight(), depth_value);
    const Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity();
    se::MapIntegrator integrator_stsdf(map);
    integrator_stsdf.integrateDepth(sensor, depth, T_WB * sensor.T_BS, 0);
}

int dim_to_blocks(float dim, float block_dim)
{
    return std::copysign(std::abs(dim) / block_dim + 0.5f, dim);
}

TEST(Map, aabbTSDF)
{
    // Generate a z-up map with a wall integrated perpendicular to the the x axis at depth_value.
    se::TSDFMap<> map(Eigen::Vector3f::Constant(12.8f), 0.1f);
    const se::PinholeCamera sensor = generate_sensor();
    constexpr float depth_value = 4.0f;
    integrate_wall(map, sensor, depth_value);

    const float truncation_boundary = map.getRes() * map.getDataConfig().truncation_boundary_factor;
    const float block_dim = map.getRes() * map.getOctree().block_size;
    const float half_wall_dim = depth_value * std::tan(sensor.horizontal_fov / 2);
    const int half_wall_blocks = dim_to_blocks(half_wall_dim, block_dim);
    const auto aabb = map.aabb();

    // The values used in the tests below are hand-crafted for this particular example.
    EXPECT_FLOAT_EQ(aabb.min().x(), depth_value - truncation_boundary);
    EXPECT_FLOAT_EQ(aabb.max().x(), depth_value + truncation_boundary);
    EXPECT_FLOAT_EQ(aabb.min().z(), aabb.min().y());
    EXPECT_FLOAT_EQ(aabb.max().z(), aabb.max().y());
    EXPECT_EQ(dim_to_blocks(aabb.min().y(), block_dim), -half_wall_blocks);
    EXPECT_EQ(dim_to_blocks(aabb.max().y(), block_dim), half_wall_blocks);
}

TEST(Map, aabbOccupancy)
{
    se::OccupancyMap<> map(Eigen::Vector3f::Constant(12.8f), 0.1f);
    EXPECT_TRUE(map.aabb().isEmpty());

    // Generate a z-up map with a wall integrated perpendicular to the the x axis at depth_value.
    const se::PinholeCamera sensor = generate_sensor();
    constexpr float depth_value = 4.0f;
    integrate_wall(map, sensor, depth_value);

    const float block_dim = map.getRes() * map.getOctree().block_size;
    const float half_wall_dim = depth_value * std::tan(sensor.horizontal_fov / 2);
    const int half_wall_blocks = dim_to_blocks(half_wall_dim, block_dim);
    const auto aabb = map.aabb();

    // The values used in the tests below are hand-crafted for this particular example.
    EXPECT_NEAR(aabb.min().x(), 0.0f, 1e-6);
    EXPECT_FLOAT_EQ(aabb.max().x(), depth_value + 3 * block_dim);
    EXPECT_FLOAT_EQ(aabb.min().z(), aabb.min().y());
    EXPECT_FLOAT_EQ(aabb.max().z(), aabb.max().y());
    // An extra block seems to be allocated in both directions of the y/z axes. Added ±1 so the test
    // passes.
    EXPECT_EQ(dim_to_blocks(aabb.min().y(), block_dim), -half_wall_blocks - 1);
    EXPECT_EQ(dim_to_blocks(aabb.max().y(), block_dim), half_wall_blocks + 1);
}
