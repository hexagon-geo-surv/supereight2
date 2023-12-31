/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <queue>
#include <sstream>

#include "se/integrator/map_integrator.hpp"

class OccupancyIntegrator : public ::testing::Test {
    protected:
    OccupancyIntegrator() : map_(Eigen::Vector3f::Constant(12.8f), 0.1f)
    {
        // Create a sensor.
        constexpr int w = 100;
        constexpr int h = w;
        const Eigen::Matrix4f T_BS =
            (Eigen::Matrix4f() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1).finished();
        const se::PinholeCamera sensor(
            {{w, h, 0.1f, 10.0f, T_BS}, 100.0f, 100.0f, w / 2 - 0.5f, h / 2 - 0.5f});
        // Create a z-up map with a wall integrated perpendicular to the the x axis at depth_value.
        constexpr float depth_value = 4.0f;
        const se::Image<float> depth(
            sensor.model.imageWidth(), sensor.model.imageHeight(), depth_value);
        const Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity();
        se::MapIntegrator integrator(map_);
        integrator.integrateDepth(sensor, depth, T_WB * T_BS, 0);
    }

    public:
    typedef se::OccupancyMap<>::OctreeType::NodeType NodeType;
    typedef se::OccupancyMap<>::OctreeType::BlockType BlockType;
    typedef se::OccupancyMap<>::OctreeType::DataType DataType;

    se::OccupancyMap<> map_;

    static void expect_valid_node_data(const NodeType& node, const se::OctantBase* const child)
    {
        const DataType& min_data = child->isBlock()
            ? static_cast<const BlockType*>(child)->getMinData()
            : static_cast<const NodeType*>(child)->getMinData();
        const DataType& max_data = child->isBlock()
            ? static_cast<const BlockType*>(child)->getMaxData()
            : static_cast<const NodeType*>(child)->getMaxData();

        std::stringstream failure_message;
        failure_message << "for node (" << node.getCoord().transpose() << ") with size "
                        << node.getSize();
        if (min_data.observed) {
            EXPECT_LE(node.getMinData().occupancy, min_data.occupancy) << failure_message.str();
        }
        if (max_data.observed) {
            EXPECT_GE(node.getMaxData().occupancy, max_data.occupancy) << failure_message.str();
        }
    }

    static void expect_valid_scale_data(const BlockType& block,
                                        const Eigen::Vector3i& parent_coord,
                                        const int scale)
    {
        const int stride = 1 << scale;
        const auto& parent_min_data = block.getMinData(parent_coord, scale);
        const auto& parent_max_data = block.getMaxData(parent_coord, scale);
        for (int z = 0; z < stride; z += stride / 2) {
            for (int y = 0; y < stride; y += stride / 2) {
                for (int x = 0; x < stride; x += stride / 2) {
                    const Eigen::Vector3i child_coord = parent_coord + Eigen::Vector3i(x, y, z);
                    const auto& child_min_data = block.getMinData(child_coord, scale);
                    const auto& child_max_data = block.getMaxData(child_coord, scale);
                    std::stringstream failure_message;
                    failure_message << "for block (" << parent_coord.transpose() << ") at scale "
                                    << scale;
                    if (child_min_data.observed) {
                        EXPECT_LE(parent_min_data.occupancy, child_min_data.occupancy)
                            << failure_message.str();
                    }
                    if (child_max_data.observed) {
                        EXPECT_GE(parent_max_data.occupancy, child_max_data.occupancy)
                            << failure_message.str();
                    }
                }
            }
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(OccupancyIntegrator, propagation)
{
    std::queue<const NodeType*> nodes;
    std::queue<const BlockType*> blocks;
    const NodeType* const root = static_cast<const NodeType*>(map_.getOctree().getRoot());
    ASSERT_TRUE(root);
    nodes.push(root);

    // Traverse the octree breadth-first from the root and test propagation at the node level.
    while (!nodes.empty()) {
        const NodeType* const node = nodes.front();
        nodes.pop();
        // Test the data of all children are within the minimum and maximum data of the parent.
        for (int child_idx = 0; child_idx < 8; child_idx++) {
            const se::OctantBase* const child = node->getChild(child_idx);
            if (child) {
                // Get the child min/max data and add it to the appropriate traversal queue.
                if (child->isBlock()) {
                    blocks.push(static_cast<const BlockType*>(child));
                }
                else {
                    nodes.push(static_cast<const NodeType*>(child));
                }
                expect_valid_node_data(*node, child);
            }
        }
    }

    // Test propagation within blocks.
    while (!blocks.empty()) {
        const BlockType* const block = blocks.front();
        blocks.pop();
        // Test each voxel for all scales except the finest.
        for (int scale = block->getMaxScale(); scale > block->getCurrentScale(); scale--) {
            const int stride = 1 << scale;
            for (int z = 0; z < block->size; z += stride) {
                for (int y = 0; y < block->size; y += stride) {
                    for (int x = 0; x < block->size; x += stride) {
                        const Eigen::Vector3i voxel_coord =
                            block->getCoord() + Eigen::Vector3i(x, y, z);
                        expect_valid_scale_data(*block, voxel_coord, scale);
                    }
                }
            }
        }
    }
}
