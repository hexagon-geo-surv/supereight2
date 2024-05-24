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
        const Eigen::Isometry3f T_BS(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY())
                                     * Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
        const se::PinholeCamera sensor(
            {{w, h, 0.1f, 10.0f, T_BS}, 100.0f, 100.0f, w / 2 - 0.5f, h / 2 - 0.5f});
        // Create a z-up map with a wall integrated perpendicular to the the x axis at 4 meters.
        const se::Image<float> depth(sensor.model.imageWidth(), sensor.model.imageHeight(), 4.0f);
        Eigen::Isometry3f T_WB = Eigen::Isometry3f::Identity();
        int frame = 0;
        se::MapIntegrator integrator(map_);
        integrator.integrateDepth(sensor, depth, T_WB * T_BS, frame++);
        // Integrate the wall from a second, partially-overlapping pose so as to get some data with
        // weight greater than 1.
        T_WB.translation().y() += 1;
        integrator.integrateDepth(sensor, depth, T_WB * T_BS, frame++);
    }

    public:
    typedef se::OccupancyMap<>::OctreeType::NodeType NodeType;
    typedef se::OccupancyMap<>::OctreeType::BlockType BlockType;
    typedef se::OccupancyMap<>::OctreeType::DataType DataType;

    se::OccupancyMap<> map_;



    static void expect_valid_node_data(const NodeType& node, const se::OctantBase* const child)
    {
        const DataType& min_data = child->is_block
            ? static_cast<const BlockType*>(child)->getMinData()
            : static_cast<const NodeType*>(child)->getMinData();
        const DataType& max_data = child->is_block
            ? static_cast<const BlockType*>(child)->getMaxData()
            : static_cast<const NodeType*>(child)->getMaxData();

        std::stringstream failure_message;
        failure_message << "for node (" << node.coord.transpose() << ") with size "
                        << node.getSize();
        if (min_data.field.observed) {
            EXPECT_LE(se::get_field(node.getMinData()), se::get_field(min_data))
                << failure_message.str();
        }
        if (max_data.field.observed) {
            EXPECT_GE(se::get_field(node.getMaxData()), se::get_field(max_data))
                << failure_message.str();
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
                    if (child_min_data.field.observed) {
                        EXPECT_LE(se::get_field(parent_min_data), se::get_field(child_min_data))
                            << failure_message.str();
                    }
                    if (child_max_data.field.observed) {
                        EXPECT_GE(se::get_field(parent_max_data), se::get_field(child_max_data))
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
                if (child->is_block) {
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
                            block->coord + Eigen::Vector3i(x, y, z);
                        expect_valid_scale_data(*block, voxel_coord, scale);
                    }
                }
            }
        }
    }
}
