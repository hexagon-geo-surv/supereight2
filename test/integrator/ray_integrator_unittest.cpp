/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <math.h>
#include <queue>
#include <se/common/filesystem.hpp>
#include <se/integrator/map_integrator.hpp>
#include <se/map/map.hpp>

typedef se::OccupancyMap<se::Res::Multi, 8>::OctreeType::NodeType NodeType;
typedef se::OccupancyMap<se::Res::Multi, 8>::OctreeType::BlockType BlockType;
typedef se::OccupancyMap<se::Res::Multi, 8>::OctreeType::DataType DataType;
typedef se::Octree<DataType, se::Res::Multi, 8> OctreeType;

static void expect_valid_node_data(const NodeType& node, const se::OctantBase* const child)
{
    const DataType& min_data = child->isBlock() ? static_cast<const BlockType*>(child)->getMinData()
                                                : static_cast<const NodeType*>(child)->getMinData();

    const DataType& max_data = child->isBlock() ? static_cast<const BlockType*>(child)->getMaxData()
                                                : static_cast<const NodeType*>(child)->getMaxData();

    std::stringstream failure_message;
    failure_message << "for node (" << node.getCoord().transpose() << ") with size "
                    << node.getSize();
    if (min_data.observed) {
        EXPECT_LE(node.getMinData().occupancy * node.getMinData().weight,
                  min_data.occupancy * min_data.weight)
            << failure_message.str();
    }
    if (max_data.observed) {
        EXPECT_GE(node.getMaxData().occupancy * node.getMaxData().weight,
                  max_data.occupancy * max_data.weight)
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

                if (child_min_data.observed) {
                    EXPECT_LE(parent_min_data.occupancy * parent_min_data.weight,
                              child_min_data.occupancy * child_min_data.weight)
                        << failure_message.str();
                }
                if (child_max_data.observed) {
                    EXPECT_GE(parent_max_data.occupancy * parent_max_data.weight,
                              child_max_data.occupancy * child_max_data.weight)
                        << failure_message.str();
                }
            }
        }
    }
}

TEST(RayIntegrator, SingleRay)
{
    // Temporary directory for test results
    const std::string tmp_ = (stdfs::temp_directory_path() / stdfs::path("supereight_test_results")).string();
    stdfs::create_directories(tmp_);

    // Create simple test ray, straight line
    constexpr float d = 10.0f;

    // // ========= Map INITIALIZATION  =========
    constexpr float res = 0.05f;
    constexpr float dim = 25.6f;
    se::OccupancyMap<se::Res::Multi> map(Eigen::Vector3f::Constant(dim), res);

    // ========= Sensor INITIALIZATION  =========
    se::LeicaLidar::Config sensorConfig;
    sensorConfig.width = 360;
    sensorConfig.height = 180;
    sensorConfig.near_plane = 0.2f;
    sensorConfig.far_plane = 30.0f;
    sensorConfig.T_BS = Eigen::Isometry3f::Identity();
    sensorConfig.elevation_resolution_angle_ = 1.0f;
    sensorConfig.azimuth_resolution_angle_ = 1.0f;
    const se::LeicaLidar sensor(sensorConfig);

    // ========= Integrator INITIALIZATION  =========
    se::MapIntegrator integrator(map);

    // ========= Integration =========
    const Eigen::Vector3f ray(d, 0., 0.);
    std::vector<const se::OctantBase*> updated_octants;

    se::RayIntegrator rayIntegrator(
        map, sensor, ray, Eigen::Isometry3f::Identity(), 0, &updated_octants);
    rayIntegrator();
    rayIntegrator.propagateBlocksToCoarsestScale();
    rayIntegrator.propagateToRoot();
    rayIntegrator.updatedOctants(&updated_octants);
    EXPECT_FALSE(updated_octants.empty());
    std::cout << "Number of updated octants: " << updated_octants.size() << std::endl;

    // ========= TESTING =========

    // Allocation Level
    constexpr float resolution_along_ray = 0.01f;
    float di = sensor.near_plane;
    while (di <= d) {
        Eigen::Vector3f ri(di, 0.f, 0.f);
        Eigen::Vector3i voxel_coordinates;
        map.pointToVoxel(ri, voxel_coordinates);

        se::RayState rayState = rayIntegrator.computeVariance(ri.norm());

        /// Determine closest currently allocated octant
        const se::OctantBase* const finest_octant_ptr =
            se::fetcher::finest_octant<OctreeType>(voxel_coordinates, 0, map.getOctree().getRoot());
        EXPECT_TRUE(finest_octant_ptr->isBlock());

        const OctreeType::BlockType* const block_ptr =
            static_cast<const OctreeType::BlockType*>(finest_octant_ptr);
        const int allocatedScale = block_ptr->getCurrentScale();
        EXPECT_NE(allocatedScale, -1);


        Eigen::Vector3f block_centre_point;
        map.voxelToPoint(block_ptr->getCoord(), 8, block_centre_point);

        int computedIntegrationScale =
            sensor.computeIntegrationScale(block_centre_point, res, -1, -1, 3);
        if (rayState == se::RayState::FreeSpace && computedIntegrationScale < 1) {
            computedIntegrationScale = 1;
        }

        EXPECT_EQ(computedIntegrationScale, allocatedScale);

        di += resolution_along_ray;
    }

    // Un-Comment if debugging necessary (allocated structure visualization)
    map.saveStructure(tmp_ + "/single_ray_allocated_structure.ply");
}

TEST(RayIntegrator, Propagation)
{
    // Temporary directory for test results
    const std::string tmp_ = (stdfs::temp_directory_path() / stdfs::path("supereight_test_results")).string();
    stdfs::create_directories(tmp_);

    /**
   * Create plane wall example
   */
    const float elevation_min = -15.0f;
    const float elevation_max = 15.0f;
    const float azimuth_min = -15.0f;
    const float azimuth_max = 15.0f;
    // angular resolution [degree]
    const float elevation_res = .1f;
    const float azimuth_res = .1f;
    // conversion degree <-> rad
    constexpr float deg_to_rad = M_PI / 180.0f;
    // distance of plane wall [m]
    float d = 10.0f;

    std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>
        rayBatch;
    size_t num_points_elevation = std::floor((elevation_max - elevation_min) / elevation_res);
    size_t num_points_azimuth = std::floor((azimuth_max - azimuth_min) / azimuth_res);

    float elevation_angle = elevation_min;
    float azimuth_angle = azimuth_min;
    float x, y, z;
    x = d;
    for (size_t i = 0; i < num_points_elevation; i++) {
        z = d * tan(elevation_angle * deg_to_rad);
        for (size_t j = 0; j < num_points_azimuth; j++) {
            y = d * tan(azimuth_angle * deg_to_rad);
            // save point
            rayBatch.push_back(std::pair<Eigen::Isometry3f, Eigen::Vector3f>(
                Eigen::Isometry3f::Identity(), Eigen::Vector3f(x, y, z)));
            // increase azimuth angle
            azimuth_angle += azimuth_res;
        }
        azimuth_angle = azimuth_min;
        //increase elevation angle
        elevation_angle += elevation_res;
    }


    // ========= Map INITIALIZATION  =========
    const float res = 0.05f;
    const float dim = 25.6f;
    se::OccupancyMap<se::Res::Multi> map(Eigen::Vector3f::Constant(dim), res);


    // ========= Sensor INITIALIZATION  =========
    se::LeicaLidar::Config sensorConfig;
    sensorConfig.width = 360;
    sensorConfig.height = 180;
    sensorConfig.near_plane = 0.6f;
    sensorConfig.far_plane = 30.0f;
    sensorConfig.T_BS = Eigen::Isometry3f::Identity();
    sensorConfig.elevation_resolution_angle_ = static_cast<float>(elevation_res);
    sensorConfig.azimuth_resolution_angle_ = static_cast<float>(azimuth_res);

    //se::LeicaLidar::Config sensorConfig(se_config.sensor);
    const se::LeicaLidar sensor(sensorConfig);

    // ========= Integrator INITIALIZATION  =========
    se::MapIntegrator integrator(map);

    // ========= Integration (Batched)
    std::vector<const se::OctantBase*> updated_octants;
    integrator.integrateRayBatch(sensor, rayBatch, 0, updated_octants);
    std::cout << "Number of updated octants: " << updated_octants.size() << std::endl;
    // Un-Comment if needed for debugging
    map.saveStructure(tmp_ + "/batch_ray_structure.ply");
    map.saveMeshVoxel(tmp_ + "/batch_ray_mesh.ply");

    std::queue<const NodeType*> nodes;
    std::queue<const BlockType*> blocks;
    const NodeType* const root = static_cast<const NodeType*>(map.getOctree().getRoot());
    ASSERT_TRUE(root);
    nodes.push(root);

    // Traverse the octree breadth-first from the root and test propagation at the node level.
    int counter = 0;
    while (!nodes.empty()) {
        counter++;
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
    std::cout << "Checked " << counter << " nodes." << std::endl;

    counter = 0;
    // Test propagation within blocks.
    while (!blocks.empty()) {
        counter++;
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
    std::cout << "Checked " << counter << " blocks." << std::endl;
}
