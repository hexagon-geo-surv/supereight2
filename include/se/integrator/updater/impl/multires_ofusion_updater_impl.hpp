/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_OFUSION_UPDATER_IMPL_HPP
#define SE_MULTIRES_OFUSION_UPDATER_IMPL_HPP



namespace se {



// Multi-res Occupancy updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::Updater(
    MapType& map,
    const SensorT& sensor,
    const Image<float>& depth_img,
    const Image<rgb_t>* colour_img,
    const Eigen::Matrix4f& T_WS,
    const int frame) :
        map_(map),
        octree_(*(map.getOctree())),
        sensor_(sensor),
        depth_img_(depth_img),
        T_SW_(math::to_inverse_transformation(T_WS)),
        frame_(frame),
        map_res_(map.getRes()),
        config_(map),
        node_set_(octree_.getBlockDepth())
{
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::operator()(
    VolumeCarverAllocation& allocation_list)
{
    TICK("fusion-total")

    TICK("fusion-nodes")
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.node_list.size(); ++i) {
        auto node_ptr = static_cast<NodeType*>(allocation_list.node_list[i]);
        const int depth = octree_.getMaxScale() - std::log2(node_ptr->getSize());
        updateNodeFree(allocation_list.node_list[i], depth);
    }
    TOCK("fusion-nodes")

    TICK("fusion-blocks")
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i) {
        updateBlock(allocation_list.block_list[i],
                    allocation_list.variance_state_list[i] == VarianceState::Constant,
                    allocation_list.projects_inside_list[i]);
    }
    TOCK("fusion-blocks")

    TOCK("fusion-total")

    /// Propagation
    TICK("propagation-total")

    TICK("propagation-blocks")
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i) {
        updater::propagate_block_to_coarsest_scale<BlockType>(allocation_list.block_list[i]);
    }
#pragma omp parallel for
    for (unsigned int i = 0; i < freed_block_list_.size(); ++i) {
        updater::propagate_block_to_coarsest_scale<BlockType>(freed_block_list_[i]);
    }
    TOCK("propagation-blocks")

    TICK("propagation-to-root")
    propagateToRoot(allocation_list.block_list);
    TOCK("propagation-to-root")

    TOCK("propagation-total")
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>,
             SensorT>::propagateToRoot(std::vector<OctantBase*>& block_list)
{
    for (auto octant_ptr : block_list) {
        BlockType& block = *static_cast<BlockType*>(octant_ptr);
        if (block.getParent()) {
            node_set_[octree_.getBlockDepth() - 1].insert(block.getParent());
        }
    }

    const auto occupancy_threshold = 0.95 * MapType::DataType::min_occupancy;
    for (int d = octree_.getBlockDepth() - 1; d > 0; d--) { // TODO: block depth - 1?
        for (auto octant_ptr : node_set_[d]) {
            if (octant_ptr->getTimeStamp() == frame_) {
                continue;
            }
            if (octant_ptr->getParent()) {
                auto node_data =
                    updater::propagate_to_parent_node<NodeType, BlockType>(octant_ptr, frame_);
                node_set_[d - 1].insert(octant_ptr->getParent());

                if (node_data.observed
                    && node_data.occupancy * node_data.weight <= occupancy_threshold) {
                    octree_.deleteChildren(static_cast<NodeType*>(octant_ptr));
                }

            } // if parent
        }     // nodes at depth d
    }         // depth d

    updater::propagate_to_parent_node<NodeType, BlockType>(octree_.getRoot(), frame_);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>,
             SensorT>::updateBlockFree(OctantBase* octant_ptr)
{
    BlockType& block = *static_cast<BlockType*>(octant_ptr);
    const auto& config = map_.getDataConfig();

    // Compute the point of the block centre in the sensor frame
    constexpr int block_size = BlockType::size;
    const Eigen::Vector3i block_coord = block.getCoord();
    Eigen::Vector3f block_centre_point_W;
    map_.voxelToPoint(block_coord, block_size, block_centre_point_W);
    const Eigen::Vector3f block_centre_point_C =
        (T_SW_ * (block_centre_point_W).homogeneous()).head<3>();

    /// Compute the integration scale
    // The last integration scale
    const int last_scale = (block.getMinScale() == -1) ? 0 : block.getCurrentScale();

    // The recommended integration scale
    const int computed_integration_scale = sensor_.computeIntegrationScale(
        block_centre_point_C, map_res_, last_scale, block.getMinScale(), block.getMaxScale());

    // The minimum integration scale (change to last if data has already been integrated)
    const int min_integration_scale =
        ((block.getMinScale() == -1 || block.maxValue() < 0.95 * config.log_odd_min))
        ? config.fs_integr_scale
        : std::max(0, last_scale - 1);
    const int max_integration_scale = (block.getMinScale() == -1)
        ? BlockType::getMaxScale()
        : std::min(BlockType::getMaxScale(), last_scale + 1);

    // The final integration scale
    const int recommended_scale = std::min(
        std::max(min_integration_scale, computed_integration_scale), max_integration_scale);

    int integration_scale = last_scale;

    /// If no data has been integrated in the block before (block.getMinScale() == -1), use the computed integration scale.
    if (block.getMinScale() == -1) {
        // Make sure the block is allocated up to the integration scale
        integration_scale = recommended_scale;
        block.allocateDownTo(integration_scale);
        block.setCurrentScale(integration_scale);
        block.initCurrCout();
        block.setInitData(DataType());
    }
    else if (recommended_scale != last_scale) ///<< Potential double integration
    {
        if (recommended_scale
            != block.buffer_scale()) ///<< Start from scratch and initialise buffer
        {
            block.initBuffer(recommended_scale);

            if (recommended_scale < last_scale) {
                const int parent_scale = last_scale;
                const unsigned int size_at_parent_scale_li = block_size >> parent_scale;
                const unsigned int size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

                const unsigned int size_at_buffer_scale_li = size_at_parent_scale_li << 1;
                const unsigned int size_at_buffer_scale_sq = math::sq(size_at_buffer_scale_li);

                for (unsigned int z = 0; z < size_at_parent_scale_li; z++) {
                    for (unsigned int y = 0; y < size_at_parent_scale_li; y++) {
#pragma omp simd // TODO: MOVE UP
                        for (unsigned int x = 0; x < size_at_parent_scale_li; x++) {
                            const int parent_idx =
                                x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                            const auto& parent_data =
                                block.currData(parent_idx); // TODO: CAN BE MADE FASTER

                            for (unsigned int k = 0; k < 2; k++) {
                                for (unsigned int j = 0; j < 2; j++) {
                                    for (unsigned int i = 0; i < 2; i++) {
                                        const int buffer_idx = (2 * x + i)
                                            + (2 * y + j) * size_at_buffer_scale_li
                                            + (2 * z + k) * size_at_buffer_scale_sq;
                                        auto& buffer_data = block.bufferData(
                                            buffer_idx); ///<< Fetch value from buffer.

                                        buffer_data.occupancy = parent_data.occupancy;
                                        buffer_data.weight =
                                            parent_data.weight; // (parent_data.y > 0) ? 1 : 0;
                                        buffer_data.observed =
                                            false; ///<< Set falls such that the observe count can work properly

                                    } // i
                                }     // j
                            }         // k

                        } // x
                    }     // y
                }         // z
            }
        }

        /// Integrate data into buffer.
        const unsigned int size_at_recommended_scale_li = BlockType::size >> recommended_scale;
        const unsigned int size_at_recommended_scale_sq = math::sq(size_at_recommended_scale_li);

        for (unsigned int z = 0; z < size_at_recommended_scale_li; z++) {
            for (unsigned int y = 0; y < size_at_recommended_scale_li; y++) {
#pragma omp simd
                for (unsigned int x = 0; x < size_at_recommended_scale_li; x++) {
                    const int buffer_idx =
                        x + y * size_at_recommended_scale_li + z * size_at_recommended_scale_sq;
                    auto& buffer_data =
                        block.bufferData(buffer_idx); /// \note pass by reference now.
                    block.incrBufferObservedCount(updater::update_voxel_free(buffer_data, config));
                } // x
            }     // y
        }         // z

        block.incrBufferIntegrCount();

        if (block.switchData()) {
            return;
        }
    }
    else {
        block.resetBuffer();
    }

    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    const unsigned int size_at_integration_scale_sq = math::sq(size_at_integration_scale_li);

    for (unsigned int z = 0; z < size_at_integration_scale_li; z++) {
        for (unsigned int y = 0; y < size_at_integration_scale_li; y++) {
#pragma omp simd // TODO: Move UP
            for (unsigned int x = 0; x < size_at_integration_scale_li; x++) {
                // Update the voxel data based using the depth measurement
                const int voxel_idx =
                    x + y * size_at_integration_scale_li + z * size_at_integration_scale_sq;
                auto& voxel_data = block.currData(voxel_idx); /// \note pass by reference now.
                block.incrCurrObservedCount(updater::update_voxel_free(voxel_data, config));
            } // x
        }     // y
    }         // z

    block.incrCurrIntegrCount();
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::updateBlock(
    OctantBase* octant_ptr,
    bool low_variance,
    bool project_inside)
{
    BlockType& block = *static_cast<BlockType*>(octant_ptr);
    const auto& config = map_.getDataConfig();

    // Compute the point of the block centre in the sensor frame
    constexpr int block_size = BlockType::size;
    const Eigen::Vector3i block_coord = block.getCoord();
    Eigen::Vector3f block_centre_point_W;
    map_.voxelToPoint(block_coord, block_size, block_centre_point_W);
    const Eigen::Vector3f block_centre_point_S =
        (T_SW_ * (block_centre_point_W).homogeneous()).head<3>();

    // Convert block centre to measurement >> PinholeCamera -> .z() | OusterLidar -> .norm()
    const float block_point_C_m = sensor_.measurementFromPoint(block_centre_point_S);

    // Compute one tau and 3x sigma value for the block
    float tau = compute_tau(block_point_C_m, config_.tau_min, config_.tau_max, config);
    float three_sigma =
        compute_three_sigma(block_point_C_m, config_.sigma_min, config_.sigma_max, config);

    /// Compute the integration scale
    // The last integration scale
    const int last_scale = (block.getMinScale() == -1) ? 0 : block.getCurrentScale();

    // The recommended integration scale
    const int computed_integration_scale = sensor_.computeIntegrationScale(
        block_centre_point_S, map_res_, last_scale, block.getMinScale(), block.getMaxScale());

    // The minimum integration scale (change to last if data has already been integrated)
    const int min_integration_scale =
        (low_variance
         && (block.getMinScale() == -1 || block.maxValue() < 0.95 * config.log_odd_min))
        ? config.fs_integr_scale
        : std::max(0, last_scale - 1);
    const int max_integration_scale = (block.getMinScale() == -1)
        ? BlockType::getMaxScale()
        : std::min(BlockType::getMaxScale(), last_scale + 1);

    // The final integration scale
    const int recommended_scale = std::min(
        std::max(min_integration_scale, computed_integration_scale), max_integration_scale);

    int integration_scale = last_scale;

    /// If no data has been integrated in the block before (block.getMinScale() == -1), use the computed integration scale.
    if (block.getMinScale() == -1) {
        // Make sure the block is allocated up to the integration scale
        integration_scale = recommended_scale;
        block.allocateDownTo(integration_scale);
        block.setCurrentScale(integration_scale);
        block.initCurrCout();
        block.setInitData(DataType());
    }
    else if (recommended_scale != last_scale) ///<< Potential double integration
    {
        if (recommended_scale
            != block.buffer_scale()) ///<< Start from scratch and initialise buffer
        {
            block.initBuffer(recommended_scale);

            if (recommended_scale < last_scale) {
                const int parent_scale = last_scale;
                const unsigned int size_at_parent_scale_li = BlockType::size >> parent_scale;
                const unsigned int size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

                const unsigned int size_at_buffer_scale_li = size_at_parent_scale_li << 1;
                const unsigned int size_at_buffer_scale_sq = math::sq(size_at_buffer_scale_li);

                for (unsigned int z = 0; z < size_at_parent_scale_li; z++) {
                    for (unsigned int y = 0; y < size_at_parent_scale_li; y++) {
#pragma omp simd
                        for (unsigned int x = 0; x < size_at_parent_scale_li; x++) {
                            const int parent_idx =
                                x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                            const auto& parent_data =
                                block.currData(parent_idx); // TODO: CAN BE MADE FASTER

                            for (unsigned int k = 0; k < 2; k++) {
                                for (unsigned int j = 0; j < 2; j++) {
                                    for (unsigned int i = 0; i < 2; i++) {
                                        const int buffer_idx = (2 * x + i)
                                            + (2 * y + j) * size_at_buffer_scale_li
                                            + (2 * z + k) * size_at_buffer_scale_sq;
                                        auto& buffer_data = block.bufferData(
                                            buffer_idx); ///<< Fetch value from buffer.

                                        buffer_data.occupancy = parent_data.occupancy;
                                        buffer_data.weight =
                                            parent_data.weight; // (parent_data.y > 0) ? 1 : 0;
                                        buffer_data.observed =
                                            false; ///<< Set falls such that the observe count can work properly

                                    } // i
                                }     // j
                            }         // k

                        } // x
                    }     // y
                }         // z
            }
        }

        /// Integrate data into buffer.
        const unsigned int recommended_stride = 1 << recommended_scale;
        const unsigned int size_at_recommended_scale_li = BlockType::size >> recommended_scale;
        const unsigned int size_at_recommended_scale_sq = math::sq(size_at_recommended_scale_li);

        const Eigen::Vector3i voxel_coord_base = block.getCoord();
        Eigen::Vector3f sample_point_base_W;
        map_.voxelToPoint(voxel_coord_base, recommended_stride, sample_point_base_W);
        const Eigen::Vector3f sample_point_base_S =
            (T_SW_ * (sample_point_base_W).homogeneous()).head<3>();

        const Eigen::Matrix3f sample_point_delta_matrix_S =
            (math::to_rotation(T_SW_)
             * (map_res_
                * (Eigen::Matrix3f() << recommended_stride,
                   0,
                   0,
                   0,
                   recommended_stride,
                   0,
                   0,
                   0,
                   recommended_stride)
                      .finished()));

        for (unsigned int z = 0; z < size_at_recommended_scale_li; z++) {
            for (unsigned int y = 0; y < size_at_recommended_scale_li; y++) {
#pragma omp simd
                for (unsigned int x = 0; x < size_at_recommended_scale_li; x++) {
                    const Eigen::Vector3f sample_point_C = sample_point_base_S
                        + sample_point_delta_matrix_S * Eigen::Vector3f(x, y, z);

                    // Project sample point to the image plane.
                    Eigen::Vector2f pixel_f;
                    if (sensor_.model.project(sample_point_C, &pixel_f)
                        != srl::projection::ProjectionStatus::Successful) {
                        continue;
                    }
                    const Eigen::Vector2i pixel = se::round_pixel(pixel_f);
                    const int pixel_idx = pixel.x() + depth_img_.width() * pixel.y();

                    // Fetch image value
                    const float depth_value = depth_img_[pixel_idx];
                    if (depth_value < sensor_.near_plane) {
                        continue;
                    }

                    const int buffer_idx =
                        x + y * size_at_recommended_scale_li + z * size_at_recommended_scale_sq;
                    auto& buffer_data =
                        block.bufferData(buffer_idx); /// \note pass by reference now.

                    if (low_variance) {
                        block.incrBufferObservedCount(
                            updater::update_voxel_free(buffer_data, config));
                    }
                    else {
                        const float sample_point_C_m = sensor_.measurementFromPoint(sample_point_C);
                        const float range = sample_point_C.norm();
                        const float range_diff =
                            (sample_point_C_m - depth_value) * (range / sample_point_C_m);
                        if (range_diff >= tau) {
                            continue;
                        }
                        else if (range_diff < -three_sigma) {
                            block.incrBufferObservedCount(
                                updater::update_voxel_free(buffer_data, config));
                        }
                        else {
                            block.incrBufferObservedCount(updater::update_voxel(
                                buffer_data, range_diff, tau, three_sigma, config));
                        }
                    }
                } // x
            }     // y
        }         // z

        block.incrBufferIntegrCount(project_inside);

        if (block.switchData()) {
            return;
        }
    }
    else {
        block.resetBuffer();
    }

    const unsigned int integration_stride = 1 << integration_scale;
    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    const unsigned int size_at_integration_scale_sq = math::sq(size_at_integration_scale_li);

    const Eigen::Vector3i voxel_coord_base = block.getCoord();
    Eigen::Vector3f sample_point_base_W;
    map_.voxelToPoint(voxel_coord_base, integration_stride, sample_point_base_W);
    const Eigen::Vector3f sample_point_base_S =
        (T_SW_ * (sample_point_base_W).homogeneous()).head<3>();

    const Eigen::Matrix3f sample_point_delta_matrix_S =
        (math::to_rotation(T_SW_)
         * (map_res_
            * (Eigen::Matrix3f() << integration_stride,
               0,
               0,
               0,
               integration_stride,
               0,
               0,
               0,
               integration_stride)
                  .finished()));

    for (unsigned int z = 0; z < size_at_integration_scale_li; z++) {
        for (unsigned int y = 0; y < size_at_integration_scale_li; y++) {
#pragma omp simd
            for (unsigned int x = 0; x < size_at_integration_scale_li; x++) {
                const Eigen::Vector3f sample_point_C =
                    sample_point_base_S + sample_point_delta_matrix_S * Eigen::Vector3f(x, y, z);

                // Project sample point to the image plane.
                Eigen::Vector2f pixel_f;
                if (sensor_.model.project(sample_point_C, &pixel_f)
                    != srl::projection::ProjectionStatus::Successful) {
                    continue;
                }
                const Eigen::Vector2i pixel = se::round_pixel(pixel_f);
                const int pixel_idx = pixel.x() + depth_img_.width() * pixel.y();

                // Fetch image value
                const float depth_value = depth_img_[pixel_idx];
                if (depth_value < sensor_.near_plane) {
                    continue;
                }

                // Update the voxel data based using the depth measurement
                const int voxel_idx =
                    x + y * size_at_integration_scale_li + z * size_at_integration_scale_sq;
                auto& voxel_data = block.currData(voxel_idx); /// \note pass by reference now.
                if (low_variance) {
                    block.incrCurrObservedCount(updater::update_voxel_free(voxel_data, config));
                }
                else {
                    const float sample_point_C_m = sensor_.measurementFromPoint(sample_point_C);
                    const float range = sample_point_C.norm();
                    const float range_diff =
                        (sample_point_C_m - depth_value) * (range / sample_point_C_m);
                    if (range_diff >= tau) {
                        continue;
                    }
                    else if (range_diff < -three_sigma) {
                        block.incrCurrObservedCount(updater::update_voxel_free(voxel_data, config));
                    }
                    else {
                        block.incrCurrObservedCount(updater::update_voxel(
                            voxel_data, range_diff, tau, three_sigma, config));
                    }
                }
            } // x
        }     // y
    }         // z

    block.incrCurrIntegrCount();
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>,
             SensorT>::updateNodeFree(OctantBase* octant_ptr, int depth)
{
    NodeType& node = *static_cast<NodeType*>(octant_ptr);

    if (node.isLeaf()) {
        typename NodeType::DataType node_data = node.getData();
        updater::update_node_free(node_data, map_.getDataConfig());
        node.setData(node_data);
#pragma omp critical(node_lock)
        { // Add node to node list for later up-propagation (finest node for this tree-branch)
            node_set_[depth - 1].insert(node.getParent());
        }
    }
    else {
        for (int child_idx = 0; child_idx < 8; child_idx++) {
            OctantBase* child_ptr = node.getChild(child_idx);
            assert(child_ptr);
            if (child_ptr->isBlock()) {
                // Voxel block has a low variance. Update data at a minimum
                // free space integration scale or finer/coarser (depending on later scale selection).
                updateBlockFree(child_ptr); // TODO: Add to block_list?
#pragma omp critical(node_lock)
                { // Add node to node list for later up-propagation (finest node for this tree-branch)
                    node_set_[depth - 1].insert(child_ptr->getParent());
                }
#pragma omp critical(block_lock)
                { // Add node to node list for later up-propagation (finest node for this tree-branch)
                    freed_block_list_.push_back(child_ptr);
                }
            }
            else {
                updateNodeFree(child_ptr, depth + 1);
            }
        }
    }
}



} // namespace se

#endif // SE_MULTIRES_OFUSION_UPDATER_IMPL_HPP
