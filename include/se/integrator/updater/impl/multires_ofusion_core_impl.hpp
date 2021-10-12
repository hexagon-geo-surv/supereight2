/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_OFUSION_CORE_IMPL_HPP
#define SE_MULTIRES_OFUSION_CORE_IMPL_HPP



namespace se {



template<typename ConfigT>
inline float compute_three_sigma(const se::field_t depth_value,
                                 const float sigma_min,
                                 const float sigma_max,
                                 const ConfigT config)
{
    if (config.uncertainty_model == se::UncertaintyModel::Linear) {
        return 3
            * se::math::clamp(
                   config.k_sigma * depth_value, sigma_min, sigma_max); // Livingroom dataset
    }
    else {
        return 3
            * se::math::clamp(config.k_sigma * se::math::sq(depth_value),
                              sigma_min,
                              sigma_max); // Cow and lady
    }
}



template<typename ConfigT>
inline float compute_tau(const field_t depth_value,
                         const float tau_min,
                         const float tau_max,
                         const ConfigT config)
{
    if (config.const_surface_thickness) {
        return tau_max; ///<< e.g. used in ICL-NUIM livingroom dataset.
    }
    else {
        return se::math::clamp(config.k_tau * depth_value, tau_min, tau_max);
    }
}



namespace updater {



template<typename DataT>
inline bool weightedMeanUpdate(DataT& data, const float sample_value, const se::weight_t max_weight)
{
    data.occupancy = (data.occupancy * data.weight + sample_value) / (data.weight + 1);
    data.weight = std::min((data.weight + 1), max_weight);
    if (data.observed) {
        return false;
    }
    else {
        data.observed = true;
        return true;
    }
}



template<typename DataT, typename ConfigT>
inline bool updateVoxel(DataT& data,
                        const float range_diff,
                        const float tau,
                        const float three_sigma,
                        const ConfigT config)
{
    float sample_value;

    if (range_diff < -three_sigma) {
        sample_value = config.log_odd_min;
    }
    else if (range_diff < tau / 2) {
        sample_value = std::min(config.log_odd_min
                                    - config.log_odd_min / three_sigma * (range_diff + three_sigma),
                                config.log_odd_max);
    }
    else if (range_diff < tau) {
        sample_value = std::min(-config.log_odd_min * tau / (2 * three_sigma), config.log_odd_max);
    }
    else {
        return false;
    }

    return weightedMeanUpdate(data, sample_value, config.max_weight);
}



template<typename DataT, typename ConfigT>
inline void freeNode(DataT& node_data, const ConfigT config)
{
    weightedMeanUpdate(node_data, config.log_odd_min, config.max_weight);
}



template<typename DataT, typename ConfigT>
inline bool freeVoxel(DataT& voxel_data, const ConfigT config)
{
    return weightedMeanUpdate(voxel_data, config.log_odd_min, config.max_weight);
}



template<typename NodeT, typename BlockT>
inline typename NodeT::DataType
propagateToNoteAtCoarserScale(se::OctantBase* octant_ptr,
                              const unsigned int /* voxel_depth */, // TODO:
                              const unsigned int frame)
{
    NodeT* node_ptr = static_cast<NodeT*>(octant_ptr);

    node_ptr->setTimeStamp(frame);

    float max_mean_occupancy = 0;
    se::weight_t max_weight = 0;
    float max_occupancy = -std::numeric_limits<float>::max();
    unsigned int observed_count = 0;
    unsigned int data_count = 0;

    for (unsigned int child_idx = 0; child_idx < 8; ++child_idx) {
        se::OctantBase* child_ptr = node_ptr->getChild(child_idx);

        if (!child_ptr) {
            continue;
        }

        const auto& child_data = (child_ptr->isBlock())
            ? static_cast<BlockT*>(child_ptr)->getMaxData()
            : static_cast<NodeT*>(child_ptr)->getMaxData();
        if (child_data.weight > 0
            && child_data.occupancy * child_data.weight > max_occupancy) // At least 1 integration
        {
            data_count++;
            max_mean_occupancy = child_data.occupancy;
            max_weight = child_data.weight;
            max_occupancy = max_mean_occupancy * max_weight;
        }
        if (child_data.observed == true) {
            observed_count++;
        }
    }

    typename NodeT::DataType node_data = node_ptr->getData();

    if (data_count > 0) {
        node_data.occupancy = max_mean_occupancy; // TODO: Need to check update?
        node_data.weight = max_weight;
        if (observed_count == 8) {
            node_data.observed = true;
        }
        node_ptr->setData(node_data);
    }
    return node_data;
}



template<typename BlockT>
inline void propagateBlockToCoarsestScale(se::OctantBase* octant_ptr)
{
    typedef typename BlockT::DataType DataType;

    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);
    int target_scale = block_ptr->getCurrentScale() + 1;
    unsigned int size_at_target_scale_li = BlockT::size >> target_scale;
    unsigned int size_at_target_scale_sq = se::math::sq(size_at_target_scale_li);

    int child_scale = target_scale - 1;
    unsigned int size_at_child_scale_li = BlockT::size >> child_scale;
    unsigned int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

    DataType min_data;
    se::field_t o_min;

    if (block_ptr->buffer_scale() > block_ptr->getCurrentScale()) {
        DataType* max_data_at_target_scale = block_ptr->blockMaxDataAtScale(target_scale);
        DataType* max_data_at_child_scale = block_ptr->blockDataAtScale(child_scale);

        min_data = max_data_at_child_scale[0];
        o_min = min_data.occupancy * min_data.weight;

        for (unsigned int z = 0; z < size_at_target_scale_li; z++) {
            for (unsigned int y = 0; y < size_at_target_scale_li; y++) {
                for (unsigned int x = 0; x < size_at_target_scale_li; x++) {
                    const int target_max_data_idx =
                        x + y * size_at_target_scale_li + z * size_at_target_scale_sq;
                    auto& target_max_data = max_data_at_target_scale[target_max_data_idx];

                    float max_mean_occupancy = 0;
                    se::weight_t max_weight = 0;
                    float max_occupancy = -std::numeric_limits<float>::max();

                    int observed_count = 0;
                    int data_count = 0;

                    for (unsigned int k = 0; k < 2; k++) {
                        for (unsigned int j = 0; j < 2; j++) {
                            for (unsigned int i = 0; i < 2; i++) {
                                const int child_max_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto child_data = max_data_at_child_scale[child_max_data_idx];

                                float o = (child_data.occupancy * child_data.weight);

                                if (child_data.weight > 0) {
                                    if (o > max_occupancy) {
                                        data_count++;
                                        // Update max
                                        max_mean_occupancy = child_data.occupancy;
                                        max_weight = child_data.weight;
                                        max_occupancy = max_mean_occupancy * max_weight;
                                    }
                                    else if (o < o_min) {
                                        min_data.occupancy = child_data.occupancy;
                                        min_data.weight = child_data.weight;
                                        o_min = o;
                                    }
                                }

                                if (child_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        target_max_data.occupancy = max_mean_occupancy;
                        target_max_data.weight = max_weight;
                        if (observed_count == 8) {
                            target_max_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                        }
                    }

                } // x
            }     // y
        }         // z
    }
    else {
        DataType* max_data_at_target_scale = block_ptr->blockMaxDataAtScale(target_scale);
        DataType* data_at_target_scale = block_ptr->blockDataAtScale(target_scale);
        DataType* data_at_child_scale = block_ptr->blockDataAtScale(child_scale);

        min_data = data_at_child_scale[0];
        o_min = min_data.occupancy * min_data.weight;

        for (unsigned int z = 0; z < size_at_target_scale_li; z++) {
            for (unsigned int y = 0; y < size_at_target_scale_li; y++) {
                for (unsigned int x = 0; x < size_at_target_scale_li; x++) {
                    const int target_data_idx =
                        x + y * size_at_target_scale_li + z * size_at_target_scale_sq;
                    auto& target_data = data_at_target_scale[target_data_idx];
                    auto& target_max_data = max_data_at_target_scale[target_data_idx];

                    se::field_t mean_occupancy = 0;
                    se::weight_t mean_weight = 0;

                    se::field_t max_mean_occupancy = 0;
                    se::weight_t max_weight = 0;
                    se::field_t max_occupancy = -std::numeric_limits<se::field_t>::max();

                    int observed_count = 0;
                    int data_count = 0;

                    for (unsigned int k = 0; k < 2; k++) {
                        for (unsigned int j = 0; j < 2; j++) {
                            for (unsigned int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto child_data = data_at_child_scale[child_data_idx];

                                if (child_data.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.occupancy;
                                    mean_weight += child_data.weight;

                                    float o = (child_data.occupancy * child_data.weight);

                                    if (o > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_data.occupancy;
                                        max_weight = child_data.weight;
                                        max_occupancy = max_mean_occupancy * max_weight;
                                    }
                                    else if (o > o_min) {
                                        min_data.occupancy = child_data.occupancy;
                                        min_data.weight = child_data.weight;
                                        o_min = o;
                                    }
                                }

                                if (child_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        target_data.occupancy = mean_occupancy / data_count;
                        target_data.weight = ceil((float) mean_weight) / data_count;
                        target_data.observed = false;

                        target_max_data.occupancy = max_mean_occupancy;
                        target_max_data.weight = max_weight;
                        if (observed_count == 8) {
                            target_max_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                        }
                    }

                } // x
            }     // y
        }         // z
    }



    for (target_scale += 1; target_scale <= BlockT::getMaxScale(); ++target_scale) {
        unsigned int size_at_target_scale_li = BlockT::size >> target_scale;
        unsigned int size_at_target_scale_sq = se::math::sq(size_at_target_scale_li);

        int child_scale = target_scale - 1;
        unsigned int size_at_child_scale_li = BlockT::size >> child_scale;
        unsigned int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

        DataType* max_data_at_target_scale = block_ptr->blockMaxDataAtScale(target_scale);
        DataType* data_at_target_scale = block_ptr->blockDataAtScale(target_scale);
        DataType* max_data_at_child_scale = block_ptr->blockMaxDataAtScale(child_scale);
        DataType* data_at_child_scale = block_ptr->blockDataAtScale(child_scale);

        for (unsigned int z = 0; z < size_at_target_scale_li; z++) {
            for (unsigned int y = 0; y < size_at_target_scale_li; y++) {
                for (unsigned int x = 0; x < size_at_target_scale_li; x++) {
                    const int target_data_idx =
                        x + y * size_at_target_scale_li + z * size_at_target_scale_sq;
                    auto& target_data = data_at_target_scale[target_data_idx];
                    auto& target_max_data = max_data_at_target_scale[target_data_idx];

                    se::field_t mean_occupancy = 0;
                    se::weight_t mean_weight = 0;

                    se::field_t max_mean_occupancy = 0;
                    se::weight_t max_weight = 0;
                    se::field_t max_occupancy = -std::numeric_limits<float>::max();

                    int observed_count = 0;
                    int data_count = 0;

                    for (unsigned int k = 0; k < 2; k++) {
                        for (unsigned int j = 0; j < 2; j++) {
                            for (unsigned int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto child_data = data_at_child_scale[child_data_idx];
                                const auto child_max_data = max_data_at_child_scale[child_data_idx];

                                if (child_max_data.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.occupancy;
                                    mean_weight += child_data.weight;

                                    if ((child_max_data.occupancy * child_max_data.weight)
                                        > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_max_data.occupancy;
                                        max_weight = child_max_data.weight;
                                        max_occupancy = max_mean_occupancy * max_weight;
                                    }
                                }

                                if (child_max_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        target_data.occupancy = mean_occupancy / data_count;
                        target_data.weight = ceil((float) mean_weight) / data_count;
                        target_data.observed = false;

                        target_max_data.occupancy = max_mean_occupancy;
                        target_max_data.weight = max_weight;
                        if (observed_count == 8) {
                            target_max_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                        }
                    }

                } // x
            }     // y
        }         // z
    }

    block_ptr->setMinData(min_data);
}



template<typename BlockT>
inline void maxCoarsePropagation(const se::OctantBase* octant_ptr,
                                 const Eigen::Vector3i target_coord,
                                 const int target_scale,
                                 const unsigned int target_stride,
                                 typename BlockT::DataType& target_data)
{
    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

    se::field_t max_mean_occupancy = -std::numeric_limits<se::field_t>::max();
    se::weight_t max_weight = 0;
    se::field_t max_occupancy = -std::numeric_limits<se::field_t>::max();
    unsigned int observed_count = 0;
    unsigned int data_count = 0;

    int child_scale = target_scale - 1;
    unsigned int child_stride = target_stride >> 1; ///<< Halfen target stride

    for (unsigned int k = 0; k < target_stride; k += child_stride) {
        for (unsigned int j = 0; j < target_stride; j += child_stride) {
            for (unsigned int i = 0; i < target_stride; i += child_stride) {
                const auto child_data =
                    block_ptr->data(target_coord + Eigen::Vector3i(i, j, k), child_scale);
                /// Only compare partly observed children (child_data.weight > 0)
                /// max_mean_occupancy is the product of data.occupancy * data.weight (i.e. not the mean log-odd occupancy)
                if (child_data.weight > 0
                    && ((child_data.occupancy * child_data.weight) > max_occupancy)) {
                    data_count++;
                    max_mean_occupancy = child_data.occupancy;
                    max_weight = child_data.weight;
                    max_occupancy = max_mean_occupancy * max_weight;
                }

                if (child_data.observed) {
                    observed_count++;
                }

            } // i
        }     // j
    }         // k

    if (data_count > 0) {
        target_data.occupancy = max_mean_occupancy;
        target_data.weight = max_weight;
        if (observed_count
            == 8) { ///<< If all children have been observed, set parent/target to observed.
            target_data.observed = true;
        }
    }
}



template<typename BlockT>
inline void meanCoarsePropagation(const se::OctantBase* octant_ptr,
                                  const Eigen::Vector3i target_coord,
                                  const int target_scale,
                                  const unsigned int target_stride,
                                  typename BlockT::DataType& target_data)
{
    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

    se::field_t mean_occupancy = 0;
    se::weight_t mean_weight = 0;
    unsigned int observed_count = 0;
    unsigned int data_count = 0;

    int child_scale = target_scale - 1;
    unsigned int child_stride = target_stride >> 1;

    for (unsigned int k = 0; k < target_stride; k += child_stride) {
        for (unsigned int j = 0; j < target_stride; j += child_stride) {
            for (unsigned int i = 0; i < target_stride; i += child_stride) {
                auto child_data =
                    block_ptr->data(target_coord + Eigen::Vector3i(i, j, k), child_scale);
                if (child_data.weight > 0) {
                    data_count++;
                    mean_occupancy += child_data.occupancy;
                    mean_weight += child_data.weight;
                }
                if (child_data.observed) {
                    observed_count++;
                }
            }
        }
    }

    if (data_count == 8) {
        target_data.occupancy = mean_occupancy / data_count;
        target_data.weight = ((float) mean_weight) / data_count;
        target_data.observed = true;
    }
}



template<typename BlockT>
inline void propagateToVoxelAtCoarserScale(const se::OctantBase* octant_ptr,
                                           const Eigen::Vector3i voxel_coord,
                                           const int target_scale,
                                           const unsigned int target_stride,
                                           typename BlockT::DataType& voxel_data)
{
    BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);
    maxCoarsePropagation(block_ptr, voxel_coord, target_scale, target_stride, voxel_data);
}



} // namespace updater
} // namespace se

#endif // SE_MULTIRES_OFUSION_CORE_IMPL_HPP
