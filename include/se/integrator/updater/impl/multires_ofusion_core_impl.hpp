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
float compute_three_sigma(const field_t depth_value,
                          const float sigma_min,
                          const float sigma_max,
                          const ConfigT config)
{
    if (config.uncertainty_model == UncertaintyModel::Linear) {
        return 3
            * std::clamp(config.k_sigma * depth_value, sigma_min, sigma_max); // Livingroom dataset
    }
    else {
        return 3
            * std::clamp(config.k_sigma * math::sq(depth_value),
                         sigma_min,
                         sigma_max); // Cow and lady
    }
}



template<typename ConfigT>
float compute_tau(const field_t depth_value,
                  const float tau_min,
                  const float tau_max,
                  const ConfigT config)
{
    return std::clamp(config.k_tau * depth_value, tau_min, tau_max);
}



namespace updater {

template<typename DataT, typename ConfigT>
bool update_voxel(DataT& data,
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

    const bool newly_observed = !data.observed;
    data.update(sample_value, config.max_weight);
    return newly_observed;
}



template<typename DataT, typename ConfigT>
bool free_voxel(DataT& voxel_data, const ConfigT config)
{
    const bool newly_observed = !voxel_data.observed;
    voxel_data.update(config.log_odd_min, config.max_weight);
    return newly_observed;
}



template<typename NodeT, typename BlockT>
typename NodeT::DataType propagate_to_parent_node(OctantBase* octant_ptr, const int frame)
{
    assert(octant_ptr);
    assert(!octant_ptr->isBlock());

    NodeT& node = *static_cast<NodeT*>(octant_ptr);
    node.setTimeStamp(frame);

    // Compute the maximum data among all children.
    field_t max_occupancy = std::numeric_limits<field_t>::lowest();
    field_t max_mean_occupancy = 0;
    weight_t max_weight = 0;
    int max_data_count = 0;

    field_t min_mean_occupancy = 0;
    weight_t min_weight = 0;
    field_t min_occupancy = std::numeric_limits<field_t>::max();
    int min_data_count = 0;

    int observed_count = 0;
    for (int child_idx = 0; child_idx < 8; ++child_idx) {
        const OctantBase* child_ptr = node.getChild(child_idx);
        if (!child_ptr) {
            continue;
        }

        const auto& child_min_data = (child_ptr->isBlock())
            ? static_cast<const BlockT*>(child_ptr)->getMinData()
            : static_cast<const NodeT*>(child_ptr)->getMinData();
        // Only consider children with at least 1 integration.
        if (child_min_data.weight > 0) {
            const field_t occupancy = get_field(child_min_data);
            if (occupancy < min_occupancy) {
                min_mean_occupancy = child_min_data.occupancy;
                min_weight = child_min_data.weight;
                min_occupancy = occupancy;
                min_data_count++;
            }
        }

        const auto& child_max_data = child_ptr->isBlock()
            ? static_cast<const BlockT*>(child_ptr)->getMaxData()
            : static_cast<const NodeT*>(child_ptr)->getMaxData();
        // Only consider children with at least 1 integration.
        if (child_max_data.weight > 0) {
            const field_t occupancy = get_field(child_max_data);
            if (occupancy > max_occupancy) {
                max_mean_occupancy = child_max_data.occupancy;
                max_weight = child_max_data.weight;
                max_occupancy = occupancy;
                max_data_count++;
            }
        }

        assert(child_min_data.observed == child_max_data.observed);
        if (child_max_data.observed == true) {
            observed_count++;
        }
    }

    if (min_data_count > 0) {
        typename NodeT::DataType node_min_data = node.getMinData();
        node_min_data.occupancy = min_mean_occupancy; // TODO: Need to check update?
        node_min_data.weight = min_weight;
        if (observed_count == 8) {
            node_min_data.observed = true;
        }
        node.setMinData(node_min_data);
    }

    typename NodeT::DataType node_data = node.getData();
    if (max_data_count > 0) {
        node_data.occupancy = max_mean_occupancy; // TODO: Need to check update?
        node_data.weight = max_weight;
        if (observed_count == 8) {
            node_data.observed = true;
        }
        node.setData(node_data);
    }
    return node_data;
}



template<typename BlockT>
void propagate_block_to_coarsest_scale(OctantBase* octant_ptr)
{
    assert(octant_ptr);
    assert(octant_ptr->isBlock());

    typedef typename BlockT::DataType DataType;

    BlockT& block = *static_cast<BlockT*>(octant_ptr);
    if (block.getCurrentScale() == block.getMaxScale()) {
        return;
    }

    int child_scale = block.getCurrentScale();
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    int size_at_parent_scale_li = BlockT::size >> parent_scale;
    int size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

    if (block.buffer_scale() > block.getCurrentScale()) {
        DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
        DataType* min_data_at_child_scale = block.blockDataAtScale(child_scale);
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* max_data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_max_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_min_data = min_data_at_parent_scale[parent_max_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_max_data_idx];

                    field_t min_mean_occupancy = 0;
                    weight_t min_weight = 0;
                    field_t min_occupancy = std::numeric_limits<float>::max();
                    int min_data_count = 0;

                    field_t max_mean_occupancy = 0;
                    weight_t max_weight = 0;
                    field_t max_occupancy = -std::numeric_limits<float>::max();
                    int max_data_count = 0;

                    int observed_count = 0;

                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto& child_min_data =
                                    min_data_at_child_scale[child_data_idx];
                                const auto& child_max_data =
                                    max_data_at_child_scale[child_data_idx];

                                if (child_max_data.weight > 0) {
                                    const field_t occupancy = get_field(child_max_data);
                                    if (occupancy > max_occupancy) {
                                        max_data_count++;
                                        // Update max
                                        max_mean_occupancy = child_max_data.occupancy;
                                        max_weight = child_max_data.weight;
                                        max_occupancy = occupancy;
                                    }
                                }
                                if (child_min_data.weight > 0) {
                                    const field_t occupancy = get_field(child_min_data);
                                    if (occupancy < min_occupancy) {
                                        min_data_count++;
                                        min_mean_occupancy = child_min_data.occupancy;
                                        min_weight = child_min_data.weight;
                                        min_occupancy = occupancy;
                                    }
                                }

                                if (child_max_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (min_data_count > 0) {
                        parent_min_data.occupancy = min_mean_occupancy;
                        parent_min_data.weight = min_weight;
                        if (observed_count == 8) {
                            parent_min_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                        }
                    }

                    if (max_data_count > 0) {
                        parent_max_data.occupancy = max_mean_occupancy;
                        parent_max_data.weight = max_weight;
                        if (observed_count == 8) {
                            parent_max_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                        }
                    }

                } // x
            }     // y
        }         // z
    }
    else {
        DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_min_data = min_data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    field_t mean_occupancy = 0;
                    weight_t mean_weight = 0;

                    field_t min_mean_occupancy = 0;
                    weight_t min_weight = 0;
                    field_t min_occupancy = std::numeric_limits<field_t>::max();

                    field_t max_mean_occupancy = 0;
                    weight_t max_weight = 0;
                    field_t max_occupancy = -std::numeric_limits<field_t>::max();

                    size_t observed_count = 0;
                    size_t data_count = 0;

                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto& child_data = data_at_child_scale[child_data_idx];

                                if (child_data.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.occupancy;
                                    mean_weight += child_data.weight;

                                    const field_t occupancy = get_field(child_data);
                                    if (occupancy > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_data.occupancy;
                                        max_weight = child_data.weight;
                                        max_occupancy = occupancy;
                                    }
                                    if (occupancy < min_occupancy) {
                                        min_mean_occupancy = child_data.occupancy;
                                        min_weight = child_data.weight;
                                        min_occupancy = occupancy;
                                    }
                                }

                                if (child_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        parent_data.occupancy = mean_occupancy / data_count;
                        parent_data.weight = ceil((float) mean_weight) / data_count;
                        parent_data.observed = false;

                        parent_min_data.occupancy = min_mean_occupancy;
                        parent_min_data.weight = min_weight;
                        parent_max_data.occupancy = max_mean_occupancy;
                        parent_max_data.weight = max_weight;
                        if (observed_count == 8) {
                            parent_max_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                            parent_min_data.observed = true;
                        }
                    }

                } // x
            }     // y
        }         // z
    }

    for (parent_scale += 1; parent_scale <= BlockT::getMaxScale(); ++parent_scale) {
        size_at_parent_scale_li = BlockT::size >> parent_scale;
        size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

        child_scale = parent_scale - 1;
        size_at_child_scale_li = BlockT::size >> child_scale;
        size_at_child_scale_sq = math::sq(size_at_child_scale_li);

        DataType* min_data_at_parent_scale = block.blockMinDataAtScale(parent_scale);
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* min_data_at_child_scale = block.blockMinDataAtScale(child_scale);
        DataType* max_data_at_child_scale = block.blockMaxDataAtScale(child_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_min_data = min_data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    field_t mean_occupancy = 0;
                    weight_t mean_weight = 0;

                    field_t min_mean_occupancy = 0;
                    weight_t min_weight = 0;
                    field_t min_occupancy = std::numeric_limits<float>::max();

                    field_t max_mean_occupancy = 0;
                    weight_t max_weight = 0;
                    field_t max_occupancy = -std::numeric_limits<float>::max();

                    size_t observed_count = 0;
                    size_t data_count = 0;

                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto& child_data = data_at_child_scale[child_data_idx];
                                const auto& child_min_data =
                                    min_data_at_child_scale[child_data_idx];
                                const auto& child_max_data =
                                    max_data_at_child_scale[child_data_idx];

                                if (child_max_data.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.occupancy;
                                    mean_weight += child_data.weight;

                                    const field_t child_max_occupancy = get_field(child_max_data);
                                    if (child_max_occupancy > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_max_data.occupancy;
                                        max_weight = child_max_data.weight;
                                        max_occupancy = child_max_occupancy;
                                    }

                                    const field_t child_min_occupancy = get_field(child_min_data);
                                    if (child_min_occupancy < min_occupancy) {
                                        // Update min
                                        min_mean_occupancy = child_min_data.occupancy;
                                        min_weight = child_min_data.weight;
                                        min_occupancy = child_min_occupancy;
                                    }
                                }

                                if (child_max_data.observed) {
                                    observed_count++;
                                }

                            } // i
                        }     // j
                    }         // k

                    if (data_count > 0) {
                        parent_data.occupancy = mean_occupancy / data_count;
                        parent_data.weight = ceil((float) mean_weight) / data_count;
                        parent_data.observed = false;

                        parent_min_data.occupancy = min_mean_occupancy;
                        parent_min_data.weight = min_weight;
                        parent_max_data.occupancy = max_mean_occupancy;
                        parent_max_data.weight = max_weight;
                        if (observed_count == 8) {
                            parent_max_data.observed =
                                true; // TODO: We don't set the observed count to true for mean values
                            parent_min_data.observed = true;
                        }
                    }

                } // x
            }     // y
        }         // z
    }
}



} // namespace updater
} // namespace se

#endif // SE_MULTIRES_OFUSION_CORE_IMPL_HPP
