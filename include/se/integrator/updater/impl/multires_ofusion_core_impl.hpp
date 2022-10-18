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
            * math::clamp(config.k_sigma * depth_value, sigma_min, sigma_max); // Livingroom dataset
    }
    else {
        return 3
            * math::clamp(config.k_sigma * math::sq(depth_value),
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
    if (config.const_surface_thickness) {
        return tau_max; ///<< e.g. used in ICL-NUIM livingroom dataset.
    }
    else {
        return math::clamp(config.k_tau * depth_value, tau_min, tau_max);
    }
}



namespace updater {



template<typename DataT>
bool weighted_mean_update(DataT& data, const field_t sample_value, const weight_t max_weight)
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

    return weighted_mean_update(data, sample_value, config.max_weight);
}



template<typename DataT, typename ConfigT>
void free_node(DataT& node_data, const ConfigT config)
{
    weighted_mean_update(node_data, config.log_odd_min, config.max_weight);
}



template<typename DataT, typename ConfigT>
bool free_voxel(DataT& voxel_data, const ConfigT config)
{
    return weighted_mean_update(voxel_data, config.log_odd_min, config.max_weight);
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
    int observed_count = 0;
    int data_count = 0;
    for (int child_idx = 0; child_idx < 8; ++child_idx) {
        const OctantBase* child_ptr = node.getChild(child_idx);
        if (!child_ptr) {
            continue;
        }
        const auto& child_data = child_ptr->isBlock()
            ? static_cast<const BlockT*>(child_ptr)->getMaxData()
            : static_cast<const NodeT*>(child_ptr)->getMaxData();
        // Only consider children with at least 1 integration.
        if (child_data.weight > 0 && child_data.occupancy * child_data.weight > max_occupancy) {
            max_mean_occupancy = child_data.occupancy;
            max_weight = child_data.weight;
            max_occupancy = max_mean_occupancy * max_weight;
            data_count++;
        }
        if (child_data.observed == true) {
            observed_count++;
        }
    }

    // Update the node data if necessary.
    typename NodeT::DataType node_data = node.getData();
    if (data_count > 0) {
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

    int child_scale = block.getCurrentScale();
    int size_at_child_scale_li = BlockT::size >> child_scale;
    int size_at_child_scale_sq = math::sq(size_at_child_scale_li);

    int parent_scale = child_scale + 1;
    int size_at_parent_scale_li = BlockT::size >> parent_scale;
    int size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

    DataType min_data;
    field_t min_occupancy;

    if (block.buffer_scale() > block.getCurrentScale()) {
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* max_data_at_child_scale = block.blockDataAtScale(child_scale);

        min_data = max_data_at_child_scale[0];
        min_occupancy = min_data.occupancy * min_data.weight;

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_max_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_max_data = max_data_at_parent_scale[parent_max_data_idx];

                    field_t max_mean_occupancy = 0;
                    weight_t max_weight = 0;
                    field_t max_occupancy = -std::numeric_limits<float>::max();

                    size_t observed_count = 0;
                    size_t data_count = 0;

                    for (int k = 0; k < 2; k++) {
                        for (int j = 0; j < 2; j++) {
                            for (int i = 0; i < 2; i++) {
                                const int child_max_data_idx = (2 * x + i)
                                    + (2 * y + j) * size_at_child_scale_li
                                    + (2 * z + k) * size_at_child_scale_sq;
                                const auto child_data = max_data_at_child_scale[child_max_data_idx];

                                field_t occupancy = (child_data.occupancy * child_data.weight);

                                if (child_data.weight > 0) {
                                    if (occupancy > max_occupancy) {
                                        data_count++;
                                        // Update max
                                        max_mean_occupancy = child_data.occupancy;
                                        max_weight = child_data.weight;
                                        max_occupancy = max_mean_occupancy * max_weight;
                                    }
                                    else if (occupancy < min_occupancy) {
                                        min_data.occupancy = child_data.occupancy;
                                        min_data.weight = child_data.weight;
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
        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        min_data = data_at_child_scale[0];
        min_occupancy = min_data.occupancy * min_data.weight;

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    field_t mean_occupancy = 0;
                    weight_t mean_weight = 0;

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
                                const auto child_data = data_at_child_scale[child_data_idx];

                                if (child_data.weight > 0) {
                                    // Update mean
                                    data_count++;
                                    mean_occupancy += child_data.occupancy;
                                    mean_weight += child_data.weight;

                                    field_t occupancy = (child_data.occupancy * child_data.weight);

                                    if (occupancy > max_occupancy) {
                                        // Update max
                                        max_mean_occupancy = child_data.occupancy;
                                        max_weight = child_data.weight;
                                        max_occupancy = max_mean_occupancy * max_weight;
                                    }
                                    else if (occupancy < min_occupancy) {
                                        min_data.occupancy = child_data.occupancy;
                                        min_data.weight = child_data.weight;
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
                        parent_data.weight =
                            weight::div(mean_weight, static_cast<weight_t>(data_count));
                        parent_data.observed = false;

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



    for (parent_scale += 1; parent_scale <= BlockT::getMaxScale(); ++parent_scale) {
        size_at_parent_scale_li = BlockT::size >> parent_scale;
        size_at_parent_scale_sq = math::sq(size_at_parent_scale_li);

        child_scale = parent_scale - 1;
        size_at_child_scale_li = BlockT::size >> child_scale;
        size_at_child_scale_sq = math::sq(size_at_child_scale_li);

        DataType* max_data_at_parent_scale = block.blockMaxDataAtScale(parent_scale);
        DataType* data_at_parent_scale = block.blockDataAtScale(parent_scale);
        DataType* max_data_at_child_scale = block.blockMaxDataAtScale(child_scale);
        DataType* data_at_child_scale = block.blockDataAtScale(child_scale);

        for (int z = 0; z < size_at_parent_scale_li; z++) {
            for (int y = 0; y < size_at_parent_scale_li; y++) {
                for (int x = 0; x < size_at_parent_scale_li; x++) {
                    const int parent_data_idx =
                        x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
                    auto& parent_data = data_at_parent_scale[parent_data_idx];
                    auto& parent_max_data = max_data_at_parent_scale[parent_data_idx];

                    field_t mean_occupancy = 0;
                    weight_t mean_weight = 0;

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
                        parent_data.occupancy = mean_occupancy / data_count;
                        parent_data.weight =
                            weight::div(mean_weight, static_cast<weight_t>(data_count));
                        parent_data.observed = false;

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

    block.setMinData(min_data);
}



} // namespace updater
} // namespace se

#endif // SE_MULTIRES_OFUSION_CORE_IMPL_HPP
