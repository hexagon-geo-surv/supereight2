/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
#define SE_MULTIRES_TSDF_UPDATER_IMPL_HPP


namespace se {



// Multi-res TSDF updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::Updater(
    MapType& map,
    const SensorT& sensor,
    const Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const int frame) :
        map_(map), sensor_(sensor), depth_img_(depth_img), T_WS_(T_WS), frame_(frame), config_(map)
{
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::operator()(
    std::vector<OctantBase*>& block_ptrs)
{
    constexpr int block_size = BlockType::getSize();
    const Eigen::Matrix4f T_SW = math::to_inverse_transformation(T_WS_);
    const Eigen::Matrix3f C_SW = math::to_rotation(T_SW);
    const Octree<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>& octree = *map_.getOctree();

    auto valid_predicate = [&](float depth_value) { return depth_value >= sensor_.near_plane; };

#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++) {
        BlockType& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.setTimeStamp(frame_);
        const Eigen::Vector3i block_coord = block.getCoord();
        Eigen::Vector3f block_centre_point_W;
        map_.voxelToPoint(block_coord, block.getSize(), block_centre_point_W);
        const Eigen::Vector3f block_centre_point_S =
            (T_SW * block_centre_point_W.homogeneous()).head<3>();
        const int last_curr_scale = block.getCurrentScale();
        const int lower_curr_scale_limit = last_curr_scale - 1;

        const int curr_scale = std::max(sensor_.computeIntegrationScale(block_centre_point_S,
                                                                        map_.getRes(),
                                                                        last_curr_scale,
                                                                        block.getMinScale(),
                                                                        block.getMaxScale()),
                                        lower_curr_scale_limit);

        block.setMinScale(block.getMinScale() < 0 ? curr_scale
                                                  : std::min(block.getMinScale(), curr_scale));

        if (curr_scale < last_curr_scale) {
            auto parent_down_funct = [](const OctreeType& /* octree */,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& data_union) {
                data_union.prop_data.delta_tsdf = data_union.data.tsdf;
                data_union.prop_data.delta_weight = 0;
            };

            auto child_down_funct = [&](const OctreeType& octree,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& child_data_union,
                                        typename BlockType::DataUnion& parent_data_union) {
                if (child_data_union.data.weight != 0) {
                    const field_t delta_tsdf =
                        parent_data_union.data.tsdf - parent_data_union.prop_data.delta_tsdf;
                    child_data_union.data.tsdf =
                        std::max(child_data_union.data.tsdf + delta_tsdf, field_t(-1));
                    // Use if instead of std::min to prevent overflow.
                    if (child_data_union.data.weight < map_.getDataConfig().max_weight
                            - parent_data_union.prop_data.delta_weight) {
                        child_data_union.data.weight += parent_data_union.prop_data.delta_weight;
                    }
                    child_data_union.prop_data.delta_weight =
                        parent_data_union.prop_data.delta_weight;
                }
                else {
                    const Eigen::Vector3f child_sample_coord_f =
                        get_sample_coord(child_data_union.coord, 1 << child_data_union.scale);
                    int _;
                    auto interp_field_value = visitor::getFieldInterp(
                        octree, child_sample_coord_f, child_data_union.scale, _);

                    if (interp_field_value) {
                        child_data_union.data.tsdf = *interp_field_value;
                        child_data_union.data.weight = parent_data_union.data.weight;

                        child_data_union.prop_data.delta_tsdf = child_data_union.data.tsdf;
                        child_data_union.prop_data.delta_weight = 0;
                    }
                }
            };

            propagator::propagateBlockDown(octree,
                                           static_cast<OctantBase*>(&block),
                                           curr_scale,
                                           child_down_funct,
                                           parent_down_funct);
        }

        block.setCurrentScale(curr_scale);
        const int stride = 1 << curr_scale;

        Eigen::Vector3f point_base_W;
        map_.voxelToPoint(block_coord, stride, point_base_W);
        const Eigen::Vector3f point_base_S = (T_SW * point_base_W.homogeneous()).head<3>();
        const Eigen::Matrix3f point_delta_matrix_S = C_SW * map_.getRes();

        for (unsigned int z = 0; z < block_size; z += stride) {
            for (unsigned int y = 0; y < block_size; y += stride) {
                for (unsigned int x = 0; x < block_size; x += stride) {
                    // Set voxel coordinates
                    const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);

                    // Set sample point in camera frame
                    const Eigen::Vector3f point_S =
                        point_base_S + point_delta_matrix_S * Eigen::Vector3f(x, y, z);

                    if (point_S.norm() > sensor_.farDist(point_S)) {
                        continue;
                    }

                    // Fetch image value
                    float depth_value(0);
                    if (!sensor_.projectToPixelValue(
                            point_S, depth_img_, depth_value, valid_predicate)) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = sensor_.measurementFromPoint(point_S);
                    const field_t sdf_value = point_S.norm() * (depth_value - m) / m;

                    if (sdf_value > -config_.truncation_boundary) {
                        typename BlockType::DataUnion data_union =
                            block.getDataUnion(voxel_coord, block.getCurrentScale());
                        updateVoxel(data_union, sdf_value);
                        block.setDataUnion(data_union);
                    }
                } // x
            }     // y
        }         // z


        auto parent_up_funct = [](typename BlockType::DataUnion& parent_data_union,
                                  typename BlockType::PropDataType& child_data_sum,
                                  const int sample_count) {
            if (sample_count != 0) {
                child_data_sum.delta_tsdf /= sample_count;
                child_data_sum.delta_weight = weight::div(
                    child_data_sum.delta_weight, static_cast<delta_weight_t>(sample_count));
                parent_data_union.data.tsdf = child_data_sum.delta_tsdf;
                parent_data_union.prop_data.delta_tsdf = child_data_sum.delta_tsdf;
                parent_data_union.data.weight = child_data_sum.delta_weight;
                parent_data_union.prop_data.delta_weight = 0;
            }
            else {
                parent_data_union.data = typename BlockType::DataType();
                parent_data_union.prop_data = typename BlockType::PropDataType();
            }
        };

        auto child_up_funct = [](typename BlockType::DataUnion& child_data_union,
                                 typename BlockType::PropDataType& child_data_sum) {
            if (child_data_union.data.weight != 0) {
                child_data_sum.delta_tsdf += child_data_union.data.tsdf;
                child_data_sum.delta_weight += child_data_union.data.weight;
                return 1;
            }
            return 0;
        };

        propagator::propagateBlockUp(
            octree, static_cast<OctantBase*>(&block), curr_scale, child_up_funct, parent_up_funct);
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::updateVoxel(
    typename BlockType::DataUnion& data_union,
    field_t sdf_value)
{
    auto& data = data_union.data;
    weight::increment(data.weight, map_.getDataConfig().max_weight);
    const field_t tsdf_value = std::min(field_t(1), sdf_value / config_.truncation_boundary);
    data.tsdf = (data.tsdf * (data.weight - 1) + tsdf_value) / data.weight;
    data.tsdf = math::clamp(data.tsdf, field_t(-1), field_t(1));
    data_union.prop_data.delta_weight++;
}



} // namespace se

#endif // SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
