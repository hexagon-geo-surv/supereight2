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
    const Eigen::Isometry3f& T_WS,
    const int frame) :
        map_(map), sensor_(sensor), depth_img_(depth_img), T_WS_(T_WS), frame_(frame), config_(map)
{
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::operator()(
    std::vector<OctantBase*>& block_ptrs)
{
    const Eigen::Isometry3f T_SW = T_WS_.inverse();
    const Octree<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>& octree = map_.getOctree();

#pragma omp parallel for
    for (size_t i = 0; i < block_ptrs.size(); i++) {
        assert(block_ptrs[i]);
        assert(block_ptrs[i]->isBlock());
        auto& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.setTimeStamp(frame_);
        const Eigen::Vector3i block_coord = block.getCoord();
        Eigen::Vector3f block_centre_point_W;
        map_.voxelToPoint(block_coord, block.getSize(), block_centre_point_W);
        const Eigen::Vector3f block_centre_point_S = T_SW * block_centre_point_W;
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
                data_union.prop_data.field.delta_tsdf = data_union.data.field.tsdf;
                data_union.prop_data.field.delta_weight = 0;
            };

            auto child_down_funct = [&](const OctreeType& octree,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& child_data_union,
                                        typename BlockType::DataUnion& parent_data_union) {
                field_t delta_tsdf = parent_data_union.data.field.tsdf
                    - parent_data_union.prop_data.field.delta_tsdf;

                if (child_data_union.data.field.weight != 0) {
                    child_data_union.data.field.tsdf =
                        std::max(child_data_union.data.field.tsdf + delta_tsdf, field_t(-1));
                    child_data_union.data.field.weight =
                        fminf(child_data_union.data.field.weight
                                  + parent_data_union.prop_data.field.delta_weight,
                              map_.getDataConfig().field.max_weight);
                    ;
                    child_data_union.prop_data.field.delta_weight =
                        parent_data_union.prop_data.field.delta_weight;
                }
                else {
                    const Eigen::Vector3f child_sample_coord_f =
                        get_sample_coord(child_data_union.coord, 1 << child_data_union.scale);
                    int child_scale_returned;
                    auto interp_field_value = visitor::getFieldInterp(
                        octree, child_sample_coord_f, child_data_union.scale, child_scale_returned);

                    if (interp_field_value) {
                        child_data_union.data.field.tsdf = *interp_field_value;
                        child_data_union.data.field.weight = parent_data_union.data.field.weight;

                        child_data_union.prop_data.field.delta_tsdf =
                            child_data_union.data.field.tsdf;
                        child_data_union.prop_data.field.delta_weight = 0;
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
        const Eigen::Vector3f point_base_S = T_SW * point_base_W;
        const Eigen::Matrix3f point_delta_matrix_S = T_SW.linear() * map_.getRes();

        for (int x = 0; x < BlockType::getSize(); x += stride) {
            for (int y = 0; y < BlockType::getSize(); y += stride) {
                for (int z = 0; z < BlockType::getSize(); z += stride) {
                    // Set voxel coordinates
                    const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);

                    // Set sample point in camera frame
                    const Eigen::Vector3f point_S =
                        point_base_S + point_delta_matrix_S * Eigen::Vector3f(x, y, z);

                    if (point_S.norm() > sensor_.farDist(point_S)) {
                        continue;
                    }

                    // Get the depth value this voxel projects into.
                    Eigen::Vector2f depth_pixel_f;
                    if (sensor_.model.project(point_S, &depth_pixel_f)
                        != srl::projection::ProjectionStatus::Successful) {
                        continue;
                    }
                    const Eigen::Vector2i depth_pixel = se::round_pixel(depth_pixel_f);
                    const float depth_value = depth_img_(depth_pixel.x(), depth_pixel.y());
                    if (depth_value < sensor_.near_plane) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = sensor_.measurementFromPoint(point_S);
                    const field_t sdf_value = (depth_value - m) / m * point_S.norm();

                    typename BlockType::DataUnion data_union =
                        block.getDataUnion(voxel_coord, block.getCurrentScale());
                    data_union.data.field.update(sdf_value,
                                                 config_.truncation_boundary,
                                                 map_.getDataConfig().field.max_weight);
                    data_union.prop_data.field.delta_weight++;
                    block.setDataUnion(data_union);
                }
            }
        }


        auto parent_up_funct = [](typename BlockType::DataUnion& parent_data_union,
                                  typename BlockType::DataType& data_tmp,
                                  const int sample_count) {
            if (sample_count != 0) {
                data_tmp.field.tsdf /= sample_count;
                data_tmp.field.weight /= sample_count;
                parent_data_union.data.field.tsdf = data_tmp.field.tsdf;
                parent_data_union.prop_data.field.delta_tsdf = data_tmp.field.tsdf;
                parent_data_union.data.field.weight = ceil(data_tmp.field.weight);
                parent_data_union.prop_data.field.delta_weight = 0;
            }
            else {
                parent_data_union.data = typename BlockType::DataType();
                parent_data_union.prop_data = typename BlockType::PropDataType();
            }
        };

        auto child_up_funct = [](typename BlockType::DataUnion& child_data_union,
                                 typename BlockType::DataType& data_tmp) {
            if (child_data_union.data.field.weight != 0) {
                data_tmp.field.tsdf += child_data_union.data.field.tsdf;
                data_tmp.field.weight += child_data_union.data.field.weight;
                return 1;
            }
            return 0;
        };

        propagator::propagateBlockUp(octree, &block, curr_scale, child_up_funct, parent_up_funct);
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}

} // namespace se

#endif // SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
