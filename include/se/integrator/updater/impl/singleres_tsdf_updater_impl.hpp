/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
#define SE_SINGLERES_TSDF_UPDATER_IMPL_HPP



namespace se {



// Single-res TSDF updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::Updater(
    MapType& map,
    const SensorT& sensor,
    const Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const int frame) :
        map_(map), sensor_(sensor), depth_img_(depth_img), T_WS_(T_WS), frame_(frame), config_(map)
{
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::operator()(
    std::vector<OctantBase*>& block_ptrs)
{
    constexpr int block_size = BlockType::getSize();
    const Eigen::Matrix4f T_SW = math::to_inverse_transformation(T_WS_);
    const Eigen::Matrix3f C_SW = math::to_rotation(T_SW);

#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++) {
        BlockType& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.setTimeStamp(frame_);
        const Eigen::Vector3i block_coord = block.getCoord();
        Eigen::Vector3f point_base_W;
        map_.voxelToPoint(block_coord, point_base_W);
        const Eigen::Vector3f point_base_S = (T_SW * point_base_W.homogeneous()).head<3>();
        const Eigen::Matrix3f point_delta_matrix_S = C_SW * map_.getRes();

        for (unsigned int z = 0; z < block_size; ++z) {
            for (unsigned int y = 0; y < block_size; ++y) {
                for (unsigned int x = 0; x < block_size; ++x) {
                    // Set voxel coordinates
                    const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);

                    // Set sample point in camera frame
                    const Eigen::Vector3f point_S =
                        point_base_S + point_delta_matrix_S * Eigen::Vector3f(x, y, z);

                    if (point_S.norm() > sensor_.farDist(point_S)) {
                        continue;
                    }

                    // Project sample point to the image plane.
                    Eigen::Vector2f pixel_f;
                    if (sensor_.model.project(point_S, &pixel_f)
                        != srl::projection::ProjectionStatus::Successful) {
                        continue;
                    }
                    const Eigen::Vector2i pixel = se::round_pixel(pixel_f);
                    const int pixel_idx = pixel.x() + depth_img_.width() * pixel.y();

                    // Fetch the image value.
                    const float depth_value = depth_img_[pixel_idx];
                    if (depth_value < sensor_.near_plane) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = sensor_.measurementFromPoint(point_S);
                    const field_t sdf_value = point_S.norm() * (depth_value - m) / m;

                    if (sdf_value > -config_.truncation_boundary) {
                        DataType& data = block.getData(voxel_coord);
                        updateVoxel(data, sdf_value);
                    }
                } // x
            }     // y
        }         // z
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::updateVoxel(
    DataType& data,
    field_t sdf_value)
{
    weight::increment(data.weight, map_.getDataConfig().max_weight);
    const field_t tsdf_value = std::min(field_t(1), sdf_value / config_.truncation_boundary);
    data.tsdf = (data.tsdf * (data.weight - 1) + tsdf_value) / data.weight;
    data.tsdf = math::clamp(data.tsdf, field_t(-1), field_t(1));
}



} // namespace se

#endif // SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
