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
    const Eigen::Isometry3f& T_WS,
    const int frame) :
        map_(map), sensor_(sensor), depth_img_(depth_img), T_WS_(T_WS), frame_(frame), config_(map)
{
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::operator()(
    std::vector<OctantBase*>& block_ptrs)
{
    unsigned int block_size = BlockType::getSize();
    const Eigen::Isometry3f T_SW = T_WS_.inverse();

#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++) {
        BlockType* block_ptr = static_cast<BlockType*>(block_ptrs[i]);
        block_ptr->setTimeStamp(frame_);
        Eigen::Vector3i block_coord = block_ptr->getCoord();
        Eigen::Vector3f point_base_W;
        map_.voxelToPoint(block_coord, point_base_W);
        const Eigen::Vector3f point_base_S = T_SW * point_base_W;
        const Eigen::Matrix3f point_delta_matrix_S = T_SW.linear() * map_.getRes();

        for (unsigned int i = 0; i < block_size; ++i) {
            for (unsigned int j = 0; j < block_size; ++j) {
                for (unsigned int k = 0; k < block_size; ++k) {
                    // Set voxel coordinates
                    Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(i, j, k);

                    // Set sample point in camera frame
                    Eigen::Vector3f point_S =
                        point_base_S + point_delta_matrix_S * Eigen::Vector3f(i, j, k);

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

                    DataType& data = block_ptr->getData(voxel_coord);
                    data.field.update(sdf_value,
                                      config_.truncation_boundary,
                                      map_.getDataConfig().field.max_weight);
                } // k
            }     // j
        }         // i
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}

} // namespace se

#endif // SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
