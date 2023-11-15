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
    unsigned int block_size = BlockType::getSize();
    const Eigen::Matrix4f T_SW = math::to_inverse_transformation(T_WS_);

    auto valid_predicate = [&](float depth_value) { return depth_value >= sensor_.near_plane; };

#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++) {
        BlockType* block_ptr = static_cast<BlockType*>(block_ptrs[i]);
        block_ptr->setTimeStamp(frame_);
        Eigen::Vector3i block_coord = block_ptr->getCoord();
        Eigen::Vector3f point_base_W;
        map_.voxelToPoint(block_coord, point_base_W);
        const Eigen::Vector3f point_base_S = (T_SW * point_base_W.homogeneous()).head(3);
        const Eigen::Matrix3f point_delta_matrix_S =
            (math::to_rotation(T_SW) * map_.getRes() * Eigen::Matrix3f::Identity());

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

                    // Fetch image value
                    float depth_value(0);
                    if (!sensor_.projectToPixelValue(
                            point_S, depth_img_, depth_value, valid_predicate)) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = sensor_.measurementFromPoint(point_S);
                    const field_t sdf_value = (depth_value - m) / m * point_S.norm();

                    DataType& data = block_ptr->getData(voxel_coord);
                    updateVoxel(data, sdf_value);
                } // k
            }     // j
        }         // i
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::updateVoxel(
    DataType& data,
    const field_t sdf_value)
{
    if (sdf_value > -config_.truncation_boundary) {
        const float tsdf_value = std::min(1.f, sdf_value / config_.truncation_boundary);

        data.tsdf = (data.tsdf * data.weight + tsdf_value) / (data.weight + 1.f);
        data.tsdf = std::clamp(data.tsdf, field_t(-1), field_t(1));
        data.weight = std::min(data.weight + 1, map_.getDataConfig().max_weight);
    }
}



} // namespace se

#endif // SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
