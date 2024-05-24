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
    const SensorT* const colour_sensor,
    const Image<colour_t>* const colour_img,
    const Eigen::Isometry3f* const T_SSc,
    const int frame) :
        map_(map),
        sensor_(sensor),
        depth_img_(depth_img),
        T_WS_(T_WS),
        colour_sensor_(colour_sensor),
        colour_img_(colour_img),
        has_colour_(colour_sensor_ && colour_img_ && T_SSc),
        frame_(frame),
        config_(map)
{
    if constexpr (MapType::col_ == Colour::On) {
        if (has_colour_) {
            T_ScS_ = T_SSc->inverse();
        }
    }
}



template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
void Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT>::operator()(
    std::vector<OctantBase*>& block_ptrs)
{
    const Eigen::Isometry3f T_SW = T_WS_.inverse();
    // Transformation from the octree frame v (in voxels) to the sensor frame S (in meters).
    const Eigen::Affine3f T_Sv = T_SW * map_.getTWM() * Eigen::Scaling(map_.getRes())
        * Eigen::Translation3f(sample_offset_frac);

#pragma omp parallel for
    for (size_t i = 0; i < block_ptrs.size(); i++) {
        assert(block_ptrs[i]);
        assert(block_ptrs[i]->is_block);
        auto& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.timestamp = frame_;
        const Eigen::Vector3i block_coord = block.coord;

        for (int x = 0; x < BlockType::getSize(); ++x) {
            for (int y = 0; y < BlockType::getSize(); ++y) {
                for (int z = 0; z < BlockType::getSize(); ++z) {
                    const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
                    // Compute the coordinates of the voxel sample position in the sensor frame.
                    const Eigen::Vector3f point_S = T_Sv * voxel_coord.cast<float>();

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

                    DataType& data = block.getData(voxel_coord);
                    const bool field_updated =
                        data.field.update(sdf_value,
                                          config_.truncation_boundary,
                                          map_.getDataConfig().field.max_weight);

                    // Compute the coordinates of the depth hit in the depth sensor frame S if data
                    // other than depth needs to be integrated.
                    Eigen::Vector3f hit_S;
                    if constexpr (MapType::col_ == Colour::On || MapType::sem_ == Semantics::On) {
                        if (has_colour_ && field_updated) {
                            sensor_.model.backProject(depth_pixel_f, &hit_S);
                            hit_S.array() *= depth_value;
                        }
                    }

                    // Update the colour data if possible and only if the field was updated, that is
                    // if we have corresponding depth information.
                    if constexpr (MapType::col_ == Colour::On) {
                        if (has_colour_ && field_updated) {
                            // Project the depth hit onto the colour image.
                            const Eigen::Vector3f hit_Sc = T_ScS_ * hit_S;
                            Eigen::Vector2f colour_pixel_f;
                            if (colour_sensor_->model.project(hit_Sc, &colour_pixel_f)
                                == srl::projection::ProjectionStatus::Successful) {
                                const Eigen::Vector2i colour_pixel =
                                    se::round_pixel(colour_pixel_f);
                                data.colour.update(
                                    (*colour_img_)(colour_pixel.x(), colour_pixel.y()),
                                    map_.getDataConfig().field.max_weight);
                            }
                        }
                    }
                }
            }
        }
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}

} // namespace se

#endif // SE_SINGLERES_TSDF_UPDATER_IMPL_HPP
