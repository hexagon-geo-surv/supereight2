/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
#define SE_MULTIRES_TSDF_UPDATER_IMPL_HPP

namespace se {

// Multi-res TSDF updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>, SensorT>::Updater(
    MapType& map,
    std::vector<OctantBase*>& block_ptrs,
    const SensorT& sensor,
    const Image<float>& depth_img,
    const Eigen::Isometry3f& T_WS,
    const SensorT* const colour_sensor,
    const Image<colour_t>* const colour_img,
    const Eigen::Isometry3f* const T_SSc,
    const timestamp_t timestamp)
{
    const bool has_colour = MapType::col_ == Colour::On && colour_sensor && colour_img && T_SSc;
    Eigen::Isometry3f T_ScS;
    if constexpr (MapType::col_ == Colour::On) {
        if (has_colour) {
            T_ScS = T_SSc->inverse();
        }
    }

    const float truncation_boundary =
        map.getRes() * map.getDataConfig().field.truncation_boundary_factor;
    const Eigen::Isometry3f T_SW = T_WS.inverse();
    // Transformation from the octree frame v (in voxels) to the sensor frame S (in meters).
    const Eigen::Affine3f T_Sv = T_SW * map.getTWM() * Eigen::Scaling(map.getRes())
        * Eigen::Translation3f(sample_offset_frac);
    const Octree<Data<Field::TSDF, ColB, SemB>, Res::Multi, BlockSize>& octree = map.getOctree();

#pragma omp parallel for
    for (size_t i = 0; i < block_ptrs.size(); i++) {
        assert(block_ptrs[i]);
        assert(block_ptrs[i]->is_block);
        auto& block = *static_cast<BlockType*>(block_ptrs[i]);
        block.timestamp = timestamp;
        const Eigen::Vector3i block_coord = block.coord;
        Eigen::Vector3f block_centre_point_W;
        map.voxelToPoint(block_coord, block.getSize(), block_centre_point_W);
        const Eigen::Vector3f block_centre_point_S = T_SW * block_centre_point_W;
        const int last_curr_scale = block.getCurrentScale();
        const int lower_curr_scale_limit = last_curr_scale - 1;

        const int curr_scale = std::max(sensor.computeIntegrationScale(block_centre_point_S,
                                                                       map.getRes(),
                                                                       last_curr_scale,
                                                                       block.getMinScale(),
                                                                       block.getMaxScale()),
                                        lower_curr_scale_limit);

        block.setMinScale(block.getMinScale() < 0 ? curr_scale
                                                  : std::min(block.getMinScale(), curr_scale));

        // Down-propagate the block data to the new, finer scale.
        if (curr_scale < last_curr_scale) {
            // Reset the parent delta data after it has been down-propagated to all its children.
            auto parent_down_funct = [](const OctreeType& /* octree */,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& data_union) {
                data_union.past_data.field.tsdf = data_union.data.field.tsdf;
                data_union.past_data.field.weight = 0;
                if constexpr (MapType::col_ == Colour::On) {
                    data_union.past_data.colour.colour = data_union.data.colour.colour;
                    data_union.past_data.colour.weight = 0;
                }
            };

            // Update or initialize the child data using the parent data.
            auto child_down_funct = [&](const OctreeType& octree,
                                        OctantBase* /* octant_ptr */,
                                        typename BlockType::DataUnion& child_data_union,
                                        const typename BlockType::DataUnion& parent_data_union) {
                if (is_valid(child_data_union.data)) {
                    // Perform a delta update on the child using the parent data.
                    const field_t delta_tsdf =
                        parent_data_union.data.field.tsdf - parent_data_union.past_data.field.tsdf;
                    child_data_union.data.field.tsdf =
                        std::max(child_data_union.data.field.tsdf + delta_tsdf, field_t(-1));
                    child_data_union.data.field.weight =
                        fminf(child_data_union.data.field.weight
                                  + parent_data_union.past_data.field.weight,
                              map.getDataConfig().field.max_weight);
                    child_data_union.past_data.field.weight =
                        parent_data_union.past_data.field.weight;
                }
                else {
                    // This child hasn't been observed before, initialize to the interpolated field
                    // value.
                    const int child_size = octantops::scale_to_size(child_data_union.scale);
                    const Eigen::Vector3f child_sample_coord_f =
                        child_data_union.coord.template cast<float>()
                        + sample_offset_frac * child_size;
                    const auto interp_field_value = visitor::getFieldInterp(
                        octree, child_sample_coord_f, child_data_union.scale);
                    if (interp_field_value) {
                        child_data_union.data.field.tsdf = *interp_field_value;
                        child_data_union.data.field.weight = parent_data_union.data.field.weight;
                        child_data_union.past_data.field.tsdf = child_data_union.data.field.tsdf;
                        child_data_union.past_data.field.weight = 0;
                        // Whether interpolation succeeds depends only on the field data.
                        if constexpr (MapType::col_ == Colour::On) {
                            const auto interp_colour_value = visitor::getColourInterp(
                                octree, child_sample_coord_f, child_data_union.scale);
                            assert(interp_colour_value);
                            child_data_union.data.colour.colour = *interp_colour_value;
                            child_data_union.data.colour.weight =
                                parent_data_union.data.field.weight;
                            child_data_union.past_data.colour.colour =
                                child_data_union.data.colour.colour;
                            child_data_union.past_data.colour.weight = 0;
                        }
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

        for (int x = 0; x < BlockType::getSize(); x += stride) {
            for (int y = 0; y < BlockType::getSize(); y += stride) {
                for (int z = 0; z < BlockType::getSize(); z += stride) {
                    const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
                    // Compute the coordinates of the voxel sample position in the sensor frame.
                    const Eigen::Vector3f point_S = T_Sv * voxel_coord.cast<float>();

                    if (point_S.norm() > sensor.farDist(point_S)) {
                        continue;
                    }

                    // Get the depth value this voxel projects into.
                    Eigen::Vector2f depth_pixel_f;
                    if (sensor.model.project(point_S, &depth_pixel_f)
                        != srl::projection::ProjectionStatus::Successful) {
                        continue;
                    }
                    const Eigen::Vector2i depth_pixel = se::round_pixel(depth_pixel_f);
                    const float depth_value = depth_img(depth_pixel.x(), depth_pixel.y());
                    if (depth_value < sensor.near_plane) {
                        continue;
                    }

                    // Update the TSDF
                    const float m = sensor.measurementFromPoint(point_S);
                    const field_t sdf_value = (depth_value - m) / m * point_S.norm();

                    typename BlockType::DataUnion data_union =
                        block.getDataUnion(voxel_coord, block.getCurrentScale());
                    const bool field_updated = data_union.data.field.update(
                        sdf_value, truncation_boundary, map.getDataConfig().field.max_weight);
                    data_union.past_data.field.weight++;

                    // Compute the coordinates of the depth hit in the depth sensor frame S if data
                    // other than depth needs to be integrated.
                    Eigen::Vector3f hit_S;
                    if constexpr (MapType::col_ == Colour::On || MapType::sem_ == Semantics::On) {
                        if (has_colour && field_updated) {
                            sensor.model.backProject(depth_pixel_f, &hit_S);
                            hit_S.array() *= depth_value;
                        }
                    }

                    // Update the colour data if possible and only if the field was updated, that is
                    // if we have corresponding depth information.
                    if constexpr (MapType::col_ == Colour::On) {
                        if (has_colour && field_updated) {
                            // Project the depth hit onto the colour image.
                            const Eigen::Vector3f hit_Sc = T_ScS * hit_S;
                            Eigen::Vector2f colour_pixel_f;
                            if (colour_sensor->model.project(hit_Sc, &colour_pixel_f)
                                == srl::projection::ProjectionStatus::Successful) {
                                const Eigen::Vector2i colour_pixel =
                                    se::round_pixel(colour_pixel_f);
                                data_union.data.colour.update(
                                    (*colour_img)(colour_pixel.x(), colour_pixel.y()),
                                    map.getDataConfig().field.max_weight);
                            }
                        }
                    }

                    block.setDataUnion(data_union);
                }
            }
        }

        // Up-propagate the block data to the coarser scales by setting the parent data to the mean
        // of the valid child data.
        auto aggregate_children_funct =
            [](typename BlockType::DataUnion& parent_data_union,
               const std::array<typename BlockType::DataType, 8>& child_data) {
                const int valid_children = data::up_prop_mean(parent_data_union.data, child_data);
                if (valid_children > 0) {
                    parent_data_union.past_data.field.tsdf = parent_data_union.data.field.tsdf;
                    parent_data_union.past_data.field.weight = 0;
                }
                else {
                    parent_data_union.past_data = typename BlockType::PastDataType();
                }
            };
        propagator::propagateBlockUp(octree, &block, curr_scale, aggregate_children_funct);
    }

    propagator::propagateTimeStampToRoot(block_ptrs);
}

} // namespace se

#endif // SE_MULTIRES_TSDF_UPDATER_IMPL_HPP
