/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCASTER_HPP
#define SE_RAYCASTER_HPP

#define INVALID -2

#include "se/common/colour_utils.hpp"
#include "se/image/image.hpp"
#include "se/map/octree/visitor.hpp"
#include "se/map/octree/voxel_block_ray_iterator.hpp"



namespace se {
namespace raycaster {

void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals,
                           const se::Image<Eigen::Vector3f>& point_cloud,
                           const bool is_lhc = false);

template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::Occupancy, std::optional<Eigen::Vector4f>>
raycast(MapT& map,
        const Eigen::Vector3f& ray_origin_W,
        const Eigen::Vector3f& ray_dir_W,
        const float t_near,
        const float t_far,
        const float mu,
        const float step,
        const float largestep);

template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::TSDF, std::optional<Eigen::Vector4f>>
raycast(MapT& map,
        const Eigen::Vector3f& ray_origin_W,
        const Eigen::Vector3f& ray_dir_W,
        const float t_near,
        const float t_far,
        const float mu,
        const float step,
        const float largestep);

template<typename MapT, typename SensorT>
void raycastVolume(const MapT& map,
                   se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                   se::Image<Eigen::Vector3f>& surface_normals_W,
                   se::Image<int8_t>& surface_scale,
                   const Eigen::Matrix4f& T_WS,
                   const SensorT& sensor);

void renderVolumeKernel(uint32_t* volume_RGBA_image_data,
                        const Eigen::Vector2i& volume_RGBA_image_res,
                        const Eigen::Vector3f& light_W,
                        const Eigen::Vector3f& ambient_W,
                        const se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                        const se::Image<Eigen::Vector3f>& surface_normals_W,
                        const se::Image<int8_t>& surface_scale);



} // namespace raycaster
} // namespace se

#include "impl/raycaster_impl.hpp"

#endif // SE_RAYCASTER_HPP
