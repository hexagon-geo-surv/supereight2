#ifndef SE_RAYCASTER_HPP
#define SE_RAYCASTER_HPP

#define INVALID -2

#include <fstream>
#include <sstream>
#include <iostream>

#include "se/image/image.hpp"
#include "se/common/colour_utils.hpp"
#include "se/common/timings.hpp"
#include "se/map/octree/voxel_block_ray_iterator.hpp"



namespace se {
namespace raycaster {

void point_cloud_to_normal(se::Image<Eigen::Vector3f>&       normals,
                           const se::Image<Eigen::Vector3f>& point_cloud,
                           const bool                        is_lhc = false);

template <typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::Occupancy, std::optional<Eigen::Vector4f>>
raycast(MapT&                  map,
        const Eigen::Vector3f& ray_origin_M,
        const Eigen::Vector3f& ray_dir_M,
        const float            t_near,
        const float            t_far,
        const float            mu,
        const float            step,
        const float            largestep);

template <typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::TSDF, std::optional<Eigen::Vector4f>>
raycast(MapT&                  map,
        const Eigen::Vector3f& ray_origin_M,
        const Eigen::Vector3f& ray_dir_M,
        const float            t_near,
        const float            t_far,
        const float            mu,
        const float            step,
        const float            largestep);

template<typename MapT, typename SensorT>
void raycastVolume(const MapT&                 map,
                   se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                   se::Image<Eigen::Vector3f>& surface_normals_M,
                   se::Image<int8_t>&          surface_scale,
                   const Eigen::Matrix4f&      T_MS,
                   const SensorT&              sensor);

void renderVolumeKernel(uint32_t*                         volume_RGBA_image_data,
                        const Eigen::Vector2i&            volume_RGBA_image_res,
                        const Eigen::Vector3f&            light_M,
                        const Eigen::Vector3f&            ambient_M,
                        const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                        const se::Image<Eigen::Vector3f>& surface_normals_M,
                        const se::Image<int8_t>&          surface_scale);



} // namespace raycaster
} // namespace se

#include "impl/raycaster_impl.hpp"

#endif // SE_RAYCASTER_HPP
