#ifndef SE_RAYCASTER_HPP
#define SE_RAYCASTER_HPP

#define INVALID -2

#include <fstream>
#include <sstream>
#include <iostream>

#include "se/image/image.hpp"
#include "se/timings.hpp"
#include "se/octree/voxel_block_ray_iterator.hpp"



namespace se {
namespace raycaster {


static se::Image<int> scale_image(640, 480);
static std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
color_map =
{
        {102, 194, 165},
        {252, 141, 98},
        {141, 160, 203},
        {231, 138, 195},
        {166, 216, 84},
        {255, 217, 47},
        {229, 196, 148},
        {179, 179, 179},
};

void pointCloudToNormalKernel(se::Image<Eigen::Vector3f>&       normals,
                              const se::Image<Eigen::Vector3f>& point_cloud,
                              const bool                        is_lhc = false);

template <typename MapT>
inline Eigen::Vector4f raycast(MapT&                  map,
                               const Eigen::Vector3f& ray_origin_M,
                               const Eigen::Vector3f& ray_dir_M,
                               const float            t_near,
                               const float            t_far,
                               const float            mu,
                               const float            step,
                               const float            largestep);

template<typename MapT, typename SensorT>
void raycastVolume(MapT&                       map,
                   se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                   se::Image<Eigen::Vector3f>& surface_normals_M,
                   const Eigen::Matrix4f&      T_MS,
                   const SensorT&              sensor);

void renderVolumeKernel(uint32_t*                         volume_RGBA_image_data,
                        const Eigen::Vector2i&            volume_RGBA_image_res,
                        const Eigen::Vector3f&            light_M,
                        const Eigen::Vector3f&            ambient_M,
                        const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                        const se::Image<Eigen::Vector3f>& surface_normals_M);



} // namespace raycaster
} // namespace se

#include "impl/raycaster_impl.hpp"

#endif // SE_RAYCASTER_HPP
