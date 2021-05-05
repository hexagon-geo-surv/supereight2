#ifndef SE_RAYCASTER_HPP
#define SE_RAYCASTER_HPP

#define INVALID -2

#include <fstream>
#include <sstream>
#include <iostream>

#include "se/image/image.hpp"
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
                              const bool                        is_lhc = false)
{

//  TICKD("pointCloudToNormalKernel");
  const int width = point_cloud.width();
  const int height = point_cloud.height();

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      const Eigen::Vector3f point = point_cloud[x + width * y];
      if (point.z() == 0.f) {
        normals[x + y * width].x() = INVALID;
        continue;
      }

      const Eigen::Vector2i p_left
              = Eigen::Vector2i(std::max(int(x) - 1, 0), y);
      const Eigen::Vector2i p_right
              = Eigen::Vector2i(std::min(x + 1, (int) width - 1), y);

      // Swapped to match the left-handed coordinate system of ICL-NUIM
      Eigen::Vector2i p_up, p_down;
      if (false) {
        p_up = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
        p_down = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
      } else {
        p_down = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
        p_up = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
      }

      const Eigen::Vector3f left = point_cloud[p_left.x() + width * p_left.y()];
      const Eigen::Vector3f right = point_cloud[p_right.x() + width * p_right.y()];
      const Eigen::Vector3f up = point_cloud[p_up.x() + width * p_up.y()];
      const Eigen::Vector3f down = point_cloud[p_down.x() + width * p_down.y()];

      if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0) {
        normals[x + y * width].x() = INVALID;
        continue;
      }
      const Eigen::Vector3f dv_x = right - left;
      const Eigen::Vector3f dv_y = up - down;
      normals[x + y * width] = dv_x.cross(dv_y).normalized();
    }
  }
//  TOCK("pointCloudToNormalKernel");
}


//template <typename MapT>
//Eigen::Vector4f raycast(const MapT&      map,
//                        const Eigen::Vector3f& ray_origin_M,
//                        const Eigen::Vector3f& ray_dir_M,
//                        const float            t_near,
//                        const float            t_far) {
//
//  float mu = 0.1f;
//  se::VoxelBlockRayIterator ray(map, ray_origin_M, ray_dir_M, t_near, t_far);
//  ray.next();
//  const float t_min = ray.tmin(); /* Get distance to the first intersected block */
//  if (t_min <= 0.f) {
//    return Eigen::Vector4f::Zero();
//  }
//  const float t_max = ray.tmax();
//
//  // first walk with large steps until we found a hit
//  float t = t_min;
//  float step_size = mu / 2;
//  Eigen::Vector3f ray_pos_M = Eigen::Vector3f::Zero();
//
//  float value_t  = 0;
//  float value_tt = 0;
//  Eigen::Vector3f point_M_t = Eigen::Vector3f::Zero();
//  Eigen::Vector3f point_M_tt = Eigen::Vector3f::Zero();
//
//  if (!find_valid_point(map, TSDF::VoxelType::selectNodeValue, TSDF::VoxelType::selectVoxelValue,
//                        ray_origin_M, ray_dir_M, step_size, t_max, t, value_t, point_M_t)) {
//    return Eigen::Vector4f::Zero();
//  }
//  step_size = se::math::clamp(value_t * TSDF::mu, TSDF::mu / 10, TSDF::mu / 2);
//  t += step_size;
//
//  if (value_t > 0) { // ups, if we were already in it, then don't render anything here
//    for (; t < t_max; t += step_size) {
//      ray_pos_M = ray_origin_M + ray_dir_M * t;
//      VoxelData data;
//      map.getAtPoint(ray_pos_M, data);
//      if (data.y == 0) {
//        t += step_size;
//        if (!find_valid_point(map, TSDF::VoxelType::selectNodeValue, TSDF::VoxelType::selectVoxelValue,
//                              ray_origin_M, ray_dir_M, step_size, t_max, t, value_t, point_M_t)) {
//          return Eigen::Vector4f::Zero();
//        }
//        if (value_t < 0) {
//          break;
//        }
//        continue;
//      }
//      value_tt = data.x;
//      point_M_tt = ray_pos_M;
//      if (value_tt <= 0.1) {
//        bool is_valid = false;
//        value_tt = map.interpAtPoint(ray_pos_M, TSDF::VoxelType::selectNodeValue, TSDF::VoxelType::selectVoxelValue, 0, is_valid).first;
//        if (!is_valid) {
//          t += step_size;
//          if (!find_valid_point(map, TSDF::VoxelType::selectNodeValue, TSDF::VoxelType::selectVoxelValue,
//                                ray_origin_M, ray_dir_M, step_size, t_max, t, value_t, point_M_t)) {
//            return Eigen::Vector4f::Zero();
//          }
//          if (value_t < 0) {
//            break;
//          }
//          continue;
//        }
//      }
//      if (value_tt < 0)  {
//        break; // got it, jump out of inner loop
//      }
//      step_size = se::math::clamp(value_tt * TSDF::mu, TSDF::mu / 10, TSDF::mu / 2);
//      value_t = value_tt;
//      point_M_t = point_M_tt;
//    }
//    if (value_tt < 0 && value_t > 0) {
//      // We overshot. Need to move backwards for zero crossing.
//      t = t - (point_M_tt - point_M_t).norm() / (value_tt - value_t) * value_tt; // (value_tt - 0)
//      Eigen::Vector4f surface_point_M = (ray_origin_M + ray_dir_M * t).homogeneous();
//      surface_point_M.w() = 0; // Rendering scale has to be zero for single res implementation
//      return surface_point_M;
//    }
//  }
//  return Eigen::Vector4f::Constant(-1.f);
//}

#define MU 0.16

template <typename MapT>
inline Eigen::Vector4f raycast(MapT&                  map,
                               const Eigen::Vector3f& ray_origin_M,
                               const Eigen::Vector3f& ray_dir_M,
                               const float            t_near,
                               const float            t_far,
                               const float            mu,
                               const float            step,
                               const float            largestep)
{

  se::VoxelBlockRayIterator<MapT> ray(map, ray_origin_M, ray_dir_M, t_near, t_far);
  ray.next();

  const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
  if (t_min <= 0.f) {
    return Eigen::Vector4f::Zero();
  }
  const float t_max = ray.tmax();

//  float t = t_near;
  float t = t_min;

  if (t_near < t_max) {
    // first walk with largesteps until we found a hit
    float stepsize = largestep;
    Eigen::Vector3f position = ray_origin_M + ray_dir_M * t;
    typename MapT::DataType data;
    float field_value;
    map.template getData<se::Safe::On>(position, data);
    float f_t = data.tsdf;
    float f_tt = 0;
    if (f_t >= 0)
    { // ups, if we were already in it, then don't render anything here
      for (; t < t_far; t += stepsize)
      {
        if (!map.template getData<se::Safe::On>(position, data))
        {
          stepsize = largestep;
          position += stepsize * ray_dir_M;
          continue;
        }

        f_tt = data.tsdf;
        if(f_tt <= 0.15 && f_tt >= -1.5f)
        {
          if (map.template interpField(position, field_value))
          {
            f_tt = field_value;
          }
        }

        if (f_tt < 0)
        {
          break;
        } // got it, jump out of inner loop

        stepsize  = fmaxf(f_tt * MU, 0.04);
        position += stepsize * ray_dir_M;
        f_t = f_tt;
      }
      if (f_tt < 0)
      {           // got it, calculate accurate intersection
        t = t + stepsize * f_tt / (f_t - f_tt);
        Eigen::Vector4f res = (ray_origin_M + ray_dir_M * t).homogeneous();
        res.w() = 0;
        return res;
      }
    }
  }
  return Eigen::Vector4f::Constant(-1);
}


template<typename MapT, typename SensorT>
void raycastVolume(MapT&                       map,
                   se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                   se::Image<Eigen::Vector3f>& surface_normals_M,
                   const Eigen::Matrix4f&      T_MS,
                   const SensorT&              sensor)
{
TICK("raycast-volume")
#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (int y = 0; y < surface_point_cloud_M.height(); y++)
  {
#pragma omp simd
    for (int x = 0; x < surface_point_cloud_M.width(); x++)
    {
      Eigen::Vector4f surface_intersection_M;

      const Eigen::Vector2i pixel(x, y);
      const Eigen::Vector2f pixel_f = pixel.cast<float>();
      Eigen::Vector3f ray_dir_C;
      sensor.model.backProject(pixel_f, &ray_dir_C);
      const Eigen::Vector3f ray_dir_M = (se::math::to_rotation(T_MS) * ray_dir_C.normalized()).head(3);
      const Eigen::Vector3f t_MS = se::math::to_translation(T_MS);
      const float resolution = map.getRes();
      surface_intersection_M = raycast(map, t_MS, ray_dir_M, sensor.nearDist(ray_dir_C), sensor.farDist(ray_dir_C), 0.1f, resolution, resolution);
      Eigen::Vector3f surface_intersection_W = surface_intersection_M.head(3) + Eigen::Vector3f::Constant(5.119999);
      if (surface_intersection_M.w() >= 0.f)
      {
        surface_point_cloud_M[x + y * surface_point_cloud_M.width()] = surface_intersection_M.head<3>();
        Eigen::Vector3f surface_normal;

        map.template gradField(surface_intersection_M.head(3), surface_normal);
        const Eigen::Vector3f surface_normal_normalized = surface_normal.normalized();

        if (surface_normal.norm() == 0.f) {
          surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
        } else
        {
          // Invert surface normals for TSDF representations.
          surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = (true)
                                                                                 ? (-1.f * surface_normal).normalized()
                                                                                 : surface_normal.normalized();
        }
      } else
      {
        surface_point_cloud_M[pixel.x() + pixel.y() * surface_point_cloud_M.width()] = Eigen::Vector3f::Zero();
        surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
      }
    }
  }
TOCK("raycast-volume")
}


void renderVolumeKernel(uint32_t*                         volume_RGBA_image_data,
                        const Eigen::Vector2i&            volume_RGBA_image_res,
                        const Eigen::Vector3f&            light_M,
                        const Eigen::Vector3f&            ambient_M,
                        const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                        const se::Image<Eigen::Vector3f>& surface_normals_M) {

  const int h = volume_RGBA_image_res.y(); // clang complains if this is inside the for loop
  const int w = volume_RGBA_image_res.x(); // clang complains if this is inside the for loop

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (int y = 0; y < h; y++) {
#pragma omp simd
    for (int x = 0; x < w; x++) {

      const size_t pixel_idx = x + w * y;

      const Eigen::Vector3f surface_point_M = surface_point_cloud_M[pixel_idx];
      const Eigen::Vector3f surface_normal_M = surface_normals_M[pixel_idx];

      if (surface_normal_M.x() != INVALID && surface_normal_M.norm() > 0.f) {
        const Eigen::Vector3f diff = (surface_point_M - light_M).normalized();
        const Eigen::Vector3f dir
                = Eigen::Vector3f::Constant(fmaxf(surface_normal_M.normalized().dot(diff), 0.f));
        Eigen::Vector3f col = dir + ambient_M;
        se::math::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones());
        col = col.cwiseProduct(se::raycaster::color_map[se::raycaster::scale_image(x, y)]);
        volume_RGBA_image_data[pixel_idx] = se::pack_rgba(col.x(), col.y(), col.z(), 0xFF);
      } else {
        volume_RGBA_image_data[pixel_idx] = 0xFF000000;
      }
    }
  }
}

} // namespace raycaster
} // namespace se

#endif // SE_RAYCASTER_HPP
