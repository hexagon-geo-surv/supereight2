#include "se/raycaster.hpp"



namespace se {
namespace raycaster{



void point_cloud_to_normal(se::Image<Eigen::Vector3f>&       normals,
                              const se::Image<Eigen::Vector3f>& point_cloud,
                              const bool                        /* is_lhc */)
{
  const int width = point_cloud.width();
  const int height = point_cloud.height();

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      const Eigen::Vector3f point = point_cloud[x + width * y];
      if (point.z() == 0.f)
      {
        normals[x + y * width].x() = INVALID;
        continue;
      }

      const Eigen::Vector2i p_left
              = Eigen::Vector2i(std::max(int(x) - 1, 0), y);
      const Eigen::Vector2i p_right
              = Eigen::Vector2i(std::min(x + 1, (int) width - 1), y);

      // Swapped to match the left-handed coordinate system of ICL-NUIM
      Eigen::Vector2i p_up, p_down;
      if (false)
      {
        p_up = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
        p_down = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
      } else
      {
        p_down = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
        p_up = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
      }

      const Eigen::Vector3f left = point_cloud[p_left.x() + width * p_left.y()];
      const Eigen::Vector3f right = point_cloud[p_right.x() + width * p_right.y()];
      const Eigen::Vector3f up = point_cloud[p_up.x() + width * p_up.y()];
      const Eigen::Vector3f down = point_cloud[p_down.x() + width * p_down.y()];

      if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0)
      {
        normals[x + y * width].x() = INVALID;
        continue;
      }
      const Eigen::Vector3f dv_x = right - left;
      const Eigen::Vector3f dv_y = up - down;
      normals[x + y * width] = dv_x.cross(dv_y).normalized();
    } // x
  } // y
}



void renderVolumeKernel(uint32_t*                         volume_RGBA_image_data,
                        const Eigen::Vector2i&            volume_RGBA_image_res,
                        const Eigen::Vector3f&            light_M,
                        const Eigen::Vector3f&            ambient_M,
                        const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                        const se::Image<Eigen::Vector3f>& surface_normals_M)
{

  const int h = volume_RGBA_image_res.y(); // clang complains if this is inside the for loop
  const int w = volume_RGBA_image_res.x(); // clang complains if this is inside the for loop

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (int y = 0; y < h; y++)
  {
#pragma omp simd
    for (int x = 0; x < w; x++)
    {

      const size_t pixel_idx = x + w * y;

      const Eigen::Vector3f surface_point_M = surface_point_cloud_M[pixel_idx];
      const Eigen::Vector3f surface_normal_M = surface_normals_M[pixel_idx];

      if (surface_normal_M.x() != INVALID && surface_normal_M.norm() > 0.f)
      {
        const Eigen::Vector3f diff = (surface_point_M - light_M).normalized();
        const Eigen::Vector3f dir
                = Eigen::Vector3f::Constant(fmaxf(surface_normal_M.normalized().dot(diff), 0.f));
        Eigen::Vector3f col = dir + ambient_M;
        se::math::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones());
        col = col.cwiseProduct(se::raycaster::color_map[se::raycaster::scale_image(x, y)]);
        volume_RGBA_image_data[pixel_idx] = se::pack_rgba(col.x(), col.y(), col.z(), 0xFF);
      } else
      {
        volume_RGBA_image_data[pixel_idx] = 0xFF000000;
      }
    } // x
  } // y
}

} // namespace raycaster
} // namespace se
