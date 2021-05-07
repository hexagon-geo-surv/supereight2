#ifndef SE_RAYCASTER_IMPL_HPP
#define SE_RAYCASTER_IMPL_HPP



namespace se {
namespace raycaster {



template <typename MapT>
inline Eigen::Vector4f raycast(MapT&                  map,
                               const Eigen::Vector3f& ray_origin_M,
                               const Eigen::Vector3f& ray_dir_M,
                               const float            t_near,
                               const float            t_far)
{

  se::VoxelBlockRayIterator<MapT> ray(map, ray_origin_M, ray_dir_M, t_near, t_far);
  ray.next();

  float step      = map.getRes();
  float largestep = MapT::OctreeType::BlockType::getSize() * step;

  const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
  if (t_min <= 0.f) {
    return Eigen::Vector4f::Zero();
  }
  const float t_max = ray.tmax();

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
        if(f_tt <= 0.1 && f_tt >= -0.5f)
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

        stepsize  = fmaxf(f_tt * map.getDataConfig().truncation_boundary, step);
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
  TICK("surface-point-cloud")
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
      surface_intersection_M = raycast(map, t_MS, ray_dir_M, sensor.nearDist(ray_dir_C), sensor.farDist(ray_dir_C));

      if (surface_intersection_M.w() >= 0.f)
      {
        surface_point_cloud_M[x + y * surface_point_cloud_M.width()] = surface_intersection_M.head<3>();
      } else
      {
        surface_point_cloud_M[pixel.x() + pixel.y() * surface_point_cloud_M.width()] = Eigen::Vector3f::Zero();
      }

    }
  }
  TOCK("surface-point-cloud")

  TICK("surface-normals")
#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (int y = 0; y < surface_point_cloud_M.height(); y++)
  {
#pragma omp simd
    for (int x = 0; x < surface_point_cloud_M.width(); x++)
    {

      const Eigen::Vector2i pixel(x, y);
      Eigen::Vector3f surface_intersection_M = surface_point_cloud_M[x + y * surface_point_cloud_M.width()];

      if (surface_intersection_M != Eigen::Vector3f::Zero())
      {
        Eigen::Vector3f surface_normal;

        map.template gradField(surface_intersection_M.head(3), surface_normal);

        if (surface_normal.norm() == 0.f)
        {
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
        surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
      }
    }
  }
  TOCK("surface-normals")
  TOCK("raycast-volume")
}



} // namespace raycaster
} // namespace se

#endif // SE_RAYCASTER_IMPL_HPP
