#ifndef SE_RAYCASTER_IMPL_HPP
#define SE_RAYCASTER_IMPL_HPP



#include "se/map/octree/visitor.hpp"



namespace se {
namespace raycaster {

/**
 * \brief Finds the first valid point along a ray starting from (ray_origin_M + t * ray_dir_M). Returns false if no
 * valid point can be found before the maximum travelled distance is reached.
 *
 * \param[in]     map                Reference to the octree
 * \param[in]     select_node_value  lambda function selecting the node value to be interpolated
 * \param[in]     select_voxel_value lambda function selecting the voxel value to be interpolated
 * \param[in]     ray_origin_M       Origin of the ray in map frame [m]
 * \param[in]     ray_dir_M          Direction of the ray in map frame [m]
 * \param[in]     step_size          Size of a step per iteration [m]
 * \param[in]     t_max              Maximum travelled distance along the ray [m]
 * \param[in,out] t                  Travelled distance along the ray [m]
 * \param[out]    f                  Interpolated value at ray_origin_M + t * ray_dir_M
 * \param[out]    p                  First valid point along the ray starting from
 *                                   the input t (ray_origin_M + t * ray_dir_M)
 * \return        is_valid           True if valid point could be found before reaching t_max. False otherwise.
 */
template <typename MapT>
inline std::optional<se::field_t> find_valid_point(const MapT&             map,
                                                   const Eigen::Vector3f&  ray_origin_M,
                                                   const Eigen::Vector3f&  ray_dir_M,
                                                   const float             step_size,
                                                   const float             t_max,
                                                   float&                  t,
                                                   Eigen::Vector3f&        point_M)
{
  std::optional<se::field_t> value = {};
  typename MapT::OctreeType::DataType peek_data;

  Eigen::Vector3f ray_pos_M = ray_origin_M + t * ray_dir_M;
  if (map.contains(ray_pos_M))
  {
    peek_data = map.getData(ray_pos_M);
    if (peek_data.weight > 0)
    {
      value = map.getFieldInterp(ray_pos_M);
    }
  }

  while (!value) {
    t += step_size;
    if (t > t_max)
    {
      return value;
    }

    ray_pos_M = ray_origin_M + t * ray_dir_M;
    if (map.contains(ray_pos_M))
    {
      peek_data = map.getData(ray_pos_M);
      if (peek_data.weight > 0)
      {
        value = map.getFieldInterp(ray_pos_M);
      }
    }
  }

  point_M = ray_pos_M;
  return value;
}



/**
 * \brief Compute the distance t in [m] travelled along the ray from the origin until the ray intersecs with the map.
 * Can be summariesed in 3 cases:
 * 1. Origin is inside the map                                      -> Return 0/valid
 * 2. Origin is outside the map and ray intersects the map          -> Return t/valid
 * 3. Origin is outside the map and ray will not intersect the map  -> Return 0/invalid
 *
 * \param[in]  ray_pos_M  current camera position in [m]
 * \param[in]  ray_dir_M ray_dir_M of the ray
 * \param[in]  map_dim   map dimension in [m]
 * \param[in]  t_far     maximum travel distance along the ray in [m]  * \param[out] is_valid  flag if the map intersection is valid
 * \return see above
 */
inline float compute_map_intersection(const Eigen::Vector3f& ray_pos_M,
                                      const Eigen::Vector3f& ray_dir_M,
                                      const Eigen::Vector3f& map_dim,
                                      const Eigen::Vector3f& map_origin,
                                      const float            t_far,
                                      bool&                  is_valid)
{
  /*
  Fast Ray-Box Intersection
  by Andrew Woo
  from "Graphics Gems", Academic Press, 1990
  */
  const Eigen::Vector3f map_min = -map_origin;
  const Eigen::Vector3f map_max = map_dim - map_origin;
  constexpr int num_dim = 3;
  Eigen::Vector3f hit_point = -1 * Eigen::Vector3f::Ones();				/* hit point */
  bool inside = true;
  Eigen::Vector3i quadrant;
  int which_plane;
  Eigen::Vector3f max_T;
  Eigen::Vector3f candidate_plane;

  /* Find candidate planes; this loop can be avoided if
     rays cast all from the eye(assume perpsective view) */
  for (int i = 0; i < num_dim; i++)
  {
    if (ray_pos_M[i] < map_min[i])
    {
      quadrant[i] = 1; // LEFT := 1
      candidate_plane[i] = map_min[i];
      inside = false;
    } else if (ray_pos_M[i] > map_max[i])
    {
      quadrant[i] = 0; // RIGHT := 0
      candidate_plane[i] = map_max[i];
      inside = false;
    } else
    {
      quadrant[i] = 2; // MIDDLE := 2
    }
  }

  /* Ray origin inside bounding box */
  if (inside)
  {
    return 0;
  }

  /* Calculate T distances to candidate planes */
  for (int i = 0; i < num_dim; i++)
  {
    if (quadrant[i] != 2 && ray_dir_M[i] !=0.) // MIDDLE := 2
    {
      max_T[i] = (candidate_plane[i] - ray_pos_M[i]) / ray_dir_M[i];
    }
    else
    {
      max_T[i] = -1.;
    }
  }

  /* Get largest of the max_T's for final choice of intersection */
  which_plane = 0;
  for (int i = 1; i < num_dim; i++)
  {
    if (max_T[which_plane] < max_T[i])
    {
      which_plane = i;
    }
  }

  /* Check final candidate actually inside box */
  if (max_T[which_plane] < 0.f)
  {
    is_valid = false;
    return 0;
  }

  for (int i = 0; i < num_dim; i++)
  {
    if (which_plane != i)
    {
      hit_point[i] = ray_pos_M[i] + max_T[which_plane] * ray_dir_M[i];
      if (hit_point[i] < map_min[i] || hit_point[i] > map_max[i])
      {
        is_valid = false;
        return 0;
      }
    } else
    {
      hit_point[i] = candidate_plane[i];
    }
  }

  float t = (hit_point - ray_pos_M).norm();
  if (t_far < t)
  {
    is_valid = false;
    return 0;
  }
  return t;
}

/**
 * \brief Advance ray from the camera position until a voxel block with a max occupancy
 * log-odd value of at least -0.2 is reached
 * \param[in]     map           Map
 * \param[in]     ray_origin_M    Position of the camera
 * \param[in/out] t             Distance until the search voxel block is reached
 * \param[in/out] t_far         Distance to far plane or until the map is surpassed
 * \param[in]     voxel_dim     Resolution of the map
 * \param[in]     max_scale     Finest scale at which to check the occupancy
 * \param[out]    is_valid      Indiactes if a voxel block was found
 * \return        Surface intersection point in [m] and scale
 */
template <typename MapT>
inline void advance_ray(const MapT&            map,
                        const Eigen::Vector3f& ray_origin_M,
                        const Eigen::Vector3f& ray_dir_M,
                        float&                 t,
                        float&                 t_far,
                        const float            voxel_dim,
                        const int              max_scale,
                        bool&                  is_valid)
{
  int scale = max_scale;  // Initialize scale
  // Additional distance travelled in [voxel]
  float v_add   = 0;                     // TODO: I'll have to re-evaluate this value.
  const float v = 1 / voxel_dim * t;       // t in voxel coordinates
  //float v_near  = 1 / voxel_dim * t_near;  // t_near in voxel coordinates
  float v_far   = 1 / voxel_dim * t_far;   // t_far in voxel coordinates
  Eigen::Vector3f ray_origin_coord_f;
  map.pointToVoxel(ray_origin_M, ray_origin_coord_f); // Origin in voxel coordinates

  // Current state of V in [voxel]
  Eigen::Vector3f V_max = Eigen::Vector3f::Ones();


  Eigen::Vector3f delta_V_map = map.getOctree()->getSize() / ray_dir_M.array().abs(); // [voxel]/[-], potentionally dividing by 0

  Eigen::Vector3f map_frac = ray_origin_coord_f / map.getOctree()->getSize();
  // V at which the map boundary gets crossed (separate V for each dimension x-y-z)
  Eigen::Vector3f v_map;
  if (ray_dir_M.x() < 0)
  {
    v_map.x() = map_frac.x() * delta_V_map.x();
  } else
  {
    v_map.x() = (1 - map_frac.x()) * delta_V_map.x();
  }

  if (ray_dir_M.y() < 0)
  {
    v_map.y() = map_frac.y() * delta_V_map.y();
  } else
  {
    v_map.y() = (1 - map_frac.y()) * delta_V_map.y();
  }

  if (ray_dir_M.z() < 0)
  {
    v_map.z() = map_frac.z() * delta_V_map.z();
  } else {
    v_map.z() = (1 - map_frac.z()) * delta_V_map.z();
  }

  // Maximum valid travelled distance in voxel is the minimum out of the far plane,
  // and the smallest distance that will make the ray cross the map boundary in either x, y or z ray_dir_M.
  v_far = std::min(std::min(std::min(v_map.x(), v_map.y()), v_map.z()) + v, v_far); // [voxel]
  t_far = voxel_dim * v_far;                                                          // [m]

  typename MapT::OctreeType::DataType data = se::visitor::getMaxData(*(map.getOctree()), ray_origin_coord_f.cast<int>(), max_scale);

  while (data.occupancy * data.weight > -0.2f && scale > 2)
  { // TODO Verify
    scale -= 1;
    data = se::visitor::getMaxData(*(map.getOctree()), ray_origin_coord_f.cast<int>(), max_scale);
  }

  Eigen::Vector3f ray_coord_f = ray_origin_coord_f;

  while ((v + v_add) < v_far)
  {
    if (scale <= 2)
    {
      t = voxel_dim * (v + v_add - 4);
      return;
    }

    const int node_size = 1 << scale;
    Eigen::Vector3i curr_node = node_size * (((ray_coord_f).array().floor()) / node_size).cast<int>();

    // Fraction of the current position in [voxel] in the current node along the x-, y- and z-axis
    Eigen::Vector3f node_frac = (ray_coord_f - curr_node.cast<float>()) / node_size;

    // Travelled distance needed in [voxel] to the whole node_size in x, y and z ray_dir_M
    Eigen::Vector3f delta_V = node_size / ray_dir_M.array().abs(); // [voxel]/[-]

    // Initalize V
    if (ray_dir_M.x() < 0)
    {
      V_max.x() = node_frac.x() * delta_V.x();
    } else
    {
      V_max.x() = (1 - node_frac.x()) * delta_V.x();
    }

    if (ray_dir_M.y() < 0) {
      V_max.y() = node_frac.y() * delta_V.y();
    } else
    {
      V_max.y() = (1 - node_frac.y()) * delta_V.y();
    }

    if (ray_dir_M.z() < 0)
    {
      V_max.z() = node_frac.z() * delta_V.z();
    } else
    {
      V_max.z() = (1 - node_frac.z()) * delta_V.z();
    }

    const float zero_depth_band = 1.0e-6f;
    for (int i = 0; i < 3; i++)
    {
      if (std::fabs(ray_dir_M[i]) < zero_depth_band)
      {
        V_max[i] = std::numeric_limits<float>::infinity();
      }
    }

    float V_min = std::min(std::min(V_max.x(), V_max.y()), V_max.z());

    v_add += V_min + 0.01;
    ray_coord_f = (v + v_add) * ray_dir_M + ray_origin_coord_f;

    data = se::visitor::getMaxData(*(map.getOctree()), ray_coord_f.cast<int>(), scale);

    if (data.occupancy * data.weight > -0.2f)
    {
      while (data.occupancy * data.weight > -0.2f && scale > 2)
      {
        scale -= 1;
        data = se::visitor::getMaxData(*(map.getOctree()), ray_coord_f.cast<int>(), scale);
      }
    } else
    {
      for (int s = scale + 1; s <= max_scale; s++)
      {
        data = se::visitor::getMaxData(*(map.getOctree()), ray_coord_f.cast<int>(), s);

        if (data.occupancy * data.weight > -0.2f)
        {
          break;
        }
        scale += 1;
      }
    }
  }

  is_valid = false;
  return;
}



/**
 * \brief Compute the intersection point and scale for a given ray
 * \param map           Continuous map wrapper
 * \param ray_origin_M  Camera position in [m]
 * \param ray_dir_M     Direction of the ray
 * \param p_near        Near plane distance in [m]
 * \param p_far         Far plane distance in [m]
 * \return              Surface intersection point in [m] and scale
 */
template <typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::Occupancy, std::optional<Eigen::Vector4f>>
raycast(MapT&                  map,
        const Eigen::Vector3f& ray_origin_M,
        const Eigen::Vector3f& ray_dir_M,
        float                  /* t_near */,
        float                  t_far)
{
  typedef typename MapT::OctreeType::DataType DataType;

  const float voxel_dim = map.getRes(); // voxel_dim   := [m / voxel];
  // inv_voxel_dim := [m] to [voxel]; voxel_dim := [voxel] to [m]
  //float t_near = near_plane;                       // max travel distance in [m]

  // Check if the ray origin is outside the map.
  // If so, compute the first point of contact with the map.
  // Stop if no intersection will occur (i.e. is_valid = false).
  bool is_valid = true;
  float t = compute_map_intersection(ray_origin_M, ray_dir_M, map.getDim(), map.getOrigin(), t_far, is_valid);

  if (!is_valid)
  {
    // Ray won't intersect with the map
    return {};
  }

  const int max_scale = std::min(7, map.getOctree()->getMaxScale() - 1); // Max possible free space skipped per iteration (node size = 2^max_scale)

  advance_ray(map, ray_origin_M, ray_dir_M, t, t_far, voxel_dim, max_scale, is_valid);

  if (!is_valid)
  {
    // Ray passes only through free space or intersects with the map before t_near or after t_far.
    return {};
  }

  Eigen::Vector3f ray_pos_M = {};

  // first walk with largesteps until we found a hit
  float step_size = voxel_dim / 2;
  std::optional<se::field_t> value_t;
  std::optional<se::field_t> value_tt;
  Eigen::Vector3f point_M_t  = Eigen::Vector3f::Zero();
  Eigen::Vector3f point_M_tt = Eigen::Vector3f::Zero();
  int scale_tt = 0;

  value_t = find_valid_point(map, ray_origin_M, ray_dir_M, step_size, t_far, t, point_M_t);
  if (!value_t)
  {
    return {};
  }
  t += step_size;

  // if we are not already in it
  if (*value_t <= 0) // MultiresOFusion::surface_boundary
  {
    for (; t < t_far; t += step_size)
    {
      ray_pos_M = ray_origin_M + ray_dir_M * t;
      DataType data = map.getData(ray_pos_M);
      if (data.weight == 0)
      {
        t += step_size;
        value_t = find_valid_point(map, ray_origin_M, ray_dir_M, step_size, t_far, t, point_M_t);
        if (!value_t)
        {
          return Eigen::Vector4f::Zero();
        }

        if (*value_t > 0) // MultiresOFusion::surface_boundary
        {
          break;
        }
        continue;
      }
      value_tt = std::optional<se::field_t>(data.occupancy);
      point_M_tt = ray_pos_M;
      if (*value_tt > -0.2f)
      {
        value_tt = map.getFieldInterp(ray_pos_M, scale_tt);
        if (!value_tt)
        {
          t += step_size;
          value_t = find_valid_point(map, ray_origin_M, ray_dir_M, step_size, t_far, t, point_M_t);
          if (!value_t)
          {
            return {};
          }
          if (*value_t > 0) // MultiresOFusion::surface_boundary
          {
            break;
          }
          continue;
        }
      }
      if (*value_tt > 0.f) // MultiresOFusion::surface_boundary
      {                // got it, jump out of inner loop
        break;
      }
      value_t = value_tt;
      point_M_t = point_M_tt;
    }
    if (*value_tt > 0.f && *value_t < 0.f) // MultiresOFusion::surface_boundary
    {
      // We overshot. Need to move backwards for zero crossing.
      t = t - (point_M_tt - point_M_t).norm() * (*value_tt - 0.f) / (*value_tt - *value_t); // MultiresOFusion::surface_boundary
      Eigen::Vector4f surface_point_M = (ray_origin_M + ray_dir_M * t).homogeneous();
      surface_point_M.w() = scale_tt;
      return surface_point_M;
    }
  }
  return {};
}



template <typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::TSDF, std::optional<Eigen::Vector4f>>
raycast(MapT&                  map,
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
    return {};
  }
  const float t_max = ray.tmax();

  float t = t_min;

  const float truncation_boundary = map.getRes() * map.getDataConfig().truncation_boundary_factor;

  if (t_near < t_max) {
    // first walk with largesteps until we found a hit
    float stepsize = largestep;
    Eigen::Vector3f position = ray_origin_M + ray_dir_M * t;
    typename MapT::DataType data = map.template getData<se::Safe::On>(position);
    float f_t = data.tsdf;
    float f_tt = 0;
    int scale_tt = 0;
    if (f_t >= 0)
    { // ups, if we were already in it, then don't render anything here
      for (; t < t_far; t += stepsize)
      {
        data = map.template getData<se::Safe::On>(position);
        if (se::is_invalid(data))
        {
          stepsize = largestep;
          position += stepsize * ray_dir_M;
          continue;
        }

        f_tt = data.tsdf;
        if(f_tt <= 0.1 && f_tt >= -0.5f)
        {
          std::optional<se::field_t> field_value =
                  [&]()->std::optional<se::field_t>{ if constexpr (MapT::res_ == se::Res::Single) { return map.template getFieldInterp(position); }
                                                     else                                         { return map.template getFieldInterp(position, scale_tt); }}();
          if (field_value)
          {
            f_tt = *field_value;
          }
        }

        if (f_tt < 0)
        {
          break;
        } // got it, jump out of inner loop

        stepsize  = std::max(f_tt * truncation_boundary, step);
        position += stepsize * ray_dir_M;
        f_t = f_tt;
      }
      if (f_tt < 0)
      {           // got it, calculate accurate intersection
        t = t + stepsize * f_tt / (f_t - f_tt);
        Eigen::Vector4f intersection_M = (ray_origin_M + ray_dir_M * t).homogeneous();
        intersection_M.w() = scale_tt;
        return intersection_M;
      }
    }
  }
  return {};
}



template<typename MapT, typename SensorT>
void raycastVolume(const MapT&                 map,
                   se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                   se::Image<Eigen::Vector3f>& surface_normals_M,
                   se::Image<int8_t>&          surface_scale,
                   const Eigen::Matrix4f&      T_MS,
                   const SensorT&              sensor)
{
#pragma omp parallel for
  for (int y = 0; y < surface_point_cloud_M.height(); y++)
  {
#pragma omp simd
    for (int x = 0; x < surface_point_cloud_M.width(); x++)
    {
      const Eigen::Vector2i pixel(x, y);
      const Eigen::Vector2f pixel_f = pixel.cast<float>();
      Eigen::Vector3f ray_dir_C;
      sensor.model.backProject(pixel_f, &ray_dir_C);
      const Eigen::Vector3f ray_dir_M = (se::math::to_rotation(T_MS) * ray_dir_C.normalized()).head(3);
      const Eigen::Vector3f t_MS = se::math::to_translation(T_MS);
      std::optional<Eigen::Vector4f> surface_intersection_M = raycast(map, t_MS, ray_dir_M, sensor.nearDist(ray_dir_C), sensor.farDist(ray_dir_C));

      if (surface_intersection_M)
      {
        surface_scale(x, y) = static_cast<int>((*surface_intersection_M).w());
        surface_point_cloud_M[x + y * surface_point_cloud_M.width()] = (*surface_intersection_M).head<3>();
      } else
      {
        surface_point_cloud_M[pixel.x() + pixel.y() * surface_point_cloud_M.width()] = Eigen::Vector3f::Zero();
      }

    }
  }

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
        auto surface_normal = map.template getFieldGrad(surface_intersection_M.head(3));
        if (!surface_normal)
        {
          surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
        } else
        {
          // Invert surface normals for TSDF representations.
          surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = (MapT::DataType::invert_normals)
                                                                                 ? (-1.f * (*surface_normal)).normalized()
                                                                                 : (*surface_normal).normalized();
        }
      } else
      {
        surface_normals_M[pixel.x() + pixel.y() * surface_normals_M.width()] = Eigen::Vector3f(INVALID, 0.f, 0.f);
      }
    }
  }
}



} // namespace raycaster
} // namespace se

#endif // SE_RAYCASTER_IMPL_HPP

