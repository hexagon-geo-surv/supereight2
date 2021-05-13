#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP



namespace se {
namespace allocator {

template<typename SensorT, typename MapT>
std::vector<typename MapT::OctreeType::BlockType *> frustum(const se::Image<depth_t>& depth_img,
                                                            SensorT&                  sensor,
                                                            const Eigen::Matrix4f&    T_MS,
                                                            MapT&                     map,
                                                            const float               band)
{
  typedef typename MapT::OctreeType::BlockType BlockType;
  auto octree_ptr = map.getOctree();

  const int num_steps = ceil(band / map.getRes());

  const Eigen::Vector3f t_MS = T_MS.topRightCorner<3, 1>();

#pragma omp declare reduction (merge : std::set<se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
  se::set<se::key_t> voxel_key_set;

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for reduction(merge: voxel_key_set)
  for (int x = 0; x < depth_img.width(); ++x) {
    for (int y = 0; y < depth_img.height(); ++y) {
      const Eigen::Vector2i pixel(x, y);
      const float depth_value = depth_img(pixel.x(), pixel.y());
      if (depth_value < sensor.near_plane) {
        continue;
      }

      Eigen::Vector3f ray_dir_C;
      const Eigen::Vector2f pixel_f = pixel.cast<float>();
      sensor.model.backProject(pixel_f, &ray_dir_C);
      const Eigen::Vector3f point_M = (T_MS * (depth_value * ray_dir_C).homogeneous()).head<3>();

      const Eigen::Vector3f reverse_ray_dir_M = (t_MS - point_M).normalized();

      const Eigen::Vector3f ray_origin_M = point_M - (band * 0.5f) * reverse_ray_dir_M;
      const Eigen::Vector3f step = (reverse_ray_dir_M * band) / num_steps;

      Eigen::Vector3f ray_pos_M = ray_origin_M;
      for (int i = 0; i < num_steps; i++) {
        Eigen::Vector3i voxel_coord;

        if (map.template pointToVoxel<se::Safe::On>(ray_pos_M, voxel_coord)) {
          se::key_t voxel_key;
          se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
          voxel_key_set.insert(voxel_key);
        }
        ray_pos_M += step;
      }
    }
  }

  se::vector<key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());

  se::vector<BlockType *> fetched_block_ptrs = se::allocator::blocks(voxel_keys, octree_ptr,
                                                                     octree_ptr->getRoot());

  return fetched_block_ptrs;
}

} // namespace allocator



static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i &octant_coord,
                                               const int octant_size,
                                               const Eigen::Vector3f &sample_offset_frac)
{
  return octant_coord.cast<float>() + sample_offset_frac * octant_size;
}



namespace {



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT,
        se::Res ResT
>
struct IntegrateDepthImplD
{

    template<typename SensorT,
            typename MapT,
            typename ConfigT
    >
    static void integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          MapT&                         map,
                          ConfigT&                      /* config */); // TODO:
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>
{
    template<typename SensorT,
            typename MapT,
            typename ConfigT
    >
    static void integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          MapT&                         map,
                          ConfigT&                      /* config */);
};



template <typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::ress_>;



template<se::Field FldT,
         se::Res ResT
>
template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<FldT, ResT>::integrate(const se::Image<se::depth_t>& depth_img,
                                                const SensorT&                sensor,
                                                const Eigen::Matrix4f&        T_MS,
                                                MapT&                         map,
                                                ConfigT&                      config)
{
}



template<typename SensorT, typename MapT, typename ConfigT>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                      const SensorT&                sensor,
                                                                      const Eigen::Matrix4f&        T_MS,
                                                                      MapT&                         map,
                                                                      ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getDataConfig().truncation_boundary;
  const se::weight_t max_weight   = map.getDataConfig().max_weight;

  // Allocation
  std::vector<typename MapT::OctreeType::BlockType *> block_ptrs = se::allocator::frustum(depth_img, sensor, T_MS, map, 2 * truncation_boundary);

  // Update
  unsigned int block_size = MapT::OctreeType::block_size;
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

  auto valid_predicate = [&](float depth_value) { return depth_value >= sensor.near_plane; };

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (unsigned int i = 0; i < block_ptrs.size(); i++)
  {
    auto block_ptr = block_ptrs[i];
    Eigen::Vector3i block_coord = block_ptr->getCoord();
    Eigen::Vector3f point_base_M;
    map.voxelToPoint(block_coord, point_base_M);
    const Eigen::Vector3f point_base_S = (T_SM * point_base_M.homogeneous()).head(3);
    const Eigen::Matrix3f point_delta_matrix_S = (se::math::to_rotation(T_SM) * map.getRes() *
                                                  Eigen::Matrix3f::Identity());

    for (unsigned int i = 0; i < block_size; ++i)
    {
      for (unsigned int j = 0; j < block_size; ++j)
      {
        for (unsigned int k = 0; k < block_size; ++k)
        {
          // Set voxel coordinates
          Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(i, j, k);

          // Set sample point in camera frame
          Eigen::Vector3f point_S = point_base_S + point_delta_matrix_S * Eigen::Vector3f(i, j, k);

          if (point_S.norm() > sensor.farDist(point_S))
          {
            continue;
          }

          // Fetch image value
          float depth_value(0);
          if (!sensor.projectToPixelValue(point_S, depth_img, depth_value, valid_predicate))
          {
            continue;
          }

          // Update the TSDF
          const float m = sensor.measurementFromPoint(point_S);
          const float sdf_value = (depth_value - m) / m * point_S.norm();
          if (sdf_value > -truncation_boundary)
          {
            const float tsdf_value = fminf(1.f, sdf_value / truncation_boundary);
            typename MapT::DataType data;
            block_ptr->getData(voxel_coord, data);
            data.tsdf = (data.tsdf * data.weight + tsdf_value) / (data.weight + 1.f);
            data.tsdf = se::math::clamp(data.tsdf, -1.f, 1.f);
            data.weight = fminf(data.weight + 1, max_weight);
            block_ptr->setData(voxel_coord, data);
          }

        } // k
      } // j
    } // i

  }
}



} // namespace anonymous



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT&                   map,
                                   const IntegratorConfig& config)
    : map_(map), config_(config)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const se::Image<se::depth_t>& depth_img,
                                         const SensorT&                sensor,
                                         const Eigen::Matrix4f&        T_MS)
{
  IntegrateDepthImpl<MapT>::integrate(depth_img, sensor, T_MS, map_, config_);
}



} // namespace se

#endif //SE_MAP_INTEGRATOR_IMPL_HPP
