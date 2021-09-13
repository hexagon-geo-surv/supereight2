#ifndef SE_RAYCAST_CARVER_IMPL_HPP
#define SE_RAYCAST_CARVER_IMPL_HPP

namespace se {
namespace fetcher {



template<typename MapT,
         typename SensorT
>
inline std::vector<se::OctantBase*> frustum(MapT&                   map,
                                            const SensorT&          sensor,
                                            const Eigen::Matrix4f&  T_MS)
{
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);
  // Loop over all allocated Blocks.
  std::vector<se::OctantBase*> fetched_block_ptrs;

  for (auto block_ptr_itr = se::FrustumIterator<MapT, SensorT>(map, sensor, T_SM); block_ptr_itr != se::FrustumIterator<MapT, SensorT>(); ++block_ptr_itr)
  {
    fetched_block_ptrs.push_back(*block_ptr_itr);
  }

  return fetched_block_ptrs;
}



} // namespace fetcher

template<typename MapT,
         typename SensorT
>
RaycastCarver<MapT, SensorT>::RaycastCarver(MapT&                   map,
                                            const SensorT&          sensor,
                                            const se::Image<float>& depth_img,
                                            const Eigen::Matrix4f&  T_MS,
                                            const int               frame) :
    map_(map),
    octree_(*(map_.getOctree())),
    sensor_(sensor),
    depth_img_(depth_img),
    T_MS_(T_MS),
    frame_(frame),
    config_(map)
{
}



template<typename MapT,
         typename SensorT
>
std::vector<se::OctantBase*> RaycastCarver<MapT, SensorT>::operator()()
{
  TICK("fetch-frustum")
  // Fetch the currently allocated Blocks in the sensor frustum.
  // i.e. the fetched blocks might contain blocks outside the current valid sensor range.
  std::vector<se::OctantBase*> fetched_block_ptrs = se::fetcher::frustum(map_, sensor_, T_MS_);
  TOCK("fetch-frustum")

  TICK("create-list")
  typename MapT::OctreeType* octree_ptr = map_.getOctree().get();
  se::OctantBase*            root_ptr   = octree_ptr->getRoot();

  const int num_steps = ceil(config_.band / (2 * map_.getRes()));

  const Eigen::Vector3f t_MS = T_MS_.topRightCorner<3, 1>();

#pragma omp declare reduction (merge : std::set<se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
  std::set<se::key_t> voxel_key_set;

#pragma omp parallel for reduction(merge: voxel_key_set)
  for (int x = 0; x < depth_img_.width(); ++x)
  {
    for (int y = 0; y < depth_img_.height(); ++y)
    {
      const Eigen::Vector2i pixel(x, y);
      const float depth_value = depth_img_(pixel.x(), pixel.y());
      // Only consider depth values inside the valid sensor range
      if (depth_value < sensor_.near_plane || depth_value > (sensor_.far_plane + config_.band * 0.5f))
      {
        continue;
      }

      Eigen::Vector3f ray_dir_C;
      const Eigen::Vector2f pixel_f = pixel.cast<float>();
      sensor_.model.backProject(pixel_f, &ray_dir_C);
      const Eigen::Vector3f point_M = (T_MS_ * (depth_value * ray_dir_C).homogeneous()).head<3>();

      const Eigen::Vector3f reverse_ray_dir_M = (t_MS - point_M).normalized();

      const Eigen::Vector3f ray_origin_M = point_M - (config_.band * 0.5f) * reverse_ray_dir_M;
      const Eigen::Vector3f step = (reverse_ray_dir_M * config_.band) / num_steps;

      Eigen::Vector3f ray_pos_M = ray_origin_M;
      for (int i = 0; i < num_steps; i++)
      {
        Eigen::Vector3i voxel_coord;

        if (map_.template pointToVoxel<se::Safe::On>(ray_pos_M, voxel_coord))
        {
          const se::OctantBase* octant_ptr = se::fetcher::block<typename MapT::OctreeType>(voxel_coord, root_ptr);
          if (octant_ptr == nullptr)
          {
            se::key_t voxel_key;
            se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
            voxel_key_set.insert(voxel_key);
          }
        }
        ray_pos_M += step;
      }
    }
  }
  // Allocate the Blocks and get pointers only to the newly-allocated Blocks.
  std::vector<key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());
  TOCK("create-list")

  TICK("allocate-list")
  std::vector<se::OctantBase*> allocated_block_ptrs = se::allocator::blocks(voxel_keys, *octree_ptr, octree_ptr->getRoot(), true);
  TOCK("allocate-list")

  TICK("combine-vectors")
  // Merge the previously-allocated and newly-allocated Block pointers.
  allocated_block_ptrs.reserve(allocated_block_ptrs.size() + fetched_block_ptrs.size());
  allocated_block_ptrs.insert(allocated_block_ptrs.end(), fetched_block_ptrs.begin(), fetched_block_ptrs.end());
  TOCK("combine-vectors")
  return allocated_block_ptrs;
}







} // namespace se

#endif // SE_RAYCAST_CARVER_IMPL_HPP

