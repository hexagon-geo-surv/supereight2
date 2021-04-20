#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include <atomic>
#include <set>

#include "se/utils/setup_util.hpp"
#include "se/utils/type_util.hpp"
#include "se/image/image.hpp"
#include "se/timings.hpp"
#include "se/allocator.hpp"


namespace se {
namespace fetcher {



//template<typename SensorT, typename MapT>
//std::vector<typename MapT::OctreeType::BlockType*> frustum(const se::Image<depth_t>&  depth_img,
//                                                               SensorT&               sensor,
//                                                               const Eigen::Matrix4f& T_MS,
//                                                               MapT&                  map)
//{
//  TICK("build-allocation-list")
//  typedef typename MapT::OctreeType::BlockType BlockType;
//  auto octree_ptr = map.octree_;
//
//  const float band = 2.f * 0.1f;
//  const int num_steps = ceil(band / map.getRes());
//
//  const Eigen::Vector3f t_MC = T_MS.topRightCorner<3, 1>();
//
//  int num_reserved = depth_img.width() * depth_img.height() * num_steps;
//#pragma omp declare reduction (merge : std::vector<se::key_t> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
//  se::vector<se::key_t> voxel_keys;
//  voxel_keys.reserve(num_reserved);
//
//  omp_set_num_threads(10);
//#pragma omp parallel for reduction(merge: voxel_keys)
//  for (int x = 0; x < depth_img.width(); ++x)
//  {
//    for (int y = 0; y < depth_img.height(); ++y)
//    {
//      const Eigen::Vector2i pixel(x, y);
//      const float depth_value = depth_img(pixel.x(), pixel.y());
//      if (depth_value < sensor.near_plane)
//      {
//        continue;
//      }
//
//      Eigen::Vector3f ray_dir_C;
//      const Eigen::Vector2f pixel_f = pixel.cast<float>();
//      sensor.model.backProject(pixel_f, &ray_dir_C);
//      const Eigen::Vector3f point_M = (T_MS * (depth_value * ray_dir_C).homogeneous()).head<3>();
//
//      const Eigen::Vector3f reverse_ray_dir_M = (t_MC - point_M).normalized();
//
//      const Eigen::Vector3f ray_origin_M = point_M - (band * 0.5f) * reverse_ray_dir_M;
//      const Eigen::Vector3f step = (reverse_ray_dir_M * band) / num_steps;
//
//      Eigen::Vector3f ray_pos_M = ray_origin_M;
//      for (int i = 0; i < num_steps; i++)
//      {
//        Eigen::Vector3i voxel_coord;
//
//        if (map. template pointToVoxel<se::Safe::On>(ray_pos_M, voxel_coord))
//        {
//          se::key_t voxel_key;
//          se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
//          voxel_keys.push_back(voxel_key);
//        }
//        ray_pos_M += step;
//      }
//    }
//  }
//  TOCK("build-allocation-list")
//
//  std::cout << voxel_keys.size() << std::endl;
//
//  TICK("allocate-frustum")
//  se::vector<BlockType*> fetched_block_ptrs = se::allocator::blocks(voxel_keys, octree_ptr, octree_ptr->getRoot());
//  TOCK("allocate-frustum")
//
//  return fetched_block_ptrs;
//}

template<typename SensorT, typename MapT>
std::vector<typename MapT::OctreeType::BlockType*> frustum(const se::Image<depth_t>&  depth_img,
                                                           SensorT&               sensor,
                                                           const Eigen::Matrix4f& T_MS,
                                                           MapT&                  map)
{
  TICK("build-allocation-list")
  typedef typename MapT::OctreeType::BlockType BlockType;
  auto octree_ptr = map.octree_;

  const float band = 2.f * 0.1f;
  const int num_steps = ceil(band / map.getRes());

  const Eigen::Vector3f t_MC = T_MS.topRightCorner<3, 1>();

#pragma omp declare reduction (merge : std::set<se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
  se::set<se::key_t> voxel_key_set;

  omp_set_num_threads(10);
#pragma omp parallel for reduction(merge: voxel_key_set)
  for (int x = 0; x < depth_img.width(); ++x)
  {
    for (int y = 0; y < depth_img.height(); ++y)
    {
      const Eigen::Vector2i pixel(x, y);
      const float depth_value = depth_img(pixel.x(), pixel.y());
      if (depth_value < sensor.near_plane)
      {
        continue;
      }

      Eigen::Vector3f ray_dir_C;
      const Eigen::Vector2f pixel_f = pixel.cast<float>();
      sensor.model.backProject(pixel_f, &ray_dir_C);
      const Eigen::Vector3f point_M = (T_MS * (depth_value * ray_dir_C).homogeneous()).head<3>();

      const Eigen::Vector3f reverse_ray_dir_M = (t_MC - point_M).normalized();

      const Eigen::Vector3f ray_origin_M = point_M - (band * 0.5f) * reverse_ray_dir_M;
      const Eigen::Vector3f step = (reverse_ray_dir_M * band) / num_steps;

      Eigen::Vector3f ray_pos_M = ray_origin_M;
      for (int i = 0; i < num_steps; i++)
      {
        Eigen::Vector3i voxel_coord;

        if (map. template pointToVoxel<se::Safe::On>(ray_pos_M, voxel_coord))
        {
          se::key_t voxel_key;
          se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
          voxel_key_set.insert(voxel_key);
        }
        ray_pos_M += step;
      }
    }
  }
  TOCK("build-allocation-list")

  se::vector<key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());

  TICK("allocate-frustum")
  se::vector<BlockType*> fetched_block_ptrs = se::allocator::blocks(voxel_keys, octree_ptr, octree_ptr->getRoot());
  TOCK("allocate-frustum")

  return fetched_block_ptrs;
}



} // namespace fetcher
} // namespace se



#endif // SE_FETCHER_HPP
