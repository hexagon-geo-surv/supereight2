#ifndef SE_ALLOCATOR_IMPL_HPP
#define SE_ALLOCATOR_IMPL_HPP

#include <parallel/algorithm>

#include "se/timings.hpp"
#include "se/image/image.hpp"



namespace se {
namespace allocator {

template <typename OctreeT>
inline typename OctreeT::BlockType* block(const Eigen::Vector3i&      voxel_coord,
                                          std::shared_ptr<OctreeT>    octree_ptr,
                                          typename OctreeT::NodeType* base_parent_ptr)
{
  se::key_t voxel_key;
  se::keyops::encode_key(voxel_coord, 0, voxel_key); // Allocate up to finest scale

  return block(voxel_key, octree_ptr, base_parent_ptr);
}



template <typename OctreeT>
inline typename OctreeT::BlockType* block(const se::key_t             voxel_key,
                                          std::shared_ptr<OctreeT>    octree_ptr,
                                          typename OctreeT::NodeType* base_parent_ptr)
{
  assert(octree_ptr); // Verify octree ptr
  assert(base_parent_ptr); // Verify parent ptr
//  assert(se::keyops::is_child(base_parent_ptr->getKey(), voxel_key)); // TODO:
  assert(se::keyops::key_to_scale(voxel_key) <= octree_ptr->max_block_scale); // Verify scale is within block

  se::OctantBase* child_ptr = se::allocator::allocate_key(voxel_key, octree_ptr, base_parent_ptr);
  return static_cast<typename OctreeT::BlockType*>(child_ptr);
}



template <typename OctreeT>
inline se::vector<typename OctreeT::BlockType*> blocks(const se::vector<Eigen::Vector3i>& voxel_coords,
                                                       std::shared_ptr<OctreeT>           octree_ptr,
                                                       typename OctreeT::NodeType*        base_parent_ptr)
{
  se::set<se::key_t> voxel_key_set;

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp declare reduction (merge : std::set<se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge: voxel_key_set)
  for (unsigned int i = 0; i < voxel_coords.size(); i++)
  {
    const Eigen::Vector3i voxel_coord = voxel_coords[i];
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
    voxel_key_set.insert(voxel_key);
  }

  se::vector<se::key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());

  return blocks(voxel_keys, octree_ptr, base_parent_ptr);
}



template <typename OctreeT>
inline se::vector<typename OctreeT::BlockType*> blocks(se::vector<se::key_t>       unique_voxel_keys,
                                                       std::shared_ptr<OctreeT>    octree_ptr,
                                                       typename OctreeT::NodeType* base_parent_ptr)
{
  assert(octree_ptr);      // Verify octree ptr
  assert(base_parent_ptr); // Verify parent ptr

  // Allocate nodes up to block_scale
  for (scale_t scale = octree_ptr->getMaxScale(); scale > octree_ptr->max_block_scale; scale--)
  {
    se::vector<se::key_t> unique_voxel_keys_at_scale;
    se::keyops::unique_at_scale(unique_voxel_keys, scale, unique_voxel_keys_at_scale);

#ifdef _OPENMP
    omp_set_num_threads(10);
#endif
#pragma omp parallel for
    for (unsigned int i = 0; i < unique_voxel_keys_at_scale.size(); i++)
    {
      const auto unique_voxel_key_at_scale = unique_voxel_keys_at_scale[i];
      se::allocator::allocate_key(unique_voxel_key_at_scale, octree_ptr, base_parent_ptr);
    }
  }

  // Allocate blocks and store block pointers
  se::vector<typename OctreeT::BlockType*> block_ptrs;
#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for
  for (unsigned int i = 0; i < unique_voxel_keys.size(); i++)
  {

    const auto unique_voxel_key = unique_voxel_keys[i];
    assert(se::keyops::key_to_scale(unique_voxel_key) <= octree_ptr->max_block_scale); // Verify scale is within block

    se::OctantBase* child_ptr = se::allocator::allocate_key(unique_voxel_key, octree_ptr, base_parent_ptr);

#pragma omp critical
    {
      block_ptrs.push_back(static_cast<typename OctreeT::BlockType*>(child_ptr));
    }
  }

  return block_ptrs;
}



namespace { // anonymous namespace

template <typename OctreeT>
inline se::OctantBase* allocate_key(const se::key_t             key,
                                    std::shared_ptr<OctreeT>    octree_ptr,
                                    typename OctreeT::NodeType* base_parent_ptr)
{
  assert(se::keyops::is_valid(key)); // Verify if the key is valid
  typename OctreeT::NodeType* parent_ptr = base_parent_ptr;
  se::OctantBase*             child_ptr  = nullptr;

  int child_scale = se::math::log2_const(parent_ptr->getSize()) - 1;
  se::code_t code = se::keyops::key_to_code(key);
  int min_scale   = std::max(se::keyops::key_to_scale(key), octree_ptr->max_block_scale);

  for (; child_scale >= min_scale; --child_scale)
  {
    se::idx_t child_idx = se::keyops::code_to_child_idx(code, child_scale);
    child_ptr = octree_ptr->allocate(parent_ptr, child_idx);
    parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
  }
  return child_ptr;
}
} // anonymous namespace

} // namespace allocator
} // namespace se

#endif //SE_TRYOUT_ALLOCATOR_IMPL_HPP
