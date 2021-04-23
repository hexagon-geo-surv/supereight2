#ifndef SE_ALLOCATOR_HPP
#define SE_ALLOCATOR_HPP

#include <memory>
#include <Eigen/Dense>

#include "se/utils/type_util.hpp"
#include "se/octree/octree.hpp"

/**
 * Helper wrapper to allocate and de-allocate octants in the octree.
 * The actual allocation and deallocation of memory is still only handled by the octree class.
 */
namespace se {
namespace allocator {

/**
 * \brief Allocate a block at the provided voxel coordinates.
 *
 * \tparam OctreeT              The Octree template
 * \param[in] voxel_coord       The 3D coordinates of a voxel within the block
 * \param[in] octree_ptr        The octree to allocate the block in
 * \param[in] base_parent_ptr   The starting node pointer (default nullptr will be replaced with octree root)
 *
 * \return A block pointer to the allocated block
 */
template <typename OctreeT>
typename OctreeT::BlockType* block(const Eigen::Vector3i&      voxel_coord,
                                   std::shared_ptr<OctreeT>    octree_ptr,
                                   typename OctreeT::NodeType* base_parent_ptr);

template <typename OctreeT>
typename OctreeT::BlockType* block(const se::key_t             voxel_key,
                                   std::shared_ptr<OctreeT>    octree_ptr,
                                   typename OctreeT::NodeType* base_parent_ptr);

template <typename OctreeT>
se::vector<typename OctreeT::BlockType*> blocks(const se::vector<Eigen::Vector3i>& voxel_coord,
                                                std::shared_ptr<OctreeT>           octree_ptr,
                                                typename OctreeT::NodeType*        base_parent_ptr);

template <typename OctreeT>
se::vector<typename OctreeT::BlockType*> blocks(se::vector<se::key_t>       voxel_keys,
                                                std::shared_ptr<OctreeT>    octree_ptr,
                                                typename OctreeT::NodeType* base_parent_ptr);

namespace { // anonymous namespace
template <typename OctreeT>
se::OctantBase* allocate_key(const se::key_t             key,
                             std::shared_ptr<OctreeT>    octree_ptr,
                             typename OctreeT::NodeType* base_parent_ptr);
} // anonymous namespace



} // struct allocator
} // namespace se

#include "impl/allocator_impl.hpp"

#endif //SE_ALLOCATOR_HPP
