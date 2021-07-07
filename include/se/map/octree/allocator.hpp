#ifndef SE_ALLOCATOR_HPP
#define SE_ALLOCATOR_HPP

#include <memory>
#include <Eigen/Dense>

#include "se/map/utils/type_util.hpp"
#include "se/map/octree/octree.hpp"

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
 * \param[in] octree        The octree to allocate the block in
 * \param[in] base_parent_ptr   The starting node pointer (default nullptr will be replaced with octree root)
 *
 * \return A block pointer to the allocated block
 */
template <typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& voxel_coord,
                             OctreeT&               octree,
                             se::OctantBase*        base_parent_ptr);

template <typename OctreeT>
inline se::OctantBase* block(const se::key_t voxel_key,
                             OctreeT&        octree,
                             se::OctantBase* base_parent_ptr);

/**
 * \brief Allocate Blocks at the provided voxel coordinates.
 *
 * \tparam OctreeT            The Octree template.
 * \param[in] voxel_coord     The 3D coordinates of a voxel within each block.
 * \param[in] octree          The octree to allocate the blocks in.
 * \param[in] base_parent_ptr The starting node pointer. A nullptr will be replaced with the octree
 *                            root.
 * \param[in] only_allocated  Return pointers only for the newly allocated Blocks instead of all the
 *                            Blocks corresponding to the coordinates in voxel_coord.
 *
 * \return Pointers to the allocated Octants.
 */
template <typename OctreeT>
std::vector<se::OctantBase*> blocks(const std::vector<Eigen::Vector3i>& voxel_coord,
                                    OctreeT&                            octree,
                                    se::OctantBase*                     base_parent_ptr,
                                    const bool                          only_allocated = false);

/**
 * \brief Allocate Blocks at the provided voxel Morton codes.
 *
 * \tparam OctreeT            The Octree template.
 * \param[in] voxel_keys      The Morton code of a voxel within each block.
 * \param[in] octree          The octree to allocate the blocks in.
 * \param[in] base_parent_ptr The starting node pointer. A nullptr will be replaced with the octree
 *                            root.
 * \param[in] only_allocated  Return pointers only for the newly allocated Blocks instead of all the
 *                            Blocks corresponding to the Morton codes in voxel_keys.
 *
 * \return Pointers to the allocated Octants.
 */
template <typename OctreeT>
std::vector<se::OctantBase*> blocks(std::vector<se::key_t>& voxel_keys,
                                    OctreeT&                octree,
                                    se::OctantBase*         base_parent_ptr,
                                    const bool              only_allocated = false);

namespace detail {


/**
 *
 * \tparam OctreeT
 * \param key
 * \param octree
 * \param base_parent_ptr
 * \param allocated_octant
 * \return
 */
template <typename OctreeT>
inline bool allocate_key(const se::key_t  key,
                         OctreeT&         octree,
                         se::OctantBase*  base_parent_ptr,
                         se::OctantBase*& allocated_octant);



} // namespace detail
} // struct allocator
} // namespace se

#include "impl/allocator_impl.hpp"

#endif //SE_ALLOCATOR_HPP
