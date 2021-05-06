#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include "se/utils/type_util.hpp"
#include "se/octree/octree.hpp"



namespace se {
namespace fetcher {



/**
 * \brief Fetch the octant for a given key.
 *
 * \tparam OctreeT
 * \param key               The key of the octant to be fetched
 * \param octree_ptr        The pointer to the octree
 * \param base_parent_ptr   The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The octant pointer is allocated, nullptr otherwise
 */
template <typename OctreeT>
se::OctantBase* octant(const se::key_t             key,
                       std::shared_ptr<OctreeT>    octree_ptr,
                       typename OctreeT::NodeType* base_parent_ptr);



/**
 * \brief Fetch the block for given block coordinates.
 *
 * \tparam OctreeT
 * \param block_coord       The coordinates of the block to be fetched
 * \param octree_ptr        The pointer to the octree
 * \param base_parent_ptr   The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The octant pointer is allocated, nullptr otherwise
 */
template <typename OctreeT>
se::OctantBase* block(const Eigen::Vector3i&      block_coord,
                      std::shared_ptr<OctreeT>    octree_ptr,
                      typename OctreeT::NodeType* base_parent_ptr);



} // namespace fetcher
} // namespace se



#include "impl/fetcher_impl.hpp"

#endif // SE_FETCHER_HPP
