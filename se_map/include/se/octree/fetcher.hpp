#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include "se/utils/type_util.hpp"
#include "se/octree/octree.hpp"



namespace se {
namespace fetcher {

/**
 * \brief Fetch the node for given coordinates and scale.
 *
 * \node TODO: Function not implemented yet.
 *
 * \tparam OctreeT
 * \param node_coord        The coordinates of the node to be fetched
 * \param node_scale        The scale of the node to be fetched
 * \param octree_ptr        The pointer to the octree
 * \param base_parent_ptr   The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The octant pointer is allocated, nullptr otherwise
 */
template <typename OctreeT>
inline se::OctantBase* node(const Eigen::Vector3i&      node_coord,
                            const se::scale_t           scale,
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
inline se::OctantBase* block(const Eigen::Vector3i&      block_coord,
                             std::shared_ptr<OctreeT>    octree_ptr,
                             typename OctreeT::NodeType* base_parent_ptr);

} // namespace fetcher
} // namespace se



#include "impl/fetcher_impl.hpp"

#endif // SE_FETCHER_HPP
