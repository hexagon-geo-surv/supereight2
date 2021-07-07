#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include "se/map/utils/type_util.hpp"
#include "se/map/octree/octree.hpp"



namespace se {
namespace fetcher {



/**
 * \brief Fetch the octant for given coordinates and scale.
 *
 * \tparam OctreeT
 * \param octant_coord    The coordinates of the octant to be fetched
 * \param scale_desired   The scale of the node to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the octant if it is allocated, nullptr otherwise
 */
template <typename OctreeT>
inline se::OctantBase* octant(const Eigen::Vector3i& octant_coord,
                              const se::scale_t      scale_desired,
                              se::OctantBase*        base_parent_ptr);

/**
 * \brief Fetch the octant for given coordinates and scale.
 *        Returnes the finest allocated octant up to the desired scale
 *
 * \tparam OctreeT
 * \param octant_coord    The coordinates of the octant to be fetched
 * \param scale_desired   The scale of the node to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the finest allocated octant up to the desired scale
 */
template <typename OctreeT>
inline se::OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                                     const se::scale_t      scale_desired,
                                     se::OctantBase*        base_parent_ptr);

/**
 * \brief Fetch the block for given block coordinates.
 *
 * \tparam OctreeT
 * \param block_coord     The coordinates of the block to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the block if allocated, nullptr otherwise
 */
template <typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& block_coord,
                             se::OctantBase*        base_parent_ptr);

/**
 * \brief Fetch the leaf for given block coordinates.
 *
 * \tparam OctreeT
 * \param leaf_coord      The coordinates of the block to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the leaf
 */
template <typename OctreeT>
inline se::OctantBase* leaf(const Eigen::Vector3i& leaf_coord,
                            se::OctantBase*        base_parent_ptr);



} // namespace fetcher
} // namespace se



#include "impl/fetcher_impl.hpp"

#endif // SE_FETCHER_HPP
