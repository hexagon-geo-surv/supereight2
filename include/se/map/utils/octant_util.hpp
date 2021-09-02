#ifndef SE_OCTANT_UTIL_HPP
#define SE_OCTANT_UTIL_HPP

#include "se/common/math_util.hpp"
#include "key_util.hpp"

#include "se/map/octant/octant.hpp"


namespace se {
namespace octantops {



/**
 * \brief Sort a vector of blocks according to its morton code from small to large.
 *
 * \tparam BlockT           The type of blocks used
 * \tparam SortT            The sorting strategy (small to large by default)
 * \param[in] block_ptrs    The vector of block pointers
 * \return The sorted vector of block pointers from large to small according to their morton code.
 */
template <typename BlockT,
          se::Sort SortT = se::Sort::SmallToLarge
>
inline typename std::enable_if_t<SortT == se::Sort::SmallToLarge> sort_blocks(std::vector<se::OctantBase*>& block_ptrs);

/**
 * \brief Sort a vector of blocks according to its morton code from large to small.
 *
 * \tparam BlockT           The type of blocks used
 * \tparam SortT            The sorting strategy (small to large by default)
 * \param[in] block_ptrs    The vector of block pointers
 * \return The sorted vector of block pointers from large to small according to their morton code.
 */
template <typename BlockT,
          se::Sort SortT
>
inline typename std::enable_if_t<SortT == se::Sort::LargeToSmall> sort_blocks(std::vector<se::OctantBase*>& block_ptrs);

/**
 * \brief Convert the size of an octant in [voxel] to its scale in the octree.
 *
 * \param[in] octant_size  The size of the octant in [voxel]
 * \return The scale of the octant
 */
inline int size_to_scale(const int octant_size);

/**
 * \brief Convert the scale of a octant to its size in [voxel].
 *
 * \param[in] octant_scale  The scale of the octant
 * \return The size of the octant in [voxel]
 */
inline int scale_to_size(const int octant_scale);

/**
 * \brief Get the octants size.
 *
 * \tparam OctreeT
 * \param[in] octant_ptr The pointer to the octant
 *
 * \return The size of the octant
 */
template <typename OctreeT>
inline int octant_to_size(const se::OctantBase* octant_ptr);

} // namespace octantops
} // namespace se

#include "impl/octant_util_impl.hpp"

#endif // SE_OCTANT_UTIL_HPP

