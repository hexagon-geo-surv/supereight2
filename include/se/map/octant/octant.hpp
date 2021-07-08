#ifndef SE_OCTANT_HPP
#define SE_OCTANT_HPP

#include <iostream>

#include "se/common/math_util.hpp"
#include "se/map/data.hpp"
#include "se/map/utils/key_util.hpp"
#include "se/map/utils/setup_util.hpp"

namespace se {

/**
 * \brief This class only helps to dynamic cast the octant to the right type and builds the base of nodes and blocks.
 */
class OctantBase
{
public:
  /**
   * \brief Setup a octant via its parent
   *
   * \param is_block    The block state of the octant
   * \param coord       The voxel coordinates of the octant
   * \param parent_ptr  The pointer to the parent of the octant
   */
  OctantBase(const bool             is_block,
             const Eigen::Vector3i& coord,
             OctantBase*            parent_ptr = nullptr);

  /**
   * \brief Verify if an octant is a block.
   *
   * \return True if the octant is a block, false otherwise
   */
  inline bool isBlock() const { return is_block_; };

  /**
   * \brief Get the voxel coordinates of the octant.
   *
   * \return The voxel coordinates of the octant
   */
  inline Eigen::Vector3i getCoord() const { return coord_; }

  /**
   * \brief Get the parent pointer of the octant.
   *
   * \return The parent pointer of the octant
   */
  inline const se::OctantBase* getParent() const { return parent_ptr_; }

  inline       se::OctantBase* getParent()       { return parent_ptr_; }

  /**
   * \brief Get the time stamp of an octant.
   *
   * \note The time stamp is defined as an integer
   *
   * \return The time stamp of the octant
   */
  inline int getTimeStamp() const { return time_stamp_; }

  /**
   * \brief Set the time stamp of an octant.
   *
   * \note The time stamp is defined as an integer
   *
   * \param[in] time_stamp  The time stamp of the octant
   */
  inline void setTimeStamp(const int time_stamp) { time_stamp_ = time_stamp; }

  /**
   * \brief Get the active state of an octant.
   *
   * \return The active state of the octant
   */
  inline bool getActive() const { return is_active_; }

  /**
   * \brief Set the active state of an octant.
   *
   * \param[in] is_active   The active state of the octant
   */
  inline void setActive (bool is_active) { is_active_ = is_active; }

  inline unsigned int getChildrenMask() const { return children_mask_; }

  /**
   * \brief Clear the children mask.
   *
   * \warning Only use this function if all children pointer are 'nullptr's.
   */
  inline void clearChildrenMask() { children_mask_ = 0; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  const bool            is_block_;
  OctantBase*           parent_ptr_;    ///< Every node/block (other than root) needs a parent
  const Eigen::Vector3i coord_;         ///< The coordinates of the block (left, front , bottom corner)
  int                   time_stamp_;    ///< The frame of the last update
  bool                  is_active_;     ///< The active state of the octant
  unsigned int          children_mask_; ///< The allocated children

  template<typename DerT, typename DatT, int BS>
  friend class BlockSingleRes;

  template<typename DatT, int BS, typename DerT>
  friend class BlockMultiRes;

  template<typename DatT, typename DerT>
  friend class NodeMultiRes;
};



} // namespace se


#include "node.hpp"
#include "block.hpp"

#endif // SE_OCTANT_HPP

