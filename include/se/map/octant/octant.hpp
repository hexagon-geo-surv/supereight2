#ifndef SE_OCTANT_HPP
#define SE_OCTANT_HPP

#include <memory>
#include <Eigen/Dense>

#include "se/common/math_util.hpp"
#include "se/map/data.hpp"
#include "se/map/utils/setup_util.hpp"

#include <iostream>

namespace se {

/**
 * \brief This class only helps to dynamic cast the octant to the right type and builds the base of nodes and blocks.
 */
class OctantBase
{
public:
  OctantBase(const bool             is_block,
             const Eigen::Vector3i& coord,
             OctantBase*            parent_ptr = nullptr);

  inline bool isBlock() const { return is_block_; };

  inline Eigen::Vector3i getCoord() const { return coord_; }

  inline se::OctantBase* getParent() { return parent_ptr_; }

  inline se::OctantBase* getParent() const { return parent_ptr_; }

  inline int getTimeStamp() const { return time_stamp_; }

  inline void setTimeStamp(const unsigned int time_stamp) { time_stamp_ = time_stamp; }

  inline bool getActive() const { return is_active_; }

  inline void setActive (bool is_active) { is_active_ = is_active; }

  inline unsigned int getChildrenMask() const { return children_mask_; }

//  inline void setChildrenMask(const unsigned int child_idx) { children_mask_ |= 1 << child_idx; } // TODO

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

