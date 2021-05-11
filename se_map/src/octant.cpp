#include "se/octree/octant.hpp"

namespace se
{
/**
 * \brief Sets the Base for every node and block.
 */
OctantBase::OctantBase(const Eigen::Vector3i& coord,
                       OctantBase*            parent_ptr)
        : parent_ptr_(parent_ptr), coord_(coord), time_stamp_(0), children_mask_(0)
{
}



Eigen::Vector3i OctantBase::getCoord() const
{
  return coord_;
}



se::OctantBase* OctantBase::getParent()
{
  return parent_ptr_;
}



se::OctantBase* OctantBase::getParent() const
{
  return parent_ptr_;
}



unsigned int OctantBase::getTimeStamp() const { return time_stamp_; }



void OctantBase::setTimeStamp(const unsigned int time_stamp) { time_stamp_ = time_stamp; }



unsigned int OctantBase::getChildrenMask() const { return children_mask_; }



void OctantBase::setChildrenMask(const unsigned int child_idx) { children_mask_ |= 1 << child_idx; }



} // namespace se

