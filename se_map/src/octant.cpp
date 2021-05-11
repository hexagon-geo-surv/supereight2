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



} // namespace se

