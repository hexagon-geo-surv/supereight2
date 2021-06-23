#include "se/octree/octant.hpp"

namespace se
{



/**
 * \brief Sets the Base for every node and block.
 */
OctantBase::OctantBase(const bool             is_block,
                       const Eigen::Vector3i& coord,
                       OctantBase*            parent_ptr)
        : is_block_(is_block), parent_ptr_(parent_ptr), coord_(coord), time_stamp_(0), is_active_(true), children_mask_(0)
{
}



} // namespace se

