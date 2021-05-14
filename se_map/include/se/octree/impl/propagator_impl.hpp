#ifndef SE_PROPAGATOR_IMPL_HPP
#define SE_PROPAGATOR_IMPL_HPP



#include "se/utils/type_util.hpp"



namespace se {
namespace propagator
{

template <typename PropagateF>
void propagateToRoot(se::vector<se::unordered_set<se::OctantBase*>> octant_ptrs,
                     PropagateF&                          propagate_funct)
{
  for (int d = octant_ptrs.size() - 1; d > 0; d--)
  {
    se::unordered_set<se::OctantBase*> child_ptrs_at_depth = octant_ptrs[d];
    se::unordered_set<se::OctantBase*>::iterator child_ptr_itr;

    for (child_ptr_itr = child_ptrs_at_depth.begin(); child_ptr_itr != child_ptrs_at_depth.end(); ++child_ptr_itr)
    {
      se::OctantBase* child_ptr = *child_ptr_itr;
      se::OctantBase* parent_ptr = child_ptr->getParent();
      assert(parent_ptr);

      propagate_funct(child_ptr, parent_ptr);

      octant_ptrs[d-1].insert(parent_ptr);
    }

  } // d
}



/**
 * \brief Propage all newly integrated values from the voxel block depth up to the root of the octree
 */
template <typename PropagateF>
void propagateToRoot(se::vector<se::OctantBase*>& octant_ptrs,
                     PropagateF&                  propagate_funct)
{
  TICK("propagate-nodes-vector")

  se::unordered_set<se::OctantBase*> child_ptrs;
  se::unordered_set<se::OctantBase*> parent_ptrs;

  for (const auto& child_ptr : octant_ptrs)
  {
    se::OctantBase* parent_ptr = child_ptr->getParent();
    assert(parent_ptr);

    if (child_ptr->getTimeStamp() > parent_ptr->getTimeStamp())
    {
      parent_ptr->setTimeStamp(child_ptr->getTimeStamp());
      child_ptrs.insert(parent_ptr);
    }
  } // block_ptrs

  while (!parent_ptrs.empty())
  {
    se::unordered_set<se::OctantBase*>::iterator child_ptr_itr;

    for (child_ptr_itr = child_ptrs.begin(); child_ptr_itr != child_ptrs.end(); ++child_ptr_itr)
    {
      se::OctantBase* child_ptr = *child_ptr_itr;
      se::OctantBase* parent_ptr = child_ptr->getParent();

      if (parent_ptr)
      {
        propagate_funct(child_ptr, parent_ptr);
        parent_ptrs.insert(parent_ptr);
      }
    }

    std::swap(child_ptrs, parent_ptrs);
    parent_ptrs.clear();

  } // while

  TOCK("propagate-nodes-vector")
}



void propagateTimeStampToRoot(se::vector<se::unordered_set<se::OctantBase*>> octant_ptrs)
{
  auto time_step_prop = [](se::OctantBase* octant_ptr, se::OctantBase* parent_ptr)
  {
      if (octant_ptr->getTimeStamp() > parent_ptr->getTimeStamp())
      {
        parent_ptr->setTimeStamp(octant_ptr->getTimeStamp());
      }
  };

  propagateToRoot(octant_ptrs, time_step_prop);
}



void propagateTimeStampToRoot(se::vector<se::OctantBase*>& octant_ptrs)
{
  auto time_step_prop = [](se::OctantBase* child_ptr, se::OctantBase* parent_ptr)
  {
      if (child_ptr->getTimeStamp() > parent_ptr->getTimeStamp())
      {
        parent_ptr->setTimeStamp(child_ptr->getTimeStamp());
      }
  };

  propagateToRoot(octant_ptrs, time_step_prop);
}


}
}



#endif //SE_PROPAGATOR_IMPL_HPP
