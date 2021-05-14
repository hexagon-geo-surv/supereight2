#ifndef SE_PROPAGATOR_HPP
#define SE_PROPAGATOR_HPP

#include "se/octree/octant.hpp"

namespace se {
namespace propagator {



template <typename PropagateF>
void propagateToRoot(se::vector<se::unordered_set<se::OctantBase*>> octant_ptrs,
                     PropagateF&                                    propagate_funct);

/**
 * \brief Propage all newly integrated values from the voxel block depth up to the root of the octree
 */
template <typename PropagateF>
void propagateToRoot(se::vector<se::OctantBase*>& octant_ptrs,
                     PropagateF&                  propagate_funct);

void propagateTimeStampToRoot(se::vector<se::set<se::OctantBase*>> octant_ptrs);

void propagateTimeStampToRoot(std::unordered_set<se::OctantBase*>& octant_ptrs);

void propagateTimeStampToRoot(se::set<se::OctantBase*>& octant_ptrs);

void propagateTimeStampToRoot(se::vector<se::OctantBase*>& octant_ptrs);



} // namespace propagator
} // namespace se



#include "impl/propagator_impl.hpp"



#endif //SE_PROPAGATOR_HPP
