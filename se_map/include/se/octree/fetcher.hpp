#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include "se/utils/type_util.hpp"
#include "se/octree/octree.hpp"



namespace se {
namespace fetcher {



template <typename OctreeT>
se::OctantBase* octant(const se::key_t             key,
                       std::shared_ptr<OctreeT>    octree_ptr,
                       typename OctreeT::NodeType* base_parent_ptr);



template <typename OctreeT>
se::OctantBase* block(const Eigen::Vector3i&      block_coord,
                      std::shared_ptr<OctreeT>    octree_ptr,
                      typename OctreeT::NodeType* base_parent_ptr);



} // namespace fetcher
} // namespace se



#include "impl/fetcher_impl.hpp"

#endif // SE_FETCHER_HPP
