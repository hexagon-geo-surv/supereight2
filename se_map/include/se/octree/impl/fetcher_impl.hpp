#ifndef SE_FETCHER_IMPL_HPP
#define SE_FETCHER_IMPL_HPP



namespace se {
namespace fetcher {



template <typename OctreeT>
se::OctantBase* octant(const se::key_t             key,
                       std::shared_ptr<OctreeT>    octree_ptr,
                       typename OctreeT::NodeType* base_parent_ptr)
{
  assert(se::keyops::is_valid(key)); // Verify if the key is valid
  typename OctreeT::NodeType* parent_ptr = base_parent_ptr;
  se::OctantBase*             child_ptr  = nullptr;

  int child_scale = se::math::log2_const(parent_ptr->getSize()) - 1;
  se::code_t code = se::keyops::key_to_code(key);
  int min_scale   = std::max(se::keyops::key_to_scale(key), octree_ptr->max_block_scale);

  for (; child_scale >= min_scale; --child_scale)
  {
    se::idx_t child_idx = se::keyops::code_to_child_idx(code, child_scale);
    if(!parent_ptr->getChild(child_idx, child_ptr))
    {
      return nullptr;
    }
    parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
  }
  return child_ptr;
}



template <typename OctreeT>
se::OctantBase* block(const Eigen::Vector3i&      block_coord,
                      std::shared_ptr<OctreeT>    octree_ptr,
                      typename OctreeT::NodeType* base_parent_ptr)
{
  se::key_t block_key;
  se::keyops::encode_key(block_coord, octree_ptr->max_block_scale, block_key);
  return se::fetcher::octant(block_key, octree_ptr, base_parent_ptr);
}


} // namespace fetcher
} // namespace se



#endif // SE_FETCHER_IMPL_HPP
