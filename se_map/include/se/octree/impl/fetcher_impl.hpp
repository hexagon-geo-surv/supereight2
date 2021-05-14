#ifndef SE_FETCHER_IMPL_HPP
#define SE_FETCHER_IMPL_HPP



namespace se {
namespace fetcher {



template <typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i&      block_coord,
                             const OctreeT&              /* octree */,
                             typename OctreeT::NodeType* base_parent_ptr)
{
  unsigned child_size  = base_parent_ptr->getSize() >> 1;
  typename OctreeT::NodeType* parent_ptr = base_parent_ptr;
  se::OctantBase* child_ptr  = nullptr;

  for (; child_size >= OctreeT::BlockType::getSize(); child_size = child_size >> 1)
  {
    se::idx_t child_idx = ((block_coord.x() & child_size) > 0) + 2 * ((block_coord.y() & child_size) > 0) + 4 * ((block_coord.z() & child_size) > 0);
    child_ptr = parent_ptr->getChild(child_idx);
    if(!child_ptr)
    {
      return nullptr;
    }
    parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
  }

  return child_ptr;
}



} // namespace fetcher
} // namespace se



#endif // SE_FETCHER_IMPL_HPP
