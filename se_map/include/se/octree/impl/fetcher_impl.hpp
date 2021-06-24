#ifndef SE_FETCHER_IMPL_HPP
#define SE_FETCHER_IMPL_HPP



namespace se {
namespace fetcher {



template <typename OctreeT>
inline se::OctantBase* octant(const Eigen::Vector3i& octant_coord,
                              const se::scale_t      scale_desired,
                              se::OctantBase*        base_parent_ptr)
{
  typename OctreeT::NodeType* parent_ptr = static_cast<typename OctreeT::NodeType*>(base_parent_ptr);
  int child_size  = parent_ptr->getSize() >> 1;
  se::OctantBase* child_ptr  = nullptr;

  int size_desired = std::max(1 << scale_desired, (int) OctreeT::BlockType::getSize()); // Note smaller than block size
  for (; child_size >= size_desired; child_size = child_size >> 1)
  {
    se::idx_t child_idx = ((octant_coord.x() & child_size) > 0) + 2 * ((octant_coord.y() & child_size) > 0) + 4 * ((octant_coord.z() & child_size) > 0);
    child_ptr = parent_ptr->getChild(child_idx);
    if(!child_ptr)
    {
      return nullptr;
    }
    parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
  }

  return child_ptr;
}



template <typename OctreeT>
inline se::OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                                     const se::scale_t      scale_desired,
                                     se::OctantBase*        base_parent_ptr)
{
  typename OctreeT::NodeType* parent_ptr = static_cast<typename OctreeT::NodeType*>(base_parent_ptr);
  int child_size  = parent_ptr->getSize() >> 1;
  se::OctantBase* child_ptr  = nullptr;

  int size_desired = std::max(1 << scale_desired, (int) OctreeT::BlockType::getSize()); // Note smaller than block size
  for (; child_size >= size_desired; child_size = child_size >> 1)
  {
    se::idx_t child_idx = ((octant_coord.x() & child_size) > 0) + 2 * ((octant_coord.y() & child_size) > 0) + 4 * ((octant_coord.z() & child_size) > 0);
    child_ptr = parent_ptr->getChild(child_idx);

    if(!child_ptr)
    {
      se::OctantBase* leaf_ptr = (parent_ptr->getChildrenMask() == 0) ? parent_ptr : nullptr;
      return leaf_ptr; // leaf is either a block or a parent with no children!
    }

    parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
  }

  return child_ptr;
}



template <typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& block_coord,
                             se::OctantBase*        base_parent_ptr)
{
  typename OctreeT::NodeType* parent_ptr = static_cast<typename OctreeT::NodeType*>(base_parent_ptr);
  unsigned child_size  = parent_ptr->getSize() >> 1;
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



template <typename OctreeT>
inline se::OctantBase* leaf(const Eigen::Vector3i& leaf_coord,
                            se::OctantBase*        base_parent_ptr)
{
  typename OctreeT::NodeType* parent_ptr = static_cast<typename OctreeT::NodeType*>(base_parent_ptr);
  unsigned child_size  = parent_ptr->getSize() >> 1;
  se::OctantBase* child_ptr  = nullptr;

  for (; child_size >= OctreeT::BlockType::getSize(); child_size = child_size >> 1)
  {
    se::idx_t child_idx = ((leaf_coord.x() & child_size) > 0) + 2 * ((leaf_coord.y() & child_size) > 0) + 4 * ((leaf_coord.z() & child_size) > 0);
    child_ptr = parent_ptr->getChild(child_idx);

    if(!child_ptr)
    {
      se::OctantBase* leaf_ptr = (parent_ptr->getChildrenMask() == 0) ? parent_ptr : nullptr;
      return leaf_ptr; // leaf is either a block or a parent with no children!
    }

    parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
  }

  return child_ptr;
}


} // namespace fetcher
} // namespace se



#endif // SE_FETCHER_IMPL_HPP
