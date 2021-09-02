#ifndef SE_OCTANT_UTIL_IMPL_HPP
#define SE_OCTANT_UTIL_IMPL_HPP



namespace se {
namespace octantops {



template <typename BlockT, se::Sort SortT>
inline typename std::enable_if_t<SortT == se::Sort::SmallToLarge> sort_blocks(std::vector<se::OctantBase*>& block_ptrs) {
  auto has_smaller_key = [ ](const se::OctantBase* block_ptr_lhs, const se::OctantBase* block_ptr_rhs )
  {
      se::key_t key_lhs;
      se::keyops::encode_key(block_ptr_lhs->getCoord(), se::math::log2_const(static_cast<const BlockT*>(block_ptr_lhs)->getSize()), key_lhs);
      se::key_t key_rhs;
      se::keyops::encode_key(block_ptr_rhs->getCoord(), se::math::log2_const(static_cast<const BlockT*>(block_ptr_rhs)->getSize()), key_rhs);
      return  key_lhs < key_rhs;
  };
  std::sort(block_ptrs.begin( ), block_ptrs.end( ), has_smaller_key);
}



template <typename BlockT, se::Sort SortT>
inline typename std::enable_if_t<SortT == se::Sort::LargeToSmall> sort_blocks(std::vector<se::OctantBase*>& block_ptrs) {
  auto has_smaller_key = [ ](const se::OctantBase* block_ptr_lhs, const se::OctantBase* block_ptr_rhs )
  {
      se::key_t key_lhs;
      se::keyops::encode_key(block_ptr_lhs->getCoord(), se::math::log2_const(static_cast<const BlockT*>(block_ptr_lhs)->getSize()), key_lhs);
      se::key_t key_rhs;
      se::keyops::encode_key(block_ptr_rhs->getCoord(), se::math::log2_const(static_cast<const BlockT*>(block_ptr_rhs)->getSize()), key_rhs);
      return  key_lhs < key_rhs;
  };
  std::sort(block_ptrs.begin( ), block_ptrs.end( ), has_smaller_key);
}



inline int size_to_scale(const int octant_size)
{
  return se::math::log2_const(octant_size);
}



inline int scale_to_size(const int scale)
{
  return 1 << scale;
}

template <typename OctreeT>
inline int octant_to_size(const se::OctantBase* octant_ptr)
{
  if (octant_ptr->isBlock())
  {
    typedef typename OctreeT::BlockType BlockType;
    const BlockType* block_ptr = static_cast<const BlockType*>(octant_ptr);
    return block_ptr->getSize();
  } else
  {
    typedef typename OctreeT::NodeType NodeType;
    const NodeType* node_ptr = static_cast<const NodeType*>(octant_ptr);
    return node_ptr->getSize();
  }
}

} // namespace octantops
} // namespace se

#endif // SE_OCTANT_UTIL_IMPL_HPP

