#ifndef SE_OCTANT_UTIL_IMPL_HPP
#define SE_OCTANT_UTIL_IMPL_HPP

namespace se {
namespace octantops {

template <typename BlockT, se::Sort SortT = se::Sort::SmallToLarge>
inline typename std::enable_if_t<SortT == se::Sort::SmallToLarge> sort_blocks(se::vector<BlockT*>& block_ptrs) {
  auto has_smaller_key = [ ](const BlockT* block_ptr_lhs, const BlockT* block_ptr_rhs )
  {
      se::key_t key_lhs;
      se::keyops::encode_key(block_ptr_lhs->getCoord(), se::math::log2_const(block_ptr_lhs->getSize()), key_lhs);
      se::key_t key_rhs;
      se::keyops::encode_key(block_ptr_rhs->getCoord(), se::math::log2_const(block_ptr_rhs->getSize()), key_rhs);
      return  key_lhs < key_rhs;
  };
  std::sort(block_ptrs.begin( ), block_ptrs.end( ), has_smaller_key);
}

template <typename BlockT, se::Sort SortT>
inline typename std::enable_if_t<SortT == se::Sort::LargeToSmall> sort_blocks(se::vector<BlockT*>& block_ptrs) {
  auto has_smaller_key = [ ](const BlockT* block_ptr_lhs, const BlockT* block_ptr_rhs )
  {
      se::key_t key_lhs;
      se::keyops::encode_key(block_ptr_lhs->getCoord(), se::math::log2_const(block_ptr_lhs->getSize()), key_lhs);
      se::key_t key_rhs;
      se::keyops::encode_key(block_ptr_rhs->getCoord(), se::math::log2_const(block_ptr_rhs->getSize()), key_rhs);
      return  key_lhs < key_rhs;
  };
  std::sort(block_ptrs.begin( ), block_ptrs.end( ), has_smaller_key);
}

}
}

#endif //SE_OCTANT_UTIL_IMPL_HPP
