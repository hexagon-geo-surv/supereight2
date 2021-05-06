#ifndef SE_OCTANT_UTIL_HPP
#define SE_OCTANT_UTIL_HPP

#include "se/utils/math_util.hpp"
#include "se/utils/key_util.hpp"



namespace se {
namespace octantops {

template <typename BlockT, se::Sort SortT = se::Sort::SmallToLarge>
inline typename std::enable_if_t<SortT == se::Sort::SmallToLarge> sort_blocks(se::vector<BlockT*>& block_ptrs);

template <typename BlockT, se::Sort SortT>
inline typename std::enable_if_t<SortT == se::Sort::LargeToSmall> sort_blocks(se::vector<BlockT*>& block_ptrs);

} // namespace octantops
} // namespace se

#include "impl/octant_util_impl.hpp"

#endif //SE_OCTANT_UTIL_HPP
