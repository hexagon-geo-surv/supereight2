#ifndef SE_OCTREE_IMPL_HPP
#define SE_OCTREE_IMPL_HPP

namespace se {

template <typename DataT,
          Res      ResT,
          int      BlockSizeT
>
Octree<DataT, ResT, BlockSizeT>::Octree(const int size) : size_(size)
{
  assert(math::is_power_of_two(size)); // Verify that the octree size is a multiple of 2.
  assert(BlockSizeT < size);           // Verify that the block size is smaller than the root.

  root_ptr_ = memory_pool_.allocateNode(Eigen::Vector3i(0,0,0), size_);
}



template <typename DataT,
          Res      ResT,
          int      BlockSizeT
>
OctreeIterator<Octree<DataT, ResT, BlockSizeT>> Octree<DataT, ResT, BlockSizeT>::begin()
{
  return OctreeIterator<Octree<DataT, ResT, BlockSizeT>>(this);
}



template <typename DataT,
          Res      ResT,
          int      BlockSizeT
>
OctreeIterator<Octree<DataT, ResT, BlockSizeT>> Octree<DataT, ResT, BlockSizeT>::end()
{
  return OctreeIterator<Octree<DataT, ResT, BlockSizeT>>();
}



template <typename DataT,
          Res      ResT,
          int      BlockSizeT
>
bool Octree<DataT, ResT, BlockSizeT>::contains(const Eigen::Vector3i& voxel_coord)
{
  return voxel_coord.x() >= 0 && voxel_coord.x() < size_ &&
         voxel_coord.y() >= 0 && voxel_coord.y() < size_ &&
         voxel_coord.z() >= 0 && voxel_coord.z() < size_;
}



template <typename DataT,
          Res      ResT,
          int            BlockSizeT
>
bool Octree<DataT, ResT, BlockSizeT>::contains(const Eigen::Vector3i& voxel_coord) const
{
  return voxel_coord.x() >=0 && voxel_coord.x() < size_ &&
         voxel_coord.y() >=0 && voxel_coord.y() < size_ &&
         voxel_coord.z() >=0 && voxel_coord.z() < size_;
}



template <typename DataT,
          Res      ResT,
          int      BlockSizeT
>
bool Octree<DataT, ResT, BlockSizeT>::allocate(NodeType*          parent_ptr,
                                               const unsigned int child_idx)
{
  return allocate(parent_ptr, child_idx, nullptr);
}



template <typename DataT,
          Res      ResT,
          int      BlockSizeT
>
bool Octree<DataT, ResT, BlockSizeT>::allocate(NodeType*          parent_ptr,
                                               const unsigned int child_idx,
                                               OctantBase*&       child_ptr)
{
  assert(!parent_ptr->isBlock()); // Verify that the parent is not a block
  assert(parent_ptr);             // Verify that the parent is not a nullptr

  if (parent_ptr->getChild(child_idx, child_ptr))
  {
    return false; // Already allocated
  }

  if (parent_ptr->getSize() == BlockSizeT << 1) // Allocate Block
  {
#pragma omp critical
    {
      child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx);
    }
  } else                                        // Allocate Node
  {
#pragma omp critical
    {
      child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx);
    }
  }
  parent_ptr->setChild(child_idx, child_ptr); // Update parent
  return true;
}



} // namespace se

#endif // SE_OCTREE_IMPL_HPP
