/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_IMPL_HPP
#define SE_OCTREE_IMPL_HPP

namespace se {

template<typename DataT, Res ResT, int BlockSize>
Octree<DataT, ResT, BlockSize>::Octree(const int size) :
        size_(math::power_two_up(size)),
        root_ptr_(memory_pool_.allocateNode(Eigen::Vector3i::Zero(), size_))
{
    assert(size_ > BlockSize);
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::begin()
{
    return OctreeIterator<Octree<DataT, ResT, BlockSize>>(this);
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::begin() const
{
    return cbegin();
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::cbegin() const
{
    return OctreeIterator<const Octree<DataT, ResT, BlockSize>>(this);
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::end()
{
    return OctreeIterator<Octree<DataT, ResT, BlockSize>>();
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::end() const
{
    return cend();
}



template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::cend() const
{
    return OctreeIterator<const Octree<DataT, ResT, BlockSize>>();
}



template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::contains(const Eigen::Vector3i& voxel_coord) const
{
    return voxel_coord.x() >= 0 && voxel_coord.x() < size_ && voxel_coord.y() >= 0
        && voxel_coord.y() < size_ && voxel_coord.z() >= 0 && voxel_coord.z() < size_;
}



template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::allocate(NodeType* parent_ptr,
                                              const int child_idx,
                                              OctantBase*& child_ptr)
{
    assert(parent_ptr);
    assert(!parent_ptr->isBlock());

    child_ptr = parent_ptr->getChild(child_idx);
    if (child_ptr) {
        return false; // Already allocated
    }

    const DataT init_data = parent_ptr->getData();
    if (parent_ptr->getSize() == BlockSize << 1) // Allocate Block
    {
#pragma omp critical
        {
            child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx, init_data);
        }
    }
    else // Allocate Node
    {
#pragma omp critical
        {
            child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx, init_data);
        }
    }
    parent_ptr->setChild(child_idx, child_ptr); // Update parent
    return true;
}



template<typename DataT, Res ResT, int BlockSize>
OctantBase* Octree<DataT, ResT, BlockSize>::allocate(NodeType* parent_ptr, const int child_idx)
{
    assert(parent_ptr);
    assert(!parent_ptr->isBlock());

    se::OctantBase* child_ptr = parent_ptr->getChild(child_idx);
    if (child_ptr) {
        return child_ptr;
    }

    const DataT init_data = parent_ptr->getData();
    if (parent_ptr->getSize() == BlockSize << 1) // Allocate Block
    {
#pragma omp critical
        {
            child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx, init_data);
        }
    }
    else // Allocate Node
    {
#pragma omp critical
        {
            child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx, init_data);
        }
    }
    parent_ptr->setChild(child_idx, child_ptr); // Update parent
    return child_ptr;
}



template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::deleteChildren(NodeType* parent_ptr)
{
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        se::OctantBase* octant_ptr = parent_ptr->getChild(child_idx);
        if (octant_ptr) {
            if (octant_ptr->isBlock()) {
                BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
                memory_pool_.deleteBlock(block_ptr);
            }
            else {
                NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);
                deleteChildren(node_ptr);
                memory_pool_.deleteNode(node_ptr);
            }
        }
        parent_ptr->setChild(child_idx, nullptr);
    }
    parent_ptr->clearChildrenMask();
}



template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::allocateAll(NodeType* parent_ptr,
                                                 const int child_idx,
                                                 OctantBase*& child_ptr)
{
    assert(parent_ptr);
    assert(!parent_ptr->isBlock());

    child_ptr = parent_ptr->getChild(child_idx);
    if (child_ptr) {
        return false; // Already allocated
    }

    const DataT init_data = parent_ptr->getData();
    for (int idx = 0; idx < 8; idx++) {
        if (parent_ptr->getSize() == BlockSize << 1) // Allocate Block
        {
#pragma omp critical
            {
                child_ptr = memory_pool_.allocateBlock(parent_ptr, idx, init_data);
            }
        }
        else // Allocate Node
        {
#pragma omp critical
            {
                child_ptr = memory_pool_.allocateNode(parent_ptr, idx, init_data);
            }
        }
        parent_ptr->setChild(idx, child_ptr); // Update parent
    }

    child_ptr = parent_ptr->getChild(child_idx);
    return true;
}



template<typename DataT, Res ResT, int BlockSize>
OctantBase* Octree<DataT, ResT, BlockSize>::allocateAll(NodeType* parent_ptr, const int child_idx)
{
    assert(parent_ptr);
    assert(!parent_ptr->isBlock());

    se::OctantBase* child_ptr = parent_ptr->getChild(child_idx);
    if (child_ptr) {
        return child_ptr;
    }

    const DataT init_data = parent_ptr->getData();
    for (int idx = 0; idx < 8; idx++) {
        if (parent_ptr->getSize() == BlockSize << 1) // Allocate Block
        {
#pragma omp critical
            {
                child_ptr = memory_pool_.allocateBlock(parent_ptr, idx, init_data);
            }
        }
        else // Allocate Node
        {
#pragma omp critical
            {
                child_ptr = memory_pool_.allocateNode(parent_ptr, idx, init_data);
            }
        }
        parent_ptr->setChild(idx, child_ptr); // Update parent
    }
    return parent_ptr->getChild(child_idx);
}



} // namespace se

#endif // SE_OCTREE_IMPL_HPP
