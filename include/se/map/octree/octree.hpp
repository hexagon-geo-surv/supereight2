/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_HPP
#define SE_OCTREE_HPP

#include <memory>

#include "se/map/octant/octant.hpp"
#include "se/map/octree/iterator.hpp"
#include "se/map/utils/key_util.hpp"
#include "se/map/utils/memory_pool.hpp"
#include "se/map/utils/setup_util.hpp"



namespace se {

/**
 * \brief The octree is the memory manager of the map.
 *        It is the only entity that is able to allocate and deallocate nodes and blocks.
 *        However it is not responsible to process the data in the nodes.
 *
 * \tparam DataT        The data struct stored in each voxel (or node for Res::Multi)
 * \tparam ResT         The resolution type (Res::Single, Res::Multi) defining if data can only stored at
 *                      a finest scale or any scale.
 * \tparam BlockSize    The size in voxels of a block. BlockSize in [1, (octree size) / 2]
 *                      Must be a power of two.
 */
template<typename DataT, Res ResT = Res::Single, int BlockSize = 8>
class Octree {
    public:
    typedef std::shared_ptr<Octree<DataT, ResT, BlockSize>> Ptr;

    typedef DataT DataType;
    typedef Node<DataT, ResT> NodeType;
    typedef Block<DataT, ResT, BlockSize> BlockType;

    typedef se::BoostMemoryPool<NodeType, BlockType> MemoryPool;

    // Compile-time constant expressions
    // # of voxels per side in a voxel block
    static constexpr unsigned int block_size = BlockSize;
    // The maximum scale of a block
    static constexpr se::scale_t max_block_scale = math::log2_const(BlockSize);

    /** \brief Initialize an octree with an edge length of at least \p size voxels. The actual edge
     * length in voxels will be the smallest power of 2 that is greater or equal to \p size.
     */
    Octree(const int size);

    Octree(const Octree&) = delete;            ///< Delete copy constructor
    Octree& operator=(const Octree&) = delete; ///< Delete copy assignment operator

    OctreeIterator<Octree<DataT, ResT, BlockSize>> begin();
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> begin() const;
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> cbegin() const;
    OctreeIterator<Octree<DataT, ResT, BlockSize>> end();
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> end() const;
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> cend() const;

    /**
   * \brief Verify if the voxel coordinates are contained in the octree.
   *
   * \param[in] voxel_coord The voxel coordinates to be verified
   *
   * \return True if contained in the octree, False otherwise
   */
    bool contains(const Eigen::Vector3i& voxel_coord) const;

    /**
   * \brief Get the node pointer to the root of the octree.
   *
   * \return The pointer to the root of the octree
   */
    se::OctantBase* getRoot()
    {
        return root_ptr_;
    };

    /**
   * \brief Get the node pointer to the root of the octree.
   *
   * \return The pointer to the root of the octree
   */
    se::OctantBase* getRoot() const
    {
        return root_ptr_;
    };

    /**
   * \brief Get the size of the octree in [voxel] units.
   *
   * \return The size of the octree
   */
    int getSize() const
    {
        return size_;
    }

    /**
   * \brief Get the maximum scale of the octree. This is equivalent to the scale of the root.
   *
   * \return The max scale of the octree
   */
    int getMaxScale() const
    {
        return se::math::log2_const(size_);
    }

    /**
   * \brief Get the octree depth the blocks are allocated at.
   *
   * \return The octree depth the blocks are allocated at
   */
    int getBlockDepth() const
    {
        return se::math::log2_const(size_) - se::math::log2_const(BlockSize);
    }

    /**
   * \brief Allocate a node for a given parent node.
   *
   * \warning The returned pointer is of type OctantBase as child might be a node or block.
   *
   * \warning This function might be dangerous when using Multires Occupancy.
   *          Preferably use the allocateAll(...) function if unsure.
   *
   * \param[in] parent_ptr  The parent of the octant to be allocated
   * \param[in] child_idx   The child index of the octant to be allocated
   * \param[out] child_ptr  The pointer ot the allocated /fetched octant
   *
   * \return Ture if the node has been newly allocated, False if it has already been allocated
   */
    bool allocate(NodeType* parent_ptr,
                  const int child_idx,
                  se::OctantBase*& child_ptr); ///< Allocate child

    /**
   * \brief Allocate a octant for a given parent node.
   *
   * \warning The returned pointer is of type OctantBase as child might be a node or block.
   *
   * \warning This function might be dangerous when using Multires Occupancy.
   *          Preferably use the allocateAll(...) function if unsure.
   *
   * \param[in] parent_ptr  The parent of the octant to be allocated
   * \param[in] child_idx   The child index of the octant to be allocated
   *
   * \return The pointer ot the allocated / fetched octant
   */
    se::OctantBase* allocate(NodeType* parent_ptr, const int child_idx);

    /**
   * \brief Allocate all of the parent node's child octants.
   *
   * \warning The returned pointer is of type OctantBase as child might be a node or block.
   *
   * \param[in] parent_ptr  The parent of the octants to be allocated
   * \param[in] child_idx   The child index of the octant to be returned
   * \param[out] child_ptr  The pointer ot the allocated / fetched octant of the child_idx
   *
   * \return Ture if the node has been newly allocated, False if it has already been allocated
   */
    bool allocateAll(NodeType* parent_ptr,
                     const int child_idx,
                     se::OctantBase*& child_ptr); ///< Allocate child

    /**
   * \brief Allocate all of the parent node's child octants.
   *
   * \warning The returned pointer is of type OctantBase as child might be a node or block.
   *
   * \param[in] parent_ptr  The parent of the octants to be allocated
   * \param[in] child_idx   The child index of the octant to be returned
   * \param[out] child_ptr  The pointer ot the allocated octant
   *
   * \return The pointer ot the allocated / fetched octant of the child_idx
   */
    se::OctantBase* allocateAll(NodeType* parent_ptr, const int child_idx);

    /**
   * \brief Recursively delete all children of a given node pointer.
   *
   * \param[in] parent_ptr The node pointer to delete the children of
   */
    void deleteChildren(NodeType* parent_ptr);

    static constexpr se::Field fld_ = DataT::fld_;
    static constexpr se::Colour col_ = DataT::col_;
    static constexpr se::Semantics sem_ = DataT::sem_;

    static constexpr se::Res res_ = ResT;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    int size_;                           ///< The size in [voxel] of the octree
    se::OctantBase* root_ptr_ = nullptr; ///< The pointer to the root node of the octree

    MemoryPool memory_pool_; ///< The memory pool pre-allocating memory for nodes and blocks

    static_assert(math::is_power_of_two(BlockSize));
};



template<typename DataT, Res ResT, int BlockSize>
constexpr se::scale_t Octree<DataT, ResT, BlockSize>::max_block_scale;

} // namespace se

#include "impl/octree_impl.hpp"

#endif // SE_OCTREE_HPP
