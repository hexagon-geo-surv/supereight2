#ifndef SE_OCTREE_HPP
#define SE_OCTREE_HPP

#include <memory>
#include <deque>

#include "se/utils/memory pool.hpp"
#include "se/utils/key_util.hpp"
#include "se/utils/setup_util.hpp"
#include "se/octree/octant.hpp"
#include "se/octree/iterator.hpp"



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
template <typename DataT,
          Res      ResT      = Res::Single,
          int      BlockSize = 8
>
class Octree {
public:
  typedef std::shared_ptr<Octree<DataT, ResT, BlockSize> > Ptr;

  typedef DataT                         DataType;
  typedef Node<DataT, ResT>             NodeType;
  typedef Block<DataT, ResT, BlockSize> BlockType;

  typedef se::BoostMemoryPool<NodeType, BlockType> MemoryPool;

  // Compile-time constant expressions
  // # of voxels per side in a voxel block
  static constexpr unsigned int block_size = BlockSize;
  // The maximum scale of a block
  static constexpr se::scale_t max_block_scale = math::log2_const(BlockSize);

  /**
   * \brief The octree needs to be initialised during the allocation.
   *
   * \param[in] size    The size in [voxel] of the octree
   */
  Octree(const int size);

  ~Octree() {};                               ///< TODO:
  Octree(const Octree&) = delete;             ///< Delete copy constructor
  Octree & operator=(const Octree&) = delete; ///< Delete copy assignment operator

  OctreeIterator<Octree<DataT, ResT, BlockSize>> begin();
  OctreeIterator<Octree<DataT, ResT, BlockSize>> end();

  /**
   * \brief Verify if the voxel coordinates are contained in the octree.
   *
   * \param[in] voxel_coord The voxel coordinates to be verified
   *
   * \return True if contained in the octree, False otherwise
   */
  inline bool contains(const Eigen::Vector3i& voxel_coord);

  /**
   * \brief Verify if the voxel coordinates are contained in the octree.
   *
   * \param[in] voxel_coord The voxel coordinates to be verified
   *
   * \return True if contained in the octree, False otherwise
   */
  inline bool contains(const Eigen::Vector3i& voxel_coord) const;

  /**
   * \brief Get the node pointer to the root of the octree.
   *
   * \return The pointer to the root of the octree
   */
  inline NodeType* getRoot() { return root_ptr_; };

  /**
   * \brief Get the size of the octree in [voxel] units.
   *
   * \return The size of the octree
   */
  inline unsigned int getSize() { return size_; }

  /**
   * \brief Get the maximum scale of the octree. This is equivalent to the scale of the root.
   *
   * \return The max scale of the octree
   */
  inline unsigned int getMaxScale() { return se::math::log2_const(size_); }

  /**
   * \brief Allocate a node for a given parent node.
   *
   * \warning The returned pointer is of type OctantBase as child might be a node or block.
   *
   * \param[in] parent_ptr  The parent of the node to be allocated
   * \param[in] child_idx   The child index of the node to be allocated
   * \param[out] child_ptr     The pointer ot the allocated node
   *
   * \return Ture if the node has been newly allocated, False if it has already been allocated
   */
  inline bool allocate(NodeType*        parent_ptr,
                       const unsigned   child_idx,
                       se::OctantBase*& child_ptr);     ///< Allocate child

  inline se::OctantBase* allocate(NodeType*        parent_ptr,
                                  const unsigned   child_idx);     ///< Allocate child

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  int size_;                     ///< The size in [voxel] of the octree
  NodeType* root_ptr_ = nullptr; ///< The pointer to the root node of the octree

  MemoryPool memory_pool_;       ///< The memory pool pre-allocating memory for nodes and blocks
};



template <typename DataT,
          Res      ResT,
          int      BlockSize
>
constexpr se::scale_t Octree<DataT, ResT, BlockSize>::max_block_scale;

} // namespace se

#include "impl/octree_impl.hpp"

#endif // SE_OCTREE_HPP