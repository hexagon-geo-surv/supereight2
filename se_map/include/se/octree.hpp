#ifndef SE_OCTREE_HPP
#define SE_OCTREE_HPP

#include <memory>
#include <deque>


#include "octant.hpp"

#include "utils/memory pool.hpp"
#include "utils/key_util.hpp"
#include "utils/setup_util.hpp"
#include "se/iterator.hpp"

namespace se {

/**
 * \brief The octree is the memory manager of the map.
 *        It is the only entity that is able to allocate and deallocate nodes and blocks.
 *        However it is not responsible to process the data in the nodes.
 *
 * \tparam DataT        The data struct stored in each voxel (or node for Res::Multi)
 * \tparam ResT         The resolution type (Res::Single, Res::Multi) defining if data can only stored at
 *                      a finest scale or any scale.
 * \tparam BlockSizeT   The size in voxels of a block. BlockSizeT in [1, (octree size) / 2]
 *                      Must be a power of two.
 */
template <typename DataT,
          Res      ResT       = Res::Single,
          unsigned BlockSizeT = 8
>
class Octree {
public:
  typedef std::shared_ptr<Octree<DataT, ResT, BlockSizeT> > Ptr;

  typedef DataT                          DataType;
  typedef Node<DataT, ResT>              NodeType;
  typedef Block<DataT, ResT, BlockSizeT> BlockType;

  typedef se::BoostMemoryPool<NodeType, BlockType> MemoryPool;

  // Compile-time constant expressions
  // # of voxels per side in a voxel block
  static constexpr unsigned int block_size = BlockSizeT;
  // The maximum scale of a block
  static constexpr se::scale_t max_block_scale = math::log2_const(BlockSizeT);

  /**
   * \brief The octree needs to be initialised during the allocation.
   *
   * \param[in] size    The size in [voxel] of the octree
   */
  Octree(const unsigned size);

  ~Octree() {};                               ///< TODO:
  Octree(const Octree&) = delete;             ///< Delete copy constructor
  Octree & operator=(const Octree&) = delete; ///< Delete copy assignment operator

  OctreeIterator<Octree<DataT, ResT, BlockSizeT>> begin();
  OctreeIterator<Octree<DataT, ResT, BlockSizeT>> end();

  NodeType* getRoot() { return root_; };

  /**
   * \brief Get a pointer with all allocated octant pointers (nodes and blocks).
   *
   * \return The list of all allocated octant pointers
   */
  se::vector<se::OctantBase*> getOctantBuffer();

  unsigned int getSize() { return size_; }

  /**
   * \brief Allocate a node for a given parent node.
   *
   * \param[in] parent_ptr  The parent of the node to be allocated
   * \param[in] child_idx   The child index of the node to be allocated
   *
   * \return Ture if the node has been newly allocated, False if it has already been allocated
   */
  bool allocate(NodeType*      parent_ptr,
                const unsigned child_idx);

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
  bool allocate(NodeType*        parent_ptr,
                const unsigned   child_idx,
                se::OctantBase*& child_ptr);     ///< Allocate child

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  /**
   * \brief Recursively add octant to octant_buffer_ until all octants are added
   *
   * \param[in] octant_ptr The node from which to start the recursive operation
   */
  void addOctantRecurse(se::OctantBase* octant_ptr);

  unsigned int size_;                         ///< The size in [voxel] of the octree
  NodeType* root_ = nullptr;  ///< The pointer to the root node of the octree
  std::vector<se::OctantBase*> octant_buffer_;

  MemoryPool memory_pool_;
};



template <typename DataT,
          Res      ResT,
          unsigned BlockSizeT
>
constexpr se::scale_t Octree<DataT, ResT, BlockSizeT>::max_block_scale;

} // namespace se

#include "impl/octree_impl.hpp"

#endif // SE_OCTREE_HPP