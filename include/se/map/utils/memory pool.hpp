#ifndef SE_MEMORY_POOL_HPP
#define SE_MEMORY_POOL_HPP

#include <Eigen/Dense>

#include <boost/pool/object_pool.hpp>

namespace se {



template <typename NodeT, typename BlockT>
class BoostMemoryPool
{
public:
  /**
   * \brief Allocate a node using its coordinates and size.
   *
   * \warning Should only be used for the root.
   */
  inline NodeT* allocateNode(const Eigen::Vector3i& node_coord,
                             const unsigned int     node_size)
  {
    return node_buffer_.construct(node_coord, node_size);
  }

  /**
   * \brief Allocate a node using its parent and child index.
   */
  inline NodeT* allocateNode(NodeT*             parent_ptr,
                             const unsigned int child_idx)
  {
    return node_buffer_.construct(parent_ptr, child_idx);
  }

  /**
   * \brief Allocate a block using its parent and child index.
   */
  inline BlockT* allocateBlock(NodeT*             parent_ptr,
                               const unsigned int child_idx)
  {
    return block_buffer_.construct(parent_ptr, child_idx);
  }

  /**
   * \brief Delete a given node.
   */
  inline void deleteNode(NodeT* node_ptr)
  {
    node_buffer_.destroy(node_ptr);
  }

  /**
   * \brief Delete a given block.
   */
  inline void deleteBlock(BlockT* block_ptr)
  {
    block_buffer_.destroy(block_ptr);
  }

  boost::object_pool<NodeT>  node_buffer_;  ///< The node buffer
  boost::object_pool<BlockT> block_buffer_; ///< The block buffer
};

}

#endif // SE_MEMORY_POOL_HPP

