#ifndef SE_MEMORY_POOL_HPP
#define SE_MEMORY_POOL_HPP

#include <Eigen/Dense>

#include <boost/pool/object_pool.hpp>

namespace se {

//class SimpleMemoryBuffer
//{
//
//};
//
//class SimpleMemoryPool
//{
//
//};


template <typename NodeT, typename BlockT>
class BoostMemoryPool
{
public:
  inline NodeT* allocateNode(const Eigen::Vector3i& node_coord,
                             const unsigned int     node_size)
  {
    return node_buffer_.construct(node_coord, node_size);
  }

  inline NodeT* allocateNode(NodeT*             parent_ptr,
                             const unsigned int child_idx)
  {
    return node_buffer_.construct(parent_ptr, child_idx);
  }

  inline BlockT* allocateBlock(NodeT*             parent_ptr,
                               const unsigned int child_idx)
  {
    return block_buffer_.construct(parent_ptr, child_idx);
  }

  inline void deleteNode(NodeT* node_ptr)
  {
    node_buffer_.destroy(node_ptr);
  }

  inline void deleteBlock(BlockT* block_ptr)
  {
    block_buffer_.destroy(block_ptr);
  }

  boost::object_pool<NodeT>  node_buffer_;
  boost::object_pool<BlockT> block_buffer_;
};

}
#endif //SE_MEMORY_POOL_HPP
