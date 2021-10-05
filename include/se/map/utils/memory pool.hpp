/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MEMORY_POOL_HPP
#define SE_MEMORY_POOL_HPP

#include <Eigen/Dense>

#include <boost/pool/object_pool.hpp>

namespace se {



template <typename NodeT, typename BlockT>
class BoostMemoryPool
{
public:
  typedef typename NodeT::DataType DataType;

  /**
   * \brief Allocate a node using its coordinates and size.
   *
   * \warning Should only be used for the root.
   */
  inline NodeT* allocateNode(const Eigen::Vector3i& node_coord,
                             const unsigned int     node_size)
  {
    return node_buffer_.construct(node_coord, node_size, DataType());
  }

  /**
   * \brief Allocate a node using its parent and child index.
   *
   * \param[in] parent_ptr The pointer to the parent node
   * \param[in] child_idx  The index of the child to be allocated
   * \param[in] init_data  The data to initialise the child node with
   *
   * \return The pointer to the child node.
   */
  inline NodeT* allocateNode(NodeT*         parent_ptr,
                             const int      child_idx,
                             const DataType init_data)
  {
    return node_buffer_.construct(parent_ptr, child_idx, init_data);
  }

  /**
   * \brief Allocate a block using its parent and child index.
   *
   * \param[in] parent_ptr The pointer to the parent node
   * \param[in] child_idx  The index of the child to be allocated
   * \param[in] init_data  The data to initialise the child node with
   *
   * \return The pointer to the child block.
   */
  inline BlockT* allocateBlock(NodeT*         parent_ptr,
                               const int      child_idx,
                               const DataType init_data)
  {
    return block_buffer_.construct(parent_ptr, child_idx, init_data);
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

