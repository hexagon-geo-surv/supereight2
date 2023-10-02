/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MEMORY_POOL_HPP
#define SE_MEMORY_POOL_HPP

#include <Eigen/Core>
#include <boost/pool/pool.hpp>

namespace se {

template<typename NodeT, typename BlockT>
class MemoryPool {
    public:
    typedef typename NodeT::DataType DataType;

    MemoryPool() : node_buffer_(sizeof(NodeT)), block_buffer_(sizeof(BlockT))
    {
    }

    /**
     * \brief Allocate a node using its coordinates and size.
     *
     * \warning Should only be used for the root.
     */
    NodeT* allocateNode(const Eigen::Vector3i& node_coord, const int node_size)
    {
        return new (node_buffer_.malloc()) NodeT(node_coord, node_size, DataType());
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
    NodeT* allocateNode(NodeT* parent_ptr, const int child_idx, const DataType& init_data)
    {
        return new (node_buffer_.malloc()) NodeT(parent_ptr, child_idx, init_data);
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
    BlockT* allocateBlock(NodeT* parent_ptr, const int child_idx, const DataType& init_data)
    {
        return new (block_buffer_.malloc()) BlockT(parent_ptr, child_idx, init_data);
    }

    /**
     * \brief Delete a given node.
     */
    void deleteNode(NodeT* node_ptr)
    {
        node_ptr->~NodeT();
        node_buffer_.free(node_ptr);
    }

    /**
     * \brief Delete a given block.
     */
    void deleteBlock(BlockT* block_ptr)
    {
        block_ptr->~BlockT();
        block_buffer_.free(block_ptr);
    }

    boost::pool<> node_buffer_;  ///< The node buffer
    boost::pool<> block_buffer_; ///< The block buffer
};

} // namespace se

#endif // SE_MEMORY_POOL_HPP
