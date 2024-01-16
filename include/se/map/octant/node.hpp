/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_NODE_HPP
#define SE_NODE_HPP

namespace se {

// Forward Declaration
template<typename DataT, typename DerivedT>
class NodeMultiRes {
};



template<Field FldT, Colour ColB, Semantics SemB, typename DerivedT>
class NodeMultiRes<Data<FldT, ColB, SemB>, DerivedT> {
};



/**
 * \brief Multi-resolution data of a TSDF node.
 *        The node doesn't carry any data and returns the default data only.
 */
template<Colour ColB, Semantics SemB, typename DerivedT>
class NodeMultiRes<Data<Field::TSDF, ColB, SemB>, DerivedT> {
    public:
    typedef Data<Field::TSDF, ColB, SemB> DataType;
    typedef DeltaData<Field::TSDF, ColB, SemB> PropDataType;

    NodeMultiRes(const DataType&)
    {
    }

    const DataType& getData() const
    {
        static const DataType default_data = DataType();
        return default_data;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



/**
 * \brief Multi-resolution data of a Occupancy node.
 */
template<Colour ColB, Semantics SemB, typename DerivedT>
class NodeMultiRes<Data<Field::Occupancy, ColB, SemB>, DerivedT> {
    public:
    typedef Data<Field::Occupancy, ColB, SemB> DataType;

    /**
     * \brief Set the inital data of the node.
     *
     * \param init_data   The initial data of the node.
     */
    NodeMultiRes(const DataType& init_data)
    {
        data_ = init_data;
        min_data_ = init_data;
    }

    /**
     * \brief Get the node data. If the node is not observed and not a leaf the default data is
     * returned.
     */
    const DataType& getData() const
    {
        static const DataType default_data = DataType();
        return (data_.observed && underlying()->isLeaf()) ? data_ : default_data;
    }

    const DataType& getMinData() const
    {
        return min_data_;
    }

    /**
     * \brief Get the max data of the node.
     *
     * \warning The data is not returned by reference as it's the case for the blocks.
     *
     * \return The max data of the node.
     */
    const DataType getMaxData() const
    {
        return data_;
    }

    /**
     * \brief Set the data / max data of the node.
     *
     * \param[in] data    The data to be set
     */
    void setData(const DataType& data)
    {
        data_ = data;
    }

    void setMinData(const DataType& data)
    {
        min_data_ = data;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    DataType data_; ///< Holds the max data of the node.
                    ///< At the leaf of the tree the nodes max data is equivalent to its data
    // The minimum data among all the Node's children.
    DataType min_data_;

    private:
    const DerivedT* underlying() const
    {
        return static_cast<const DerivedT*>(this);
    }
};



/**
 * \brief Single-resolution data of a node.
 *        The node doesn't carry any data and returns the default data only.
 */
template<typename DataT>
class NodeSingleRes {
    public:
    NodeSingleRes(const DataT&)
    {
    }

    const DataT& getData() const
    {
        static const DataT default_data = DataT();
        return default_data;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



/** An intermediate node of an se::Octree.
 *
 * An se::Node is never a leaf in TSDF octrees but may be a leaf in occupancy octrees.
 *
 * \tparam DataT The type of data stored in the octree.
 * \tparam ResT  The value of se::Res for the octree.
 */
template<typename DataT, Res ResT = Res::Single>
class Node : public OctantBase,
             public std::conditional<ResT == Res::Single,
                                     NodeSingleRes<DataT>,
                                     NodeMultiRes<DataT, Node<DataT, ResT>>>::type {
    public:
    typedef DataT DataType;

    /** Construct a node at coordinates \p coord in voxels, with an edge length \p size in voxels
     * and initialize its data with \p init_data.
     *
     * \warning This constructor should only be used for the octree root node as it doesn't set the
     * parent pointer.
     */
    Node(const Eigen::Vector3i& coord, const int size, const DataT& init_data);

    /** Construct the child node of \p parent_ptr with index \p child_idx and initialize its data
     * with \p init_data. The value of \p child_idx must be in the interval [0, 7] inclusive.
     */
    Node(Node* parent_ptr, const int child_idx, const DataT& init_data);

    /** Return the edge length of the node in voxels */
    int getSize() const;

    /** Return a pointer to the node child with index \p child_idx. The value of \p child_idx must
     * be in the interval [0, 7] inclusive. Returns nullptr if the child is not allocated.
     */
    const OctantBase* getChild(const int child_idx) const;

    /** A non-const overload of the previous member function. */
    OctantBase* getChild(const int child_idx);

    /** Set the node child with index \p child_idx to \p child_ptr and return a pointer to the old
     * child. The value of \p child_idx must be in the interval [0, 7] inclusive.
     */
    OctantBase* setChild(const int child_idx, OctantBase* child_ptr);

    /** Return the coordinates in voxels of the child with index \p child_idx. */
    Eigen::Vector3i getChildCoord(const int child_idx) const;

    /** Return the index of the child of the node with coordinates \p child_coord. The returned
     * index is in the interval [0, 7] inclusive.
     *
     * \warning Will return garbage if \p child_coord doesn't correspond to the coordinates of a
     * child of the node.
     */
    int getChildIdx(const Eigen::Vector3i& child_coord);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    // Pointers to the eight node children. Must be nullptr for unallocated children.
    std::array<OctantBase*, 8> children_ptr_;
    // The edge length of the node in voxels.
    const int size_;
};

} // namespace se

#include "impl/node_impl.hpp"

#endif // SE_NODE_HPP
