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

/**
 * \brief Get the child idx for a given child coordinate and pointer to the parent node.
 *
 * \warning The function requires a pointer to the actual node rather than the se::OctantBase*.
 *
 * \tparam NodeT
 * \param[in] child_coord   The voxel coordinates of the child
 * \param[in] parent_ptr    The pointer to the parent node
 *
 * \return The child index
 */
template<typename NodeT>
int get_child_idx(const Eigen::Vector3i& octant_coord, const NodeT* node_ptr);



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


/**
 * \brief The node type of the octant
 *
 * \tparam DataT
 * \tparam ResT
 */
template<typename DataT, Res ResT = Res::Single>
class Node : public OctantBase,
             public std::conditional<ResT == Res::Single,
                                     NodeSingleRes<DataT>,
                                     NodeMultiRes<DataT, Node<DataT, ResT>>>::type {
    public:
    typedef DataT DataType;

    /**
     * \brief Setup a node via its voxel coordinates and size
     *
     * \warning This function should only be used for the octrees root.
     *
     * \param[in] coord   The coordinates in [voxel] of the node
     * \param[in] size    The size in [voxel] of the node
     */
    Node(const Eigen::Vector3i& coord, const int size, const DataT& init_data);

    /**
     * \brief Setup a node via its parent and child index.
     *
     * \param[in] parent_ptr  The pointer to the parent node
     * \param[in] child_idx   The child index of the node
     */
    Node(Node* parent_ptr, const int child_idx, const DataT& init_data);

    /**
     * \brief Get the size in [voxel] of the node.
     *
     * \return The size of the node
     */
    int getSize() const;

    /**
     * \brief Get the pointer to one of the children of the node.
     *
     * \param[in] child_idx   The child index of the requested child
     *
     * \return The pointer to the child. nullptr if not allocated
     */
    const OctantBase* getChild(const int child_idx) const;

    /**
     * \brief Get the pointer to one of the children of the node.
     *
     * \param[in] child_idx   The child index of the requested child
     *
     * \return The pointer to the child. nullptr if not allocated
     */
    OctantBase* getChild(const int child_idx);

    /**
     * \brief Set the pointer of one of the children of the node.
     *
     * \param[in] child_idx   The child index of the child to be set
     * \param[in] child_ptr   The pointer to the child to be set
     */
    OctantBase* setChild(const int child_idx, OctantBase* child_ptr);

    /** Return the coordinates in voxels of the child with index \p child_idx. */
    Eigen::Vector3i getChildCoord(const int child_idx) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    std::array<OctantBase*, 8>
        children_ptr_; ///< Pointers to the eight children (should be all nullptr at initialisation due to smart pointers)
    const int size_; ///< The size in [voxel] of the node in comparision to the finest voxel
};

} // namespace se

#include "impl/node_impl.hpp"

#endif // SE_NODE_HPP
