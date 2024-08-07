/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_NODE_IMPL_HPP
#define SE_NODE_IMPL_HPP

namespace se {

template<typename DataT, Res ResT>
Node<DataT, ResT>::Node(const Eigen::Vector3i& coord, const int size, const DataT& init_data) :
        OctantBase(coord, false, nullptr),
        NodeData<DataT, Node<DataT, ResT>>(init_data),
        size_(size)
{
    assert(math::is_power_of_two(size));
    children_ptr_.fill(nullptr);
}



template<typename DataT, Res ResT>
Node<DataT, ResT>::Node(Node* parent_ptr, const int child_idx, const DataT& init_data) :
        OctantBase(parent_ptr->coord
                       + (parent_ptr->size_ >> 1)
                           * Eigen::Vector3i((1 & child_idx) > 0,
                                             (2 & child_idx) > 0,
                                             (4 & child_idx) > 0),
                   false,
                   parent_ptr),
        NodeData<DataT, Node<DataT, ResT>>(init_data),
        size_(parent_ptr->size_ >> 1)
{
    assert(child_idx >= 0);
    assert(static_cast<size_t>(child_idx) < children_ptr_.size());
    children_ptr_.fill(nullptr);
}



template<typename DataT, Res ResT>
int Node<DataT, ResT>::getSize() const
{
    return size_;
}



template<typename DataT, Res ResT>
OctantBase* Node<DataT, ResT>::getChild(const int child_idx)
{
    assert(child_idx >= 0);
    assert(static_cast<size_t>(child_idx) < children_ptr_.size());
    return children_ptr_[child_idx];
}



template<typename DataT, Res ResT>
const OctantBase* Node<DataT, ResT>::getChild(const int child_idx) const
{
    assert(child_idx >= 0);
    assert(static_cast<size_t>(child_idx) < children_ptr_.size());
    return children_ptr_[child_idx];
}



template<typename DataT, Res ResT>
OctantBase* Node<DataT, ResT>::setChild(const int child_idx, OctantBase* child_ptr)
{
    assert(child_idx >= 0);
    assert(static_cast<size_t>(child_idx) < children_ptr_.size());
    // Set or clear the corresponding bit in OctantBase::child_mask.
    if (child_ptr) {
        child_mask |= 1 << child_idx;
    }
    else {
        child_mask &= ~(1 << child_idx);
    }
    std::swap(child_ptr, children_ptr_[child_idx]);
    return child_ptr;
}



template<typename DataT, Res ResT>
Eigen::Vector3i Node<DataT, ResT>::getChildCoord(const int child_idx) const
{
    assert(child_idx >= 0);
    assert(static_cast<size_t>(child_idx) < children_ptr_.size());
    const Eigen::Vector3i child_offset(
        (child_idx & 1) != 0, (child_idx & 2) != 0, (child_idx & 4) != 0);
    const int child_size = getSize() / 2;
    return coord + child_size * child_offset;
}



template<typename DataT, Res ResT>
int Node<DataT, ResT>::getChildIdx(const Eigen::Vector3i& child_coord) const
{
    assert(keyops::is_child(keyops::encode_key(coord, math::log2_const(getSize())),
                            keyops::encode_key(child_coord, math::log2_const(getSize() / 2)))
           && "child_coord must correspond to a child of the node");
    const Eigen::Vector3i offset = child_coord - coord;
    const int child_size = getSize() / 2;
    return ((offset.x() & child_size) > 0) + 2 * ((offset.y() & child_size) > 0)
        + 4 * ((offset.z() & child_size) > 0);
}

} // namespace se

#endif // SE_NODE_IMPL_HPP
