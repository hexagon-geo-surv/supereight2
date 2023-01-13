/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_FETCHER_IMPL_HPP
#define SE_FETCHER_IMPL_HPP



namespace se {
namespace fetcher {



template<typename OctreeT>
inline se::OctantBase* octant(const Eigen::Vector3i& octant_coord,
                              const se::scale_t scale_desired,
                              se::OctantBase* base_parent_ptr)
{
    return const_cast<OctantBase*>(octant<OctreeT>(
        octant_coord, scale_desired, const_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
inline const se::OctantBase* octant(const Eigen::Vector3i& octant_coord,
                                    const se::scale_t scale_desired,
                                    const se::OctantBase* base_parent_ptr)
{
    const typename OctreeT::NodeType* parent_ptr =
        static_cast<const typename OctreeT::NodeType*>(base_parent_ptr);
    int child_size = parent_ptr->getSize() >> 1;
    const se::OctantBase* child_ptr = nullptr;

    int size_desired =
        std::max(1 << scale_desired, OctreeT::BlockType::getSize()); // Not smaller than block size
    for (; child_size >= size_desired; child_size = child_size >> 1) {
        se::idx_t child_idx = ((octant_coord.x() & child_size) > 0)
            + 2 * ((octant_coord.y() & child_size) > 0) + 4 * ((octant_coord.z() & child_size) > 0);
        child_ptr = parent_ptr->getChild(child_idx);
        if (!child_ptr) {
            return nullptr;
        }
        parent_ptr = static_cast<const typename OctreeT::NodeType*>(child_ptr);
    }
    return child_ptr;
}



template<typename OctreeT>
inline se::OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                                     const se::scale_t scale_desired,
                                     se::OctantBase* base_parent_ptr)
{
    return const_cast<OctantBase*>(finest_octant<OctreeT>(
        octant_coord, scale_desired, const_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
inline const se::OctantBase* finest_octant(const Eigen::Vector3i& octant_coord,
                                           const se::scale_t scale_desired,
                                           const se::OctantBase* base_parent_ptr)
{
    const typename OctreeT::NodeType* parent_ptr =
        static_cast<const typename OctreeT::NodeType*>(base_parent_ptr);
    int child_size = parent_ptr->getSize() >> 1;
    const se::OctantBase* child_ptr = nullptr;

    int size_desired =
        std::max(1 << scale_desired, OctreeT::BlockType::getSize()); // Not smaller than block size
    for (; child_size >= size_desired; child_size = child_size >> 1) {
        se::idx_t child_idx = ((octant_coord.x() & child_size) > 0)
            + 2 * ((octant_coord.y() & child_size) > 0) + 4 * ((octant_coord.z() & child_size) > 0);
        child_ptr = parent_ptr->getChild(child_idx);

        if (!child_ptr) {
            const se::OctantBase* leaf_ptr = (parent_ptr->isLeaf()) ? parent_ptr : nullptr;
            return leaf_ptr; // leaf is either a block or a parent with no children!
        }

        parent_ptr = static_cast<const typename OctreeT::NodeType*>(child_ptr);
    }

    return child_ptr;
}



template<typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& block_coord, se::OctantBase* base_parent_ptr)
{
    typename OctreeT::NodeType* parent_ptr =
        static_cast<typename OctreeT::NodeType*>(base_parent_ptr);
    int child_size = parent_ptr->getSize() >> 1;
    se::OctantBase* child_ptr = nullptr;

    for (; child_size >= OctreeT::BlockType::getSize(); child_size = child_size >> 1) {
        se::idx_t child_idx = ((block_coord.x() & child_size) > 0)
            + 2 * ((block_coord.y() & child_size) > 0) + 4 * ((block_coord.z() & child_size) > 0);
        child_ptr = parent_ptr->getChild(child_idx);
        if (!child_ptr) {
            return nullptr;
        }
        parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
    }

    return child_ptr;
}



template<typename OctreeT>
inline OctantBase* leaf(const Eigen::Vector3i& leaf_coord, OctantBase* base_parent_ptr)
{
    return const_cast<OctantBase*>(
        leaf<OctreeT>(leaf_coord, const_cast<const OctantBase*>(base_parent_ptr)));
}



template<typename OctreeT>
inline const OctantBase* leaf(const Eigen::Vector3i& leaf_coord, const OctantBase* base_parent_ptr)
{
    assert(base_parent_ptr);
    const OctantBase* octant = base_parent_ptr;
    while (!octant->isLeaf()) {
        typedef typename OctreeT::NodeType NodeType;
        // Only Nodes can be non-leaves so the following cast is safe.
        const NodeType* node = static_cast<const NodeType*>(octant);
        const OctantBase* child = node->getChild(get_child_idx(leaf_coord, node));
        if (!child) {
            return octant;
        }
        octant = child;
    }
    return octant;
}

} // namespace fetcher
} // namespace se

#endif // SE_FETCHER_IMPL_HPP
