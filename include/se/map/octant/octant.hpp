/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTANT_HPP
#define SE_OCTANT_HPP

#include "se/common/math_util.hpp"
#include "se/map/data.hpp"
#include "se/map/utils/key_util.hpp"
#include "se/map/utils/setup_util.hpp"

namespace se {

/**
 * \brief This class only helps to dynamic cast the octant to the right type and builds the base of nodes and blocks.
 */
class OctantBase {
    /** Pointer to the parent octant. The parent pointer of the root octant is nullptr. */
    OctantBase* const parent_ptr_;

    public:
    /** The coordinates in voxels of the octant's vertex closest to the origin. All coordinates must
     * be non-negative.
     */
    const Eigen::Vector3i coord;

    /** The frame the octant was last updated at. */
    timestamp_t timestamp;

    /** The i-th bit of the mask must be set if the i-th child of the octant is allocated. */
    std::uint8_t child_mask;

    /**
     * \brief Setup a octant via its parent
     *
     * \param is_block    The block state of the octant
     * \param coord       The voxel coordinates of the octant
     * \param parent_ptr  The pointer to the parent of the octant
     */
    OctantBase(const bool is_block, const Eigen::Vector3i& coord, OctantBase* parent_ptr = nullptr);

    /**
     * \brief Verify if an octant is a block.
     *
     * \return True if the octant is a block, false otherwise
     */
    bool isBlock() const
    {
        return is_block_;
    }

    /**
     * \brief Get the parent pointer of the octant.
     *
     * \return The parent pointer of the octant
     */
    const OctantBase* parent() const
    {
        return parent_ptr_;
    }

    OctantBase* parent()
    {
        return parent_ptr_;
    }

    /**
     * \brief Test if this octant is a leaf, i.e. it has no children.
     */
    bool isLeaf() const
    {
        return child_mask == 0u;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    const bool is_block_;

    template<typename DerT, typename DatT, int BS>
    friend class BlockSingleRes;

    template<typename DatT, int BS, typename DerT>
    friend class BlockMultiRes;

    template<typename DatT, typename DerT>
    friend class NodeMultiRes;
};

} // namespace se

#include "block.hpp"
#include "node.hpp"

#endif // SE_OCTANT_HPP
