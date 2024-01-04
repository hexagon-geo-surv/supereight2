/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SUBMAP_HPP
#define SE_SUBMAP_HPP

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <memory>
#include <unordered_map>
#include <vector>

namespace se {

/** Stores an se::Map and an associated transformation from the submap to the world frame. Useful
 * for integrating supereight2 maps into a submapping framework.
 */
template<typename MapT>
struct Submap {
    std::shared_ptr<MapT> map;

    /** The pose of the submap frame K expressed in the world frame W. */
    Eigen::Isometry3f T_WK = Eigen::Isometry3f::Identity();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename MapT>
using SubmapVec = std::vector<Submap<MapT>, Eigen::aligned_allocator<Submap<MapT>>>;

template<typename KeyT, typename MapT>
using SubmapUnordMap =
    std::unordered_map<KeyT,
                       Submap<MapT>,
                       std::hash<KeyT>,
                       std::equal_to<KeyT>,
                       Eigen::aligned_allocator<std::pair<const KeyT, Submap<MapT>>>>;

} // namespace se

#endif // SE_SUBMAP_HPP
