/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCAST_CARVER_HPP
#define SE_RAYCAST_CARVER_HPP

#include <Eigen/Geometry>
#include <se/common/timings.hpp>
#include <se/image/image.hpp>
#include <se/map/octant/octant.hpp>
#include <se/map/octree/allocator.hpp>
#include <se/map/octree/fetcher.hpp>
#include <se/map/octree/iterator.hpp>
#include <set>

namespace se {
namespace fetcher {



template<typename MapT, typename SensorT>
inline std::vector<se::OctantBase*>
frustum(MapT& map, const SensorT& sensor, const Eigen::Isometry3f& T_WS);



} // namespace fetcher



/** Allocator used for TSDF mapping. It allocates blocks (i.e. octree leaves) in a band around the
 * surface by performing ray marching along the depth measurement rays.
 */
template<typename MapT, typename SensorT>
class RaycastCarver {
    public:
    /** Constructs a RaycastCarver but doesn't perform any allocations yet. Call
     * se::RaycastCarver::operator()() to perform the actual allocations.
     *
     * \param[in] map         The map to be updated.
     * \param[in] sensor      The sensor model used to capture \p depth_img.
     * \param[in] depth_img   The depth image to be integrated into \p map.
     * \param[in] T_WS        The transformation from the sensor frame S that \p depth_img was
     *                        captured from to the world frame W.
     * \param[in] timestamp   The timestamp of the frame to be integrated. Currently unused.
     */
    RaycastCarver(MapT& map,
                  const SensorT& sensor,
                  const se::Image<float>& depth_img,
                  const Eigen::Isometry3f& T_WS,
                  const timestamp_t timestamp);

    /** Performs the necessary allocations and returns the allocated blocks. */
    std::vector<se::OctantBase*> operator()();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    MapT& map_;
    typename MapT::OctreeType& octree_;
    const SensorT& sensor_;
    const se::Image<float>& depth_img_;
    const Eigen::Isometry3f& T_WS_;
    const float band_;
};

} // namespace se

#include "impl/raycast_carver_impl.hpp"

#endif // SE_RAYCAST_CARVER_HPP
