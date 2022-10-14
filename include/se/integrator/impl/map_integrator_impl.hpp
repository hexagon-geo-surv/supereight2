/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

#include "se/integrator/updater/updater.hpp"

namespace se {



static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord,
                                               const int octant_size)
{
    return octant_coord.cast<float>() + sample_offset_frac * octant_size;
}



namespace details {



/**
 * Integration helper struct for partial function specialisation
 */
template<Field FldT, Res ResT>
struct IntegrateImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
    // No implementation needed since all potential overloads have been implemented.
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template<>
struct IntegrateImplD<Field::TSDF, Res::Single> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame)
    {
        // Allocation
        TICK("allocation")
        RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
        std::vector<OctantBase*> block_ptrs = raycast_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        Updater updater(map, sensor, depth_img, T_WS, frame);
        updater(block_ptrs);
        TOCK("update")
    }
};



/**
 * Multi-res TSDF integration helper struct for partial function specialisation
 */
template<>
struct IntegrateImplD<Field::TSDF, Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame)
    {
        // Allocation
        TICK("allocation")
        RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
        std::vector<OctantBase*> block_ptrs = raycast_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        Updater updater(map, sensor, depth_img, T_WS, frame);
        updater(block_ptrs);
        TOCK("update")
    }
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateImplD<Field::Occupancy, Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame)
    {
        // Allocation
        TICK("allocation")
        VolumeCarver<MapT, SensorT> volume_carver(
            map,
            sensor,
            depth_img,
            T_WS,
            frame); //< process based on variance state and project inside
        VolumeCarverAllocation allocation_list = volume_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        Updater updater(map, sensor, depth_img, T_WS, frame);
        updater(allocation_list);
        TOCK("update")
    }
};



template<typename MapT>
using IntegrateImpl = IntegrateImplD<MapT::fld_, MapT::res_>;



} // namespace details



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT& map) : map_(map)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const Image<float>& depth_img,
                                         const Eigen::Matrix4f& T_WS,
                                         const unsigned int frame)
{
    details::IntegrateImpl<MapT>::integrate(map_, sensor, depth_img, T_WS, frame);
}



} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
