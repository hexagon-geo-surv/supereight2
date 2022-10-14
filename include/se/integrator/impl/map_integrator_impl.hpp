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
                          const Image<rgb_t>* colour_img,
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
                          const Image<rgb_t>* colour_img,
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
        Updater updater(map, sensor, depth_img, colour_img, T_WS, frame);
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
                          const Image<rgb_t>* colour_img,
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
        Updater updater(map, sensor, depth_img, colour_img, T_WS, frame);
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
                          const Image<rgb_t>* colour_img,
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
        Updater updater(map, sensor, depth_img, colour_img, T_WS, frame);
        updater(allocation_list);
        TOCK("update")
    }
};



template<typename MapT>
using IntegrateImpl = IntegrateImplD<MapT::fld_, MapT::res_>;



} // namespace details



namespace integrator {

template<typename MapT, typename SensorT>
void integrate(MapT& map,
               const Image<float>& depth_img,
               const SensorT& sensor,
               const Eigen::Matrix4f& T_WS,
               const unsigned int frame)
{
    details::IntegrateImpl<MapT>::integrate(map, sensor, depth_img, nullptr, T_WS, frame);
}



template<typename MapT, typename SensorT>
typename std::enable_if_t<MapT::col_ == Colour::On> integrate(MapT& map,
                                                              const Image<float>& depth_img,
                                                              const Image<rgb_t>& colour_img,
                                                              const SensorT& sensor,
                                                              const Eigen::Matrix4f& T_WS,
                                                              const unsigned int frame)
{
    if (depth_img.width() != colour_img.width() || depth_img.height() != colour_img.height()) {
        std::ostringstream oss;
        oss << "depth (" << depth_img.width() << "x" << depth_img.height() << ") and colour ("
            << colour_img.width() << "x" << colour_img.height() << ") image dimensions differ";
        throw std::invalid_argument(oss.str());
    }
    details::IntegrateImpl<MapT>::integrate(map, sensor, depth_img, &colour_img, T_WS, frame);
}

} // namespace integrator

} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
