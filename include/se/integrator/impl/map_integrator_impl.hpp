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
    assert(octant_size > 0);
    return octant_coord.cast<float>() + se::sample_offset_frac * octant_size;
}



namespace details {



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateDepthImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants);
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants);
};



/**
 * Multi-res TSDF integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants);
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants);
};



template<typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;



template<se::Field FldT, se::Res ResT>
template<typename SensorT, typename MapT>
void IntegrateDepthImplD<FldT, ResT>::integrate(
    MapT& /* map */,
    const SensorT& /* sensor */,
    const se::Image<float>& /* depth_img */,
    const Eigen::Matrix4f& /* T_WS */,
    const unsigned int /* frame */,
    std::vector<const OctantBase*>* /* updated_octants */)
{
}



template<typename SensorT, typename MapT>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(
    MapT& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame,
    std::vector<const OctantBase*>* updated_octants)
{
    assert(sensor.model.imageWidth() == depth_img.width());
    assert(sensor.model.imageHeight() == depth_img.height());
    // Allocation
    TICK("allocation")
    se::RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
    std::vector<OctantBase*> block_ptrs = raycast_carver();
    TOCK("allocation")

    // Update
    TICK("update")
    se::Updater updater(map, sensor, depth_img, T_WS, frame);
    updater(block_ptrs);
    TOCK("update")

    if (updated_octants) {
        // TODO: Currently returning allocated, not updated octants. Remove non-updated blocks from
        // block_ptrs during the call to se::Updater::update() to return only updated octants.
        updated_octants->clear();
        updated_octants->reserve(block_ptrs.size());
        updated_octants->insert(updated_octants->end(), block_ptrs.begin(), block_ptrs.end());
    }
}



template<typename SensorT, typename MapT>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame,
    std::vector<const OctantBase*>* updated_octants)
{
    assert(sensor.model.imageWidth() == depth_img.width());
    assert(sensor.model.imageHeight() == depth_img.height());
    // Allocation
    TICK("allocation")
    se::RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
    std::vector<OctantBase*> block_ptrs = raycast_carver();
    TOCK("allocation")

    // Update
    TICK("update")
    se::Updater updater(map, sensor, depth_img, T_WS, frame);
    updater(block_ptrs);
    TOCK("update")

    if (updated_octants) {
        // TODO: Currently returning allocated, not updated octants. Remove non-updated blocks from
        // block_ptrs during the call to se::Updater::update() to return only updated octants.
        updated_octants->clear();
        updated_octants->reserve(block_ptrs.size());
        updated_octants->insert(updated_octants->end(), block_ptrs.begin(), block_ptrs.end());
    }
}



template<typename SensorT, typename MapT>
void IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const se::Image<float>& depth_img,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame,
    std::vector<const OctantBase*>* updated_octants)
{
    assert(sensor.model.imageWidth() == depth_img.width());
    assert(sensor.model.imageHeight() == depth_img.height());
    // Allocation
    TICK("allocation")
    VolumeCarver<MapT, SensorT> volume_carver(
        map, sensor, depth_img, T_WS, frame); //< process based on variance state and project inside
    se::VolumeCarverAllocation allocation_list = volume_carver();
    TOCK("allocation")

    // Update
    TICK("update")
    se::Updater updater(map, sensor, depth_img, T_WS, frame);
    updater(allocation_list);
    TOCK("update")

    if (updated_octants) {
        // TODO: Currently returning allocated, not updated octants. Remove non-updated octants from
        // allocation_list during the call to se::Updater::update() to return only updated octants.
        updated_octants->clear();
        updated_octants->reserve(allocation_list.node_list.size()
                                 + allocation_list.block_list.size());
        updated_octants->insert(updated_octants->end(),
                                allocation_list.node_list.begin(),
                                allocation_list.node_list.end());
        updated_octants->insert(updated_octants->end(),
                                allocation_list.block_list.begin(),
                                allocation_list.block_list.end());
    }
}



} // namespace details



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT& map) : map_(map)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Matrix4f& T_WS,
                                         const unsigned int frame)
{
    se::details::IntegrateDepthImpl<MapT>::integrate(map_, sensor, depth_img, T_WS, frame, nullptr);
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Matrix4f& T_WS,
                                         const unsigned int frame,
                                         std::vector<const OctantBase*>& updated_octants)
{
    se::details::IntegrateDepthImpl<MapT>::integrate(
        map_, sensor, depth_img, T_WS, frame, &updated_octants);
}



} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
