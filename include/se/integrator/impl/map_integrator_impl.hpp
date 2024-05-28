/*
 * SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

#include "se/integrator/updater/updater.hpp"

namespace se {
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
                          const Eigen::Isometry3f& T_WS,
                          const SensorT* const colour_sensor,
                          const se::Image<colour_t>* const colour_img,
                          const Eigen::Isometry3f* const T_SSc,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants);
};



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Eigen::Vector3f& ray_S,
                          const Eigen::Isometry3f& T_WS,
                          const unsigned int frame);
};



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayBatchImplD {
    template<typename SensorT, typename MapT>
    static void integrate(
        MapT& map,
        const SensorT& sensor,
        const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                          Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
            rayPoseBatch,
        const unsigned int frame,
        std::vector<const OctantBase*>* updated_octants);
};



/**
 * TSDF integration helper struct for partial function specialisation
 */
template<Res ResT>
struct IntegrateDepthImplD<se::Field::TSDF, ResT> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const se::Image<float>& depth_img,
                          const Eigen::Isometry3f& T_WS,
                          const SensorT* const colour_sensor,
                          const se::Image<colour_t>* const colour_img,
                          const Eigen::Isometry3f* const T_SSc,
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
        se::Updater updater(
            map, block_ptrs, sensor, depth_img, T_WS, colour_sensor, colour_img, T_SSc, frame);
        TOCK("update")

        if (updated_octants) {
            // TODO: Currently returning allocated, not updated octants. Remove non-updated blocks from
            // block_ptrs during the call to se::Updater::update() to return only updated octants.
            updated_octants->clear();
            updated_octants->reserve(block_ptrs.size());
            updated_octants->insert(updated_octants->end(), block_ptrs.begin(), block_ptrs.end());
        }
    }
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
                          const Eigen::Isometry3f& T_WS,
                          const SensorT* const colour_sensor,
                          const se::Image<colour_t>* const colour_img,
                          const Eigen::Isometry3f* const T_SSc,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants)
    {
        assert(sensor.model.imageWidth() == depth_img.width());
        assert(sensor.model.imageHeight() == depth_img.height());
        // Allocation
        TICK("allocation")
        VolumeCarver<MapT, SensorT> volume_carver(
            map,
            sensor,
            depth_img,
            T_WS,
            frame); //< process based on variance state and project inside
        se::VolumeCarverAllocation allocation_list = volume_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        se::Updater updater(map, sensor, depth_img, T_WS, colour_sensor, colour_img, T_SSc, frame);
        updater(allocation_list, updated_octants);
        TOCK("update")
    }
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Eigen::Vector3f& depth_img,
                          const Eigen::Isometry3f& T_WS,
                          const unsigned int frame,
                          std::vector<const OctantBase*>* updated_octants);
};

/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(
        MapT& map,
        const SensorT& sensor,
        const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                          Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
            rayPoseBatch,
        const unsigned int frame,
        std::vector<const OctantBase*>* updated_octants);
};


template<typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayImpl = IntegrateRayImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayBatchImpl = IntegrateRayBatchImplD<MapT::fld_, MapT::res_>;



template<typename SensorT, typename MapT>
void IntegrateRayImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const Eigen::Vector3f& ray_S,
    const Eigen::Isometry3f& T_WS,
    const unsigned int frame,
    std::vector<const OctantBase*>* updated_octants)
{
    TICK("Ray Integration")
    TICK("allocation-integration")
    se::RayIntegrator rayIntegrator(map, sensor, ray_S, T_WS, frame, updated_octants);
    rayIntegrator();
    TOCK("allocation-integration")
    TICK("propagateBlocksToCoarsestScale")
    rayIntegrator.propagateBlocksToCoarsestScale();
    TOCK("propagateBlocksToCoarsestScale")
    TICK("propagateToRoot")
    rayIntegrator.propagateToRoot();
    TOCK("propagateToRoot")
    rayIntegrator.updatedOctants(updated_octants);
    TOCK("Ray Integration")
}

template<typename SensorT, typename MapT>
void IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                      Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
        rayPoseBatch,
    const unsigned int frame,
    std::vector<const OctantBase*>* updated_octants)
{
    se::RayIntegrator<MapT, SensorT> rayIntegrator(
        map, sensor, rayPoseBatch[0].second, rayPoseBatch[0].first, frame, updated_octants);

    // do downsampling
    int skip_count = 0;

    for (size_t i = 0; i < rayPoseBatch.size(); i++) {
        TICK("Ray Integration")
        TICK("allocation-integration")
        if (rayIntegrator.resetIntegrator(rayPoseBatch[i].second, rayPoseBatch[i].first, frame)) {
            rayIntegrator();
        }
        else {
            skip_count++;
        }
        TOCK("allocation-integration")
        TOCK("Ray Integration")
    }
    // Do Propagations
    TICK("propagateBlocksToCoarsestScale")
    rayIntegrator.propagateBlocksToCoarsestScale();
    TOCK("propagateBlocksToCoarsestScale")
    TICK("propagateToRoot")
    rayIntegrator.propagateToRoot();
    TOCK("propagateToRoot")
    rayIntegrator.updatedOctants(updated_octants);
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
                                         const Eigen::Isometry3f& T_WS,
                                         const unsigned int frame)
{
    se::details::IntegrateDepthImpl<MapT>::template integrate<SensorT>(
        map_, sensor, depth_img, T_WS, nullptr, nullptr, nullptr, frame, nullptr);
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Isometry3f& T_WS,
                                         const unsigned int frame,
                                         std::vector<const OctantBase*>& updated_octants)
{
    se::details::IntegrateDepthImpl<MapT>::template integrate<SensorT>(
        map_, sensor, depth_img, T_WS, nullptr, nullptr, nullptr, frame, &updated_octants);
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Isometry3f& T_WS,
                                         const SensorT& colour_sensor,
                                         const se::Image<colour_t>& colour_img,
                                         const Eigen::Isometry3f& T_SSc,
                                         const unsigned int frame)
{
    se::details::IntegrateDepthImpl<MapT>::template integrate<SensorT>(
        map_, sensor, depth_img, T_WS, &colour_sensor, &colour_img, &T_SSc, frame, nullptr);
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT& sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Isometry3f& T_WS,
                                         const SensorT& colour_sensor,
                                         const se::Image<colour_t>& colour_img,
                                         const Eigen::Isometry3f& T_SSc,
                                         const unsigned int frame,
                                         std::vector<const OctantBase*>& updated_octants)
{
    se::details::IntegrateDepthImpl<MapT>::template integrate<SensorT>(map_,
                                                                       sensor,
                                                                       depth_img,
                                                                       T_WS,
                                                                       &colour_sensor,
                                                                       &colour_img,
                                                                       &T_SSc,
                                                                       frame,
                                                                       &updated_octants);
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRay(const SensorT& sensor,
                                       const Eigen::Vector3f& ray_S,
                                       const Eigen::Isometry3f& T_WS,
                                       const unsigned int frame)
{
    se::details::IntegrateRayImpl<MapT>::integrate(map_, sensor, ray_S, T_WS, frame, nullptr);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRay(const SensorT& sensor,
                                       const Eigen::Vector3f& ray_S,
                                       const Eigen::Isometry3f& T_WS,
                                       const unsigned int frame,
                                       std::vector<const OctantBase*>& updated_octants)
{
    se::details::IntegrateRayImpl<MapT>::integrate(
        map_, sensor, ray_S, T_WS, frame, &updated_octants);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRayBatch(
    const SensorT& sensor,
    const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                      Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
        rayPoseBatch,
    const unsigned int frame)
{
    se::details::IntegrateRayBatchImpl<MapT>::integrate(map_, sensor, rayPoseBatch, frame, nullptr);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRayBatch(
    const SensorT& sensor,
    const std::vector<std::pair<Eigen::Isometry3f, Eigen::Vector3f>,
                      Eigen::aligned_allocator<std::pair<Eigen::Isometry3f, Eigen::Vector3f>>>&
        rayPoseBatch,
    const unsigned int frame,
    std::vector<const OctantBase*>& updated_octants)
{
    se::details::IntegrateRayBatchImpl<MapT>::integrate(
        map_, sensor, rayPoseBatch, frame, &updated_octants);
}



} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
