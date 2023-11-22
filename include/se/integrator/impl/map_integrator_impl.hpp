/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
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
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Eigen::Vector3f& ray_S,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT, se::Res ResT>
struct IntegrateRayBatchImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>,
                              Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                          const unsigned int frame);
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



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const Eigen::Vector3f& depth_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};

/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template<>
struct IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>,
                              Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                          const unsigned int frame);
};


template<typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayImpl = IntegrateRayImplD<MapT::fld_, MapT::res_>;

template<typename MapT>
using IntegrateRayBatchImpl = IntegrateRayBatchImplD<MapT::fld_, MapT::res_>;


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



template<typename SensorT, typename MapT>
void IntegrateRayImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const Eigen::Vector3f& ray_S,
    const Eigen::Matrix4f& T_WS,
    const unsigned int frame)
{

    omp_set_num_threads(3);
    TICK("Ray Integration")
    TICK("allocation-integration")
    se::RayIntegrator rayIntegrator(map,sensor,ray_S,T_WS,frame);
    rayIntegrator();
    TOCK("allocation-integration")
    TICK("propagateBlocksToCoarsestScale")
    rayIntegrator.propagateBlocksToCoarsestScale();
    TOCK("propagateBlocksToCoarsestScale")
    TICK("propagateToRoot")
    rayIntegrator.propagateToRoot();
    TOCK("propagateToRoot")
    TOCK("Ray Integration")
}

template<typename SensorT, typename MapT>
void IntegrateRayBatchImplD<se::Field::Occupancy, se::Res::Multi>::integrate(
    MapT& map,
    const SensorT& sensor,
    const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
    const unsigned int frame)
{
    se::RayIntegrator<MapT,SensorT> rayIntegrator(map, sensor, rayPoseBatch.at(0).second, rayPoseBatch.at(0).first, frame);

    omp_set_num_threads(3); // ToDo: check if this holds for later functions?
    // do downsampling
    int skip_count = 0;

    for(size_t i = 0; i < rayPoseBatch.size(); i++){
        TICK("Ray Integration")
        TICK("allocation-integration")
        if(rayIntegrator.resetIntegrator(rayPoseBatch.at(i).second, rayPoseBatch.at(i).first, frame)){
            rayIntegrator();
        }else{
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



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRay(const SensorT& sensor,
                                       const Eigen::Vector3f& ray_S,
                                       const Eigen::Matrix4f& T_WS,
                                       const unsigned int frame)
{
    se::details::IntegrateRayImpl<MapT>::integrate(map_, sensor, ray_S, T_WS, frame);
}

template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateRayBatch(const SensorT& sensor,
                                            const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                                            /*const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& rayBatch,
                                            const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poseBatch,*/
                                            const unsigned int frame)
{
    //se::details::IntegrateRayBatchImpl<MapT>::integrate(map_, sensor, rayBatch, poseBatch, frame);
    se::details::IntegrateRayBatchImpl<MapT>::integrate(map_, sensor, rayPoseBatch, frame);
}



} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
