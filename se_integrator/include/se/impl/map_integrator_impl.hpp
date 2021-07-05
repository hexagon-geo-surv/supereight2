#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP



#include "se/utils/math_util.hpp"

#include "se/allocator/raycast_carver.hpp"
#include "se/allocator/volume_carver.hpp"

#include "se/updater/updater.hpp"



namespace se {



static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord,
                                               const int              octant_size)
{
  return octant_coord.cast<float>() + se::sample_offset_frac * octant_size;
}



namespace details {



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT,
         se::Res ResT
>
struct IntegrateDepthImplD
{
  template<typename SensorT,
           typename MapT,
           typename ConfigT
  >
  static void integrate(const se::Image<se::depth_t>& depth_img,
                        const SensorT&                sensor,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
                        MapT&                         map,
                        ConfigT&                      /* config */); // TODO:
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>
{
  template<typename SensorT,
           typename MapT,
           typename ConfigT
  >
  static void integrate(const se::Image<se::depth_t>& depth_img,
                        const SensorT&                sensor,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
                        MapT&                         map,
                        ConfigT&                      /* config */);
};



/**
 * Multi-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>
{
  template<typename SensorT,
           typename MapT,
           typename ConfigT
  >
  static void integrate(const se::Image<se::depth_t>& depth_img,
                        const SensorT&                sensor,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
                        MapT&                         map,
                        ConfigT&                      /* config */);
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>
{
  template<typename SensorT,
           typename MapT,
           typename ConfigT
  >
  static void integrate(const se::Image<se::depth_t>& depth_img,
                        const SensorT&                sensor,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
                        MapT&                         map,
                        ConfigT&                      /* config */);
};



template <typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;



template<se::Field FldT,
         se::Res ResT
>
template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<FldT, ResT>::integrate(const se::Image<se::depth_t>& depth_img,
                                                const SensorT&                sensor,
                                                const Eigen::Matrix4f&        T_MS,
                                                const unsigned int            frame,
                                                MapT&                         map,
                                                ConfigT&                      config)
{
}



template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                      const SensorT&                sensor,
                                                                      const Eigen::Matrix4f&        T_MS,
                                                                      const unsigned int            frame,
                                                                      MapT&                         map,
                                                                      ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getRes() * map.getDataConfig().truncation_boundary_factor;
  const float band                = 2.f * truncation_boundary;

  // Allocation
  se::RaycastCarver raycast_carver(map, sensor, depth_img, T_MS, frame);
  std::vector<OctantBase*> block_ptrs = raycast_carver.allocateBand(band); //

  // Update
  se::Updater updater(map, sensor, depth_img, T_MS, frame);
  updater(block_ptrs, truncation_boundary);
}




template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                     const SensorT&                sensor,
                                                                     const Eigen::Matrix4f&        T_MS,
                                                                     const unsigned int            frame,
                                                                     MapT&                         map,
                                                                     ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getRes() * map.getDataConfig().truncation_boundary_factor;
  const float band                = 2.f * truncation_boundary;

  // Allocation
  se::RaycastCarver raycast_carver(map, sensor, depth_img, T_MS, frame);
  std::vector<OctantBase*> block_ptrs = raycast_carver.allocateBand(band);

  // Update
  se::Updater updater(map, sensor, depth_img, T_MS, frame);
  updater(block_ptrs, truncation_boundary);
}



template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                          const SensorT&                sensor,
                                                                          const Eigen::Matrix4f&        T_MS,
                                                                          const unsigned int            frame,
                                                                          MapT&                         map,
                                                                          ConfigT&                      /* config */)
{
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

  // Allocation
  VolumeCarver<MapT, SensorT> volume_carver(map, sensor, depth_img, T_SM, frame); //< process based on variance state and project inside
  se::VolumeCarverAllocation allocation_list = volume_carver.allocateFrustum();

  // Update
  std::vector<std::set<se::OctantBase*>> node_set(map.getOctree()->getBlockDepth());
  std::vector<se::OctantBase*>           freed_block_list;
  MultiresOFusionUpdater<MapT, SensorT> updater(depth_img, map, sensor, T_SM, frame, node_set, freed_block_list);

#pragma omp parallel for
  for (unsigned int i = 0; i < allocation_list.node_list.size(); ++i)
  {
    auto node_ptr = static_cast<typename MapT::OctreeType::NodeType*>(allocation_list.node_list[i]);
    const int depth = map.getOctree()->getMaxScale() - se::math::log2_const(node_ptr->getSize());
    updater.freeNodeRecurse(allocation_list.node_list[i], depth);
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i)
  {
    updater.updateBlock(allocation_list.block_list[i],
                        allocation_list.variance_state_list[i] == se::VarianceState::Constant,
                        allocation_list.projects_inside_list[i]);
  }

  /// Propagation
#pragma omp parallel for
  for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i)
  {
    updater::propagateBlockToCoarsestScale<typename MapT::OctreeType::BlockType>(allocation_list.block_list[i]);
  }
#pragma omp parallel for
  for (unsigned int i = 0; i < freed_block_list.size(); ++i)
  {
    updater::propagateBlockToCoarsestScale<typename MapT::OctreeType::BlockType>(freed_block_list[i]);
  }

  updater.propagateToRoot(allocation_list.block_list);
}



} // namespace details



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT&                   map,
                                   const IntegratorConfig& config)
    : map_(map), config_(config)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const se::Image<se::depth_t>& depth_img,
                                         const SensorT&                sensor,
                                         const Eigen::Matrix4f&        T_MS,
                                         const unsigned int            frame)
{
  se::details::IntegrateDepthImpl<MapT>::integrate(depth_img, sensor, T_MS, frame, map_, config_);
}



} // namespace se

#endif //SE_MAP_INTEGRATOR_IMPL_HPP
