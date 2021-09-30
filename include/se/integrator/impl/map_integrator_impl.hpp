#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

#include "se/integrator/updater/updater.hpp"

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
           typename MapT
  >
  static void integrate(MapT&                   map,
                        const SensorT&          sensor,
                        const se::Image<float>& depth_img,
                        const Eigen::Matrix4f&  T_WS,
                        const unsigned int      frame); // TODO:
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>
{
  template<typename SensorT,
           typename MapT
  >
  static void integrate(MapT&                   map,
                        const SensorT&          sensor,
                        const se::Image<float>& depth_img,
                        const Eigen::Matrix4f&  T_WS,
                        const unsigned int      frame);
};



/**
 * Multi-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>
{
  template<typename SensorT,
           typename MapT
  >
  static void integrate(MapT&                   map,
                        const SensorT&          sensor,
                        const se::Image<float>& depth_img,
                        const Eigen::Matrix4f&  T_WS,
                        const unsigned int      frame);
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>
{
  template<typename SensorT,
           typename MapT
  >
  static void integrate(MapT&                   map,
                        const SensorT&          sensor,
                        const se::Image<float>& depth_img,
                        const Eigen::Matrix4f&  T_WS,
                        const unsigned int      frame);
};



template <typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;



template<se::Field FldT,
         se::Res ResT
>
template<typename SensorT,
         typename MapT
>
void IntegrateDepthImplD<FldT, ResT>::integrate(MapT&                   /* map */,
                                                const SensorT&          /* sensor */,
                                                const se::Image<float>& /* depth_img */,
                                                const Eigen::Matrix4f&  /* T_WS */,
                                                const unsigned int      /* frame */)
{
}



template<typename SensorT,
         typename MapT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(MapT&                   map,
                                                                      const SensorT&          sensor,
                                                                      const se::Image<float>& depth_img,
                                                                      const Eigen::Matrix4f&  T_WS,
                                                                      const unsigned int      frame)
{
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
}



template<typename SensorT,
         typename MapT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>::integrate(MapT&                   map,
                                                                     const SensorT&          sensor,
                                                                     const se::Image<float>& depth_img,
                                                                     const Eigen::Matrix4f&  T_WS,
                                                                     const unsigned int      frame)
{
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
}



template<typename SensorT,
         typename MapT
>
void IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>::integrate(MapT&                   map,
                                                                          const SensorT&          sensor,
                                                                          const se::Image<float>& depth_img,
                                                                          const Eigen::Matrix4f&  T_WS,
                                                                          const unsigned int      frame)
{
  // Allocation
  VolumeCarver<MapT, SensorT> volume_carver(map, sensor, depth_img, T_WS, frame); //< process based on variance state and project inside
  se::VolumeCarverAllocation allocation_list = volume_carver();

  // Update
  se::Updater updater(map, sensor, depth_img, T_WS, frame);
  updater(allocation_list);
}



} // namespace details



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT& map)
    : map_(map)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const SensorT&          sensor,
                                         const se::Image<float>& depth_img,
                                         const Eigen::Matrix4f&  T_WS,
                                         const unsigned int      frame)
{
  se::details::IntegrateDepthImpl<MapT>::integrate(map_, sensor, depth_img, T_WS, frame);
}



} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP

