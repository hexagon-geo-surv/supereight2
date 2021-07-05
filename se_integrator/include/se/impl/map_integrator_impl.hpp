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
  static void integrate(MapT&                         map,
                        const SensorT&                sensor,
                        const se::Image<se::depth_t>& depth_img,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
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
  static void integrate(MapT&                         map,
                        const SensorT&                sensor,
                        const se::Image<se::depth_t>& depth_img,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
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
  static void integrate(MapT&                         map,
                        const SensorT&                sensor,
                        const se::Image<se::depth_t>& depth_img,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
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
  static void integrate(MapT&                         map,
                        const SensorT&                sensor,
                        const se::Image<se::depth_t>& depth_img,
                        const Eigen::Matrix4f&        T_MS,
                        const unsigned int            frame,
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
void IntegrateDepthImplD<FldT, ResT>::integrate(MapT&                         map,
                                                const SensorT&                sensor,
                                                const se::Image<se::depth_t>& depth_img,
                                                const Eigen::Matrix4f&        T_MS,
                                                const unsigned int            frame,
                                                ConfigT&                      config)
{
}



template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(MapT&                         map,
                                                                      const SensorT&                sensor,
                                                                      const se::Image<se::depth_t>& depth_img,
                                                                      const Eigen::Matrix4f&        T_MS,
                                                                      const unsigned int            frame,
                                                                      ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getRes() * map.getDataConfig().truncation_boundary_factor;
  const float band                = 2.f * truncation_boundary;

  // Allocation
  se::RaycastCarver raycast_carver(map, sensor, depth_img, T_MS, frame);
  std::vector<OctantBase*> block_ptrs = raycast_carver.allocateBand(band);

  // Update
  se::Updater updater(map, sensor, depth_img, T_MS, frame);
  updater(block_ptrs);
}



template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>::integrate(MapT&                         map,
                                                                     const SensorT&                sensor,
                                                                     const se::Image<se::depth_t>& depth_img,
                                                                     const Eigen::Matrix4f&        T_MS,
                                                                     const unsigned int            frame,
                                                                     ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getRes() * map.getDataConfig().truncation_boundary_factor;
  const float band                = 2.f * truncation_boundary;

  // Allocation
  se::RaycastCarver raycast_carver(map, sensor, depth_img, T_MS, frame);
  std::vector<OctantBase*> block_ptrs = raycast_carver.allocateBand(band);

  // Update
  se::Updater updater(map, sensor, depth_img, T_MS, frame);
  updater(block_ptrs);
}



template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>::integrate(MapT&                         map,
                                                                          const SensorT&                sensor,
                                                                          const se::Image<se::depth_t>& depth_img,
                                                                          const Eigen::Matrix4f&        T_MS,
                                                                          const unsigned int            frame,
                                                                          ConfigT&                      /* config */)
{
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

  // Allocation
  VolumeCarver<MapT, SensorT> volume_carver(map, sensor, depth_img, T_SM, frame); //< process based on variance state and project inside
  se::VolumeCarverAllocation allocation_list = volume_carver.allocateFrustum();

  // Update
  se::Updater updater(map, sensor, depth_img, T_MS, frame);
  updater(allocation_list);
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
void MapIntegrator<MapT>::integrateDepth(const SensorT&                sensor,
                                         const se::Image<se::depth_t>& depth_img,
                                         const Eigen::Matrix4f&        T_MS,
                                         const unsigned int            frame)
{
  se::details::IntegrateDepthImpl<MapT>::integrate(map_, sensor, depth_img, T_MS, frame, config_);
}



} // namespace se

#endif //SE_MAP_INTEGRATOR_IMPL_HPP
