#ifndef SE_MAP_INTEGRATOR_HPP
#define SE_MAP_INTEGRATOR_HPP

#include <cstddef>
#include <iterator>

#include "se/utils/setup_util.hpp"
#include "se/octree/integrator.hpp"
#include "se/octree/fetcher.hpp"
#include "se/timings.hpp"



namespace se {
namespace allocator {

/**
 * \brief Allocate frustum in band around the surface.
 *
 * \tparam SensorT
 * \tparam MapT
 * \param[in] depth_img The sensor depth image
 * \param[in] sensor    The sensor use for the projection
 * \param[in] T_MS      The transformation from sensor to map frame
 * \param[in] map       The reference to the map
 * \param[in] band      The size of the band allocated around the surface
 *
 * \return The allocated and fetched notes in the band around the surface measurements
 */
template<typename SensorT, typename MapT>
std::vector<typename MapT::OctreeType::BlockType*> frustum(const se::Image<depth_t>& depth_img,
                                                           SensorT&                  sensor,
                                                           const Eigen::Matrix4f&    T_MS,
                                                           MapT&                     map,
                                                           const float               band);

} // namespace allocator


namespace {

/**
 * \brief compute the sample coordinates for a given octant coordinate
 *
 * \param octant_coord       The octant coordinates
 * \param octant_size        The size of the octant
 * \param sample_offset_frac The offset fraction of the sample point to the octant corner,
 *                           i.e. (0,0,0) for octant corner and (0.5, 0.5, 0.5) for octant centre
 *
 * \return The octant sample coordinates
 */
static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord,
                                               const int              octant_size,
                                               const Eigen::Vector3f& sample_offset_frac);

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
                        MapT&                         map,
                        ConfigT&                      /* config */);
};



template <typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::ress_>;



} // namespace anonymous


/**
 * TODO:
 */
struct IntegratorConfig
{
  IntegratorConfig() {}
};



template <typename MapT>
class MapIntegrator
{
public:
  MapIntegrator(MapT&                   map,
                const IntegratorConfig& config = IntegratorConfig());

  /**
   * \brief Integrate depth image into the maps field representation.
   *
   * \tparam SensorT
   * \param[in] depth_img   The sensor depth image
   * \param[in] sensor      The sensor use for the projection
   * \param[in] T_MS        The transformation from sensor to map frame
   */
  template <typename SensorT>
  void integrateDepth(const se::Image<se::depth_t>&      depth_img,
                      const SensorT&                     sensor,
                      const Eigen::Matrix4f&             T_MS);

private:
  MapT&            map_;
  IntegratorConfig config_;
};



} // namespace se

#include "impl/map_integrator_impl.hpp"



#endif //SE_TRYOUT_MAP_INTEGRATOR_HPP
