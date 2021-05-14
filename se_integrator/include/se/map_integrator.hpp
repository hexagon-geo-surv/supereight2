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
se::vector<se::OctantBase*> frustum(const se::Image<depth_t>& depth_img,
                                    SensorT&                  sensor,
                                    const Eigen::Matrix4f&    T_MS,
                                    MapT&                     map,
                                    const float               band);

} // namespace allocator



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
  void integrateDepth(const se::Image<se::depth_t>& depth_img,
                      const SensorT&                sensor,
                      const Eigen::Matrix4f&        T_MS,
                      const unsigned int            frame);

private:
  MapT&            map_;
  IntegratorConfig config_;
};



} // namespace se

#include "impl/map_integrator_impl.hpp"



#endif //SE_TRYOUT_MAP_INTEGRATOR_HPP
