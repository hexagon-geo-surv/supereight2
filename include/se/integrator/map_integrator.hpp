#ifndef SE_MAP_INTEGRATOR_HPP
#define SE_MAP_INTEGRATOR_HPP

#include <cstddef>
#include <iterator>

#include "se/map/utils/setup_util.hpp"
#include "se/map/octree/integrator.hpp"
#include "se/map/octree/fetcher.hpp"
#include "se/common/timings.hpp"



namespace se {
namespace allocator {

/**
 * \brief Allocate frustum in band around the surface.
 *
 * \tparam MapT
 * \tparam SensorT
 * \param[in] map       The reference to the map
 * \param[in] sensor    The sensor use for the projection
 * \param[in] depth_img The sensor depth image
 * \param[in] T_MS      The transformation from sensor to map frame
 * \param[in] band      The size of the band allocated around the surface
 *
 * \return The allocated and fetched notes in the band around the surface measurements
 */
template<typename MapT,
         typename SensorT
>
std::vector<se::OctantBase*> frustum(MapT&                      map,
                                     SensorT&                   sensor,
                                     const se::Image<depth_t>& depth_img,
                                     const Eigen::Matrix4f&     T_MS,
                                     const float                band);

} // namespace allocator



namespace fetcher {
  /**
   * \brief Return the currently allocated Blocks that intersect the camera frustum.
   * Some false positives might be returned since Blocks are approximated by their bounding spheres
   * and because sphereInFrustum() may return false positives in rare cases.
   *
   * \tparam MapT    The map type.
   * \tparam SensorT The sensor type.
   * \param map      The map to fetch Blocks from.
   * \param sensor   The sensor whose frustum is used for the test.
   * \param T_MS     The pose of the sensor in the map frame.
   *
   * \return A vector of pointers to Blocks that intersect the sensor frustum.
   */
  template<typename MapT,
           typename SensorT
  >
  inline std::vector<se::OctantBase*> frustum(MapT&                  map,
                                              const SensorT&         sensor,
                                              const Eigen::Matrix4f& T_MS);
} // namespace fetcher



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
                                               const int              octant_size);



template <typename MapT>
class MapIntegrator
{
public:
  MapIntegrator(MapT& map);

  /**
   * \brief Integrate depth image into the maps field representation.
   *
   * \tparam SensorT
   * \param[in] depth_img   The sensor depth image
   * \param[in] sensor      The sensor use for the projection
   * \param[in] T_MS        The transformation from sensor to map frame
   * \param[in] frame       The frame number to be integrated
   */
  template <typename SensorT>
  void integrateDepth(const SensorT&                sensor,
                      const se::Image<se::depth_t>& depth_img,
                      const Eigen::Matrix4f&        T_MS,
                      const unsigned int            frame);

private:
  MapT&            map_;
};



} // namespace se

#include "impl/map_integrator_impl.hpp"

#endif // SE_MAP_INTEGRATOR_HPP

