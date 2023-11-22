/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_INTEGRATOR_HPP
#define SE_MAP_INTEGRATOR_HPP

#include <cstddef>
#include <iterator>

#include "se/common/math_util.hpp"
#include "se/integrator/allocator/raycast_carver.hpp"
#include "se/integrator/allocator/volume_carver.hpp"
#include "se/integrator/ray_integrator.hpp"
#include "se/map/octree/fetcher.hpp"
#include "se/map/octree/integrator.hpp"
#include "se/map/utils/setup_util.hpp"

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
 * \param[in] T_WS      The transformation from sensor to world frame
 * \param[in] band      The size of the band allocated around the surface
 *
 * \return The allocated and fetched notes in the band around the surface measurements
 */
template<typename MapT, typename SensorT>
std::vector<se::OctantBase*> frustum(MapT& map,
                                     SensorT& sensor,
                                     const se::Image<float>& depth_img,
                                     const Eigen::Matrix4f& T_WS,
                                     const float band);

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
 * \param T_WS     The pose of the sensor in the world frame.
 *
 * \return A vector of pointers to Blocks that intersect the sensor frustum.
 */
template<typename MapT, typename SensorT>
inline std::vector<se::OctantBase*>
frustum(MapT& map, const SensorT& sensor, const Eigen::Matrix4f& T_WS);
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
                                               const int octant_size);



template<typename MapT>
class MapIntegrator {
    public:
    MapIntegrator(MapT& map);

    /**
     * \brief Integrate depth image into the maps field representation.
     *
     * \tparam SensorT
     * \param[in] depth_img   The sensor depth image
     * \param[in] sensor      The sensor use for the projection
     * \param[in] T_WS        The transformation from sensor to world frame
     * \param[in] frame       The frame number to be integrated
     */
    template<typename SensorT>
    void integrateDepth(const SensorT& sensor,
                        const se::Image<float>& depth_img,
                        const Eigen::Matrix4f& T_WS,
                        const unsigned int frame);

    /** Same as se::MapIntegrator::integrateDepth() but also stores pointers to the octants updated
     * during the integration in \p updated_octants.
     */
    template<typename SensorT>
    void integrateDepth(const SensorT& sensor,
                        const se::Image<float>& depth_img,
                        const Eigen::Matrix4f& T_WS,
                        const unsigned int frame,
                        std::vector<const OctantBase*>& updated_octants);

    /**
     * \brief Integrate single ray measurement into the maps field representation.
     *
     * \tparam SensorT
     * \param[in] ray_S       The measured ray in sensor frame
     * \param[in] sensor      The sensor use for the projection
     * \param[in] T_WS        The transformation from sensor to world frame
     * \param[in] frame       The frame number to be integrated (number of so-far integrated rays)
     */
    template<typename SensorT>
    void integrateRay(const SensorT& sensor,
                      const Eigen::Vector3f& ray_S,
                      const Eigen::Matrix4f& T_WS,
                      const unsigned int frame);

    /**
     * \brief Integrate a batch of ray images into the maps field representation.
     *
     * \tparam SensorT
     * \param[in] sensor      The sensor use for the projection
     * \param[in] rayBatch    The batch of ray measurements (in sensor frame)
     * \param[in] poseBatch   The batch of corresponding sensor poses in the world frame
     * \param[in] frame       The frame number to be integrated (will be number of batch)
     */
    template<typename SensorT>
    void integrateRayBatch(const SensorT& sensor,
                           const std::vector<std::pair<Eigen::Matrix4f,Eigen::Vector3f>,
                               Eigen::aligned_allocator<std::pair<Eigen::Matrix4f,Eigen::Vector3f>>>& rayPoseBatch,
                           const unsigned int frame);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    MapT& map_;
};



} // namespace se

#include "impl/map_integrator_impl.hpp"

#endif // SE_MAP_INTEGRATOR_HPP
