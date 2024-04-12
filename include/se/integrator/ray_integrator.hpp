/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAY_INTEGRATOR_HPP
#define SE_RAY_INTEGRATOR_HPP

#include <Eigen/Core>
#include <set>
#include <thread>

#include "se/integrator/ray_integrator_core.hpp"

namespace se {

enum class RayState { FreeSpace, Transition, Occupied, Undefined };

template<typename MapT, typename SensorT>
class RayIntegrator {
    public:
    RayIntegrator(MapT& /* map */,
                  const SensorT& /* sensor */,
                  const Eigen::Vector3f& /*ray*/,
                  const Eigen::Isometry3f& /* T_SW need Lidar frame?*/,
                  const int /* frame */,
                  std::vector<const OctantBase*>* /*updated_octants = nullptr*/){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                    SensorT> {
    public:
    typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;

    /**
    * \brief The config file of the single ray carver
    *
    * \param[in] map   The map to allocate the ray in
    */
    struct RayIntegratorConfig {
        RayIntegratorConfig(const MapType& map) :
                sigma_min(map.getDataConfig().sigma_min_factor * map.getRes()),
                sigma_max(map.getDataConfig().sigma_max_factor * map.getRes()),
                tau_min(map.getDataConfig().tau_min_factor * map.getRes()),
                tau_max(map.getDataConfig().tau_max_factor * map.getRes())
        {
        }

        const float sigma_min;
        const float sigma_max;
        const float tau_min;
        const float tau_max;
    };


    /**
    * \brief Setup the single ray carver.
    *
    * \param[in]  map                  The reference to the map to be updated.
    * \param[in]  sensor               The sensor model.
    * \param[in]  ray                  The ray to be integrated.
    * \param[in]  T_WS                 The transformation from sensor to world frame.
    * \param[in]  frame                The frame number to be integrated.
    */
    RayIntegrator(MapType& map,
                  const SensorT& sensor,
                  const Eigen::Vector3f& ray,
                  const Eigen::Isometry3f& T_WS,
                  const int frame,
                  std::vector<const OctantBase*>* updated_octants = nullptr);

    /**
     * \brief Reset ray, pose and frame counter for the integrator
     *
     * \param[in] ray           The new ray measurement
     * \param[in] T_WS          The corresponding pose
     * \param[in] frame         The frame id
     * \param[in] skip_check    Boolean to decide if we check if ray can be skipped
     *
     * \return False if ray should be skipped. Otherwise true
     */
    bool resetIntegrator(const Eigen::Vector3f& ray,
                         const Eigen::Isometry3f& T_WS,
                         const int frame,
                         bool skip_check = false);

    /**
     * \brief Allocate and update along the ray using a step size depending on the chosen resolution.
     * The resolution is chosen based on the angle between neighboring Lidar rays.
     * Up and down-propagation needed for immediate update is done on-the-fly.
     */
    void operator()();

    /**
     * Update Operations
     */

    void propagateToRoot();

    void propagateBlocksToCoarsestScale();

    void updatedOctants(std::vector<const OctantBase*>* updated_octants);

    /**
    * \brief Return a conservative measure of the expected variance of a sensor model inside a voxel
    *        given its position and depth variance.
    *
    * \param[in] ray_step_depth depth of sample measurement along the ray.
    *
    * \return Estimate of the state along the ray (FreeSpace / Occupied / Behind)
    */
    se::RayState computeVariance(const float ray_step_depth);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
    * \brief Recursively decide if to allocate or terminate a node.
    *
    * \note se::LeicaLidar implementation
    *
    * \tparam SensorTDummy
    * \param[in] ray_sample     The current sample point along the ray
    * \param[in] voxel_coord    The voxel coordinates of the current sample along the ray
    * \param[in] rayState       The state along the ray (is it free space?)
    * \param[in] octant_ptr     Starting point for tree traversal
    */
    template<class SensorTDummy = SensorT>
    typename std::enable_if_t<std::is_same<SensorTDummy, se::LeicaLidar>::value, void>
    operator()(const Eigen::Vector3f& ray_sample,
               const Eigen::Vector3i& voxel_coord,
               se::RayState rayState,
               se::OctantBase* octant_ptr);


    /**
     * Update Operations
     */

    void updateBlock(se::OctantBase* octant_ptr,
                     Eigen::Vector3i& voxel_coords,
                     int desired_scale,
                     float sample_dist);

    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;

    std::vector<std::set<se::OctantBase*>> node_set_;
    std::vector<se::OctantBase*> updated_blocks_vector_;
    std::unordered_set<se::OctantBase*> updated_blocks_set_; // ToDo rename: updated_octants_;?
    bool track_updated_octants_ = false;
    RayIntegratorConfig config_;

    Eigen::Isometry3f T_SW_;
    Eigen::Vector3f ray_;
    Eigen::Vector3i last_visited_voxel_;

    const float map_res_;

    int free_space_scale_ = 0;
    int computed_integration_scale_ = 0;
    int frame_;

    // Sensor Model - Tau and Sigma
    float ray_dist_ = 0.;
    float tau_ = 0.;
    float three_sigma_ = 0.;
};

} // namespace se
#include "impl/ray_integrator_impl.hpp"

#endif // SE_RAY_INTEGRATOR_HPP
