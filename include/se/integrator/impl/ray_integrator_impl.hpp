/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAY_INTEGRATOR_IMPL_HPP
#define SE_RAY_INTEGRATOR_IMPL_HPP

#define PRINT_TIMING 0

namespace se {

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
              SensorT>::RayIntegrator(MapType& map,
                                      const SensorT& sensor,
                                      const Eigen::Vector3f& ray,
                                      const Eigen::Isometry3f& T_WS,
                                      const int frame,
                                      std::vector<const OctantBase*>* updated_octants) :
        map_(map),
        octree_(map.getOctree()),
        sensor_(sensor),
        node_set_(octree_.getBlockDepth()),
        config_(map),
        T_SW_(T_WS.inverse()),
        ray_(ray),
        last_visited_voxel_(Eigen::Vector3i::Constant(-1)),
        map_res_(map.getRes()),
        free_space_scale_(map_.getDataConfig().fs_integr_scale),
        frame_(frame),
        ray_dist_(ray_.norm()),
        tau_(compute_tau(ray_dist_, config_.tau_min, config_.tau_max, map_.getDataConfig())),
        three_sigma_(compute_three_sigma(ray_dist_,
                                         config_.sigma_min,
                                         config_.sigma_max,
                                         map_.getDataConfig()))
{
    track_updated_octants_ = updated_octants;
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
bool RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT>::resetIntegrator(const Eigen::Vector3f& ray,
                                             const Eigen::Isometry3f& T_WS,
                                             const int frame,
                                             bool skip_ray)
{
    // Check if ray is expected to add new information. From the angle between sequential rays we compute the
    // distance between two measurements and check if that is smaller than the voxel / block at the
    // current integration scale
    if (skip_ray) {
        if ((ray - ray_).norm() < sqrt(3.0f) * map_res_) {
            // Make sure that measurements can only be jumped in case of not too large deltas
            const float angle_to_prev_ray = std::acos(ray.normalized().dot(ray_.normalized()));
            const float dist_to_prev_update = std::sin(angle_to_prev_ray) * ray.norm();
            if (dist_to_prev_update < map_res_ * (1 << computed_integration_scale_)) {
                return false;
            }
        }
    }

    ray_ = ray;
    ray_dist_ = ray_.norm();
    T_SW_ = T_WS.inverse();
    frame_ = frame;
    return true;
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT>::operator()()
{
    /// (0) Get Root Pointer
    se::OctantBase* root_ptr = octree_.getRoot();

    /// (1) Allocate all 8 children first level (if not yet allocated)
    octree_.allocateChildren(static_cast<NodeType*>(root_ptr));
    /// Check validity of measured value (sensor near plane)
    if (ray_dist_ < sensor_.near_plane) {
        return;
    }

    /// (3) Compute surface thickness tau and extend ray with surface thickness
    //Eigen::Vector3f extended_ray = (ray_dist_ + tau_) * ray_.normalized();
    //Eigen::Vector3f extended_ray_direction = extended_ray.normalized();

    /// Cut ray-sampling to max. of sensor range (sensor far plane)
    //float extended_ray_dist = (extended_ray.norm() > sensor_.far_plane) ? sensor_.far_plane : extended_ray.norm();

    /// (3) Determine maximum update distance along the ray, cut to maximum of sensor range (far plane)
    const Eigen::Vector3f ray_dir_S = ray_.normalized();
    const float max_update_dist = std::min(ray_dist_ + tau_, sensor_.far_plane);

    /// (4) Raycasting
    float safe_boundary = sensor_.near_plane;
    Eigen::Vector3f starting_point = safe_boundary * ray_dir_S;
    Eigen::Vector3f r_i_S = starting_point;
    Eigen::Vector3f r_i_W;
    Eigen::Isometry3f T_WS = T_SW_.inverse();

    while (r_i_S.norm() < max_update_dist) {
        // Compute if in free space
        se::RayState ray_state = computeVariance(r_i_S.norm());

        r_i_W = T_WS * r_i_S;
        Eigen::Vector3i voxel_coord;
        if (!map_.template pointToVoxel<se::Safe::On>(r_i_W, voxel_coord)) {
            break;
        }

        if (voxel_coord == last_visited_voxel_) {
            // can jump to next sample
            r_i_S += 0.5 * map_res_ * (1 << computed_integration_scale_) * ray_dir_S;
            continue;
        }
        last_visited_voxel_ = voxel_coord;

        (*this)(r_i_S, voxel_coord, ray_state, root_ptr);
        r_i_S += 0.5 * map_res_ * (1 << computed_integration_scale_) * ray_dir_S;
    }
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT>::propagateBlocksToCoarsestScale()
{
#pragma omp parallel for num_threads(3)
    for (size_t i = 0; i < updated_blocks_vector_.size(); i++) {
        se::ray_integrator::propagate_block_to_coarsest_scale<BlockType>(updated_blocks_vector_[i]);
    }
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
se::RayState RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                           SensorT>::computeVariance(const float ray_step_depth)
{
    // Assume worst case scenario -> no multiplication with proj_scale
    const float z_diff = (ray_step_depth - ray_dist_);

    if (z_diff < -three_sigma_) { // Guaranteed free Space
        return se::RayState::FreeSpace;
    }
    else if (z_diff < tau_ / 2.0) {
        return se::RayState::Transition;
    }
    else if (
        z_diff
        < tau_) { // within surface thickness => Occupied ToDo: remove distinction transition occupied
        return se::RayState::Occupied;
    }
    else { // Behind Surface
        return se::RayState::Undefined;
    }
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, se::LeicaLidar>::value, void>
RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
              SensorT>::operator()(const Eigen::Vector3f& ray_sample,
                                   const Eigen::Vector3i& voxel_coord,
                                   se::RayState rayState,
                                   se::OctantBase* octant_ptr)
{
    /// (1.a) Determine closest currently allocated octant
    se::OctantBase* finest_octant_ptr = se::fetcher::finest_octant<OctreeType>(
        voxel_coord, 0, octant_ptr); // up to scale 0 meaning down to finest resolution

    /// (1.b) Check if block level is already reached
    if (finest_octant_ptr->isBlock()) {
        /// ALREADY BLOCK-LEVEL -> DO update here
        /// (2.a) Determine Integration Scale
        // compute integration scale
        BlockType* block_ptr = static_cast<BlockType*>(finest_octant_ptr);
        // The last integration scale
        const int last_scale = (block_ptr->getMinScale() == -1) ? 0 : block_ptr->getCurrentScale();

        // Compute the point of the block centre in the sensor frame ToDo: is this needed? Can't we just use the point in sensor frame?
        const unsigned int block_size = BlockType::size;
        const Eigen::Vector3i block_coord = block_ptr->getCoord();
        Eigen::Vector3f block_centre_point_W;
        map_.voxelToPoint(block_coord, block_size, block_centre_point_W);
        const Eigen::Vector3f block_centre_point_C = T_SW_ * block_centre_point_W;

        // The recommended integration scale
        int computed_integration_scale = sensor_.computeIntegrationScale(block_centre_point_C,
                                                                         map_res_,
                                                                         last_scale,
                                                                         block_ptr->getMinScale(),
                                                                         block_ptr->getMaxScale());
        if (rayState == se::RayState::FreeSpace
            && (computed_integration_scale < free_space_scale_)) {
            computed_integration_scale = free_space_scale_;
        }

        /// (2.b) Update Block
        updateBlock(finest_octant_ptr,
                    const_cast<Eigen::Vector3i&>(voxel_coord),
                    computed_integration_scale,
                    ray_sample.norm());
        computed_integration_scale_ = computed_integration_scale;

        /// (2.c) Save block for later up-propagation (only save once
        if (updated_blocks_set_.count(finest_octant_ptr) == 0) {
            updated_blocks_set_.insert(finest_octant_ptr);
            updated_blocks_vector_.push_back(finest_octant_ptr);
        }
    }
    else {
        octree_.allocateChildren(static_cast<NodeType*>(finest_octant_ptr));
        return (*this)(ray_sample, voxel_coord, rayState, finest_octant_ptr);
    }

    return;
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT>::updateBlock(se::OctantBase* octant_ptr,
                                         Eigen::Vector3i& voxel_coords,
                                         int desired_scale,
                                         float sample_dist)
{
    if (sample_dist < 1e-08)
        return;
    // Block (pointer) to be updated
    BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
    const Eigen::Vector3i block_coord =
        block_ptr->getCoord(); /// < Coordinates of block to be updated

    // Set timestamp of the current block
    timestamp_t previous_time_stamp = block_ptr->getTimeStamp();
    const bool is_already_integrated = (previous_time_stamp == frame_);
    if (!is_already_integrated) {
        block_ptr->setTimeStamp(frame_);
    }


    // The last integration scale
    // -- Nothing integrated yet (-1) => set to desired_scale
    // -- otherwise current scale of block ptr
    const int last_scale =
        (block_ptr->getMinScale() == -1) ? desired_scale : block_ptr->getCurrentScale();
    int integration_scale = last_scale;


    // Case 1: Nothing integrated yet:
    if (block_ptr->getMinScale() == -1) { // nothing inegrated yet
        // Make sure the block is allocated up to the integration scale
        block_ptr->allocateDownTo(integration_scale);
        block_ptr->setInitData(DataType());
    }
    else if (desired_scale < last_scale) {
        se::ray_integrator::propagate_block_down_to_scale<BlockType>(block_ptr, desired_scale);

        // set new scale and min scale
        integration_scale = desired_scale;
    }
    else if (!is_already_integrated && (desired_scale > last_scale)) {
        se::ray_integrator::propagate_block_to_scale<BlockType>(block_ptr, desired_scale);
        integration_scale = desired_scale;
        block_ptr->deleteUpTo(integration_scale);
        block_ptr->setCurrentScale(integration_scale);
    }

    /// Determine voxels to be updated
    const unsigned int integration_stride = 1
        << integration_scale; // 1 for scale 0, 2 for scale 1, 4 for scale 2 etc.
    ///< Number of voxels per dimension that has to be updated at integration scale
    const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
    ///< Size of the block at integration scale
    const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);
    ///< Squared size of the block at integration scale

    const Eigen::Vector3i offset_coords = (voxel_coords - block_coord) / integration_stride;

    const int voxel_idx = offset_coords.x() + size_at_integration_scale_li * offset_coords.y()
        + size_at_integration_scale_sq * offset_coords.z();
    DataType* data_at_scale = block_ptr->blockDataAtScale(integration_scale);
    auto& voxel_data = data_at_scale[voxel_idx];
    float range_diff = sample_dist - ray_dist_;
    ray_integrator::update_voxel(voxel_data, range_diff, tau_, three_sigma_, map_.getDataConfig());
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT>::propagateToRoot()
{
    // Retrieving Parent Nodes for all updated blocks
    for (const auto& octant_ptr : updated_blocks_vector_) {
        BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
        if (block_ptr->getParent()) {
            node_set_[octree_.getBlockDepth() - 1].insert(block_ptr->getParent());
        }
    }

    for (int d = octree_.getBlockDepth() - 1; d > 0; d--) // TODO: block depth - 1?
    {
        std::set<se::OctantBase*>::iterator it;
        for (it = node_set_[d].begin(); it != node_set_[d].end(); ++it) {
            se::OctantBase* octant_ptr = *it;
            if (octant_ptr->getTimeStamp() == frame_) {
                continue;
            }

            if (octant_ptr->getParent()) {
                auto node_data = ray_integrator::propagate_to_parent_node<NodeType, BlockType>(
                    octant_ptr, frame_);
                node_set_[d - 1].insert(octant_ptr->getParent());
                if (track_updated_octants_) {
                    updated_blocks_set_.insert(octant_ptr);
                }

                // If all nodes free space, delete children and just leave coarser resolution
                if (node_data.observed
                    && get_field(node_data) <= 0.95 * MapType::DataType::min_occupancy) {
                    auto* node_ptr = static_cast<NodeType*>(octant_ptr);
                    if (track_updated_octants_) {
                        for (int i = 0; i < 8; i++) {
                            OctantBase* const child_ptr = node_ptr->getChild(i);
                            if (child_ptr) {
                                updated_blocks_set_.erase(child_ptr);
                            }
                        }
                    }
                    octree_.deleteChildren(node_ptr);
                }

            } // if parent
        }     // nodes at depth d
    }         // depth d

    ray_integrator::propagate_to_parent_node<NodeType, BlockType>(octree_.getRoot(), frame_);
}

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
void RayIntegrator<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>,
                   SensorT>::updatedOctants(std::vector<const OctantBase*>* updated_octants)
{
    if (track_updated_octants_) {
        updated_octants->clear();
        updated_octants->reserve(updated_blocks_set_.size());
        updated_octants->insert(
            updated_octants->end(), updated_blocks_set_.begin(), updated_blocks_set_.end());
    }
}

} // namespace se

#endif //SE_RAY_INTEGRATOR_IMPL_HPP
