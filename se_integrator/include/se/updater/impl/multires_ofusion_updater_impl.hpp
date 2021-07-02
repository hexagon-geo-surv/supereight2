#ifndef SE_MULTIRES_OFUSION_UPDATER_IMPL_HPP
#define SE_MULTIRES_OFUSION_UPDATER_IMPL_HPP



namespace se {



template<typename MapT,
         typename SensorT
>
MultiresOFusionUpdater<MapT, SensorT>::MultiresOFusionUpdater(const se::Image<float>&                 depth_image,
                                                              MapT&                                   map,
                                                              const PinholeCamera&                    sensor,
                                                              const Eigen::Matrix4f&                  T_SM,
                                                              const int                               frame,
                                                              std::vector<std::set<se::OctantBase*>>& node_set,
                                                              std::vector<se::OctantBase*>&           freed_block_list) :
    depth_image_(depth_image),
    map_(map),
    octree_(*map.getOctree()),
    sensor_(sensor),
    T_CM_(T_SM),
    frame_(frame),
    map_res_(map.getRes()),
    sigma_min_(map.getDataConfig().sigma_min_factor * map_res_),
    sigma_max_(map.getDataConfig().sigma_max_factor * map_res_),
    tau_min_(map.getDataConfig().tau_min_factor * map_res_),
    tau_max_(map.getDataConfig().tau_max_factor * map_res_),
    node_set_(node_set),
    freed_block_list_(freed_block_list)
{
}



template<typename MapT,
        typename SensorT
>
void MultiresOFusionUpdater<MapT, SensorT>::propagateToRoot(std::vector<se::OctantBase*>& block_list)
{
  for(const auto& octant_ptr : block_list)
  {
    typename MapT::OctreeType::BlockType* block_ptr = static_cast<typename MapT::OctreeType::BlockType*>(octant_ptr);
    if(block_ptr->getParent())
    {
      node_set_[map_.getOctree()->getBlockDepth() - 1].insert(block_ptr->getParent());
    }
  }

  for (int d = map_.getOctree()->getBlockDepth() - 1; d > 0; d--) // TODO: block depth - 1?
  {
    std::set<se::OctantBase*>::iterator it;
    for (it = node_set_[d].begin(); it != node_set_[d].end(); ++it)
    {
      se::OctantBase* octant_ptr = *it;
      if(octant_ptr->getTimeStamp() == frame_)
      {
        continue;
      }
      if(octant_ptr->getParent())
      {
        auto node_data = updater::propagateToNoteAtCoarserScale<typename MapT::OctreeType::NodeType, typename MapT::OctreeType::BlockType>(octant_ptr, map_.getOctree()->getMaxScale(), frame_);
        node_set_[d-1].insert(octant_ptr->getParent());

        if (   node_data.observed
            && node_data.occupancy * node_data.weight <= 0.95 * map_.getDataConfig().min_occupancy)
        {
          map_.getOctree()->deleteChildren(static_cast<typename MapT::OctreeType::NodeType*>(octant_ptr));
        }

        // TODO: ^SWITCH 1 - Alternative approach (conservative)
        // Delete node if it's in free space and it's max value already surpassed a lower threshold.
        // Approach only saves time.
//          if (   node_data.observed
//              && node_data.x <= 0.95 * map_.getDataConfig().log_odd_min
//              && node_data.y > map_.getDataConfig().max_weight / 2)
//          {
//            map_.getOctree()->deleteChildren(octant_ptr);
//          }

      } // if parent
    } // nodes at depth d
  } // depth d

  updater::propagateToNoteAtCoarserScale<typename MapT::OctreeType::NodeType, typename MapT::OctreeType::BlockType>(map_.getOctree()->getRoot(), map_.getOctree()->getMaxScale(), frame_);

}



template<typename MapT,
         typename SensorT
>
void MultiresOFusionUpdater<MapT, SensorT>::freeBlock(se::OctantBase* octant_ptr)
{

  BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);

  // Compute the point of the block centre in the sensor frame
  const unsigned int block_size = BlockType::size;
  const Eigen::Vector3i block_coord = block_ptr->getCoord();
  Eigen::Vector3f block_centre_point_M;
  /// CHANGED
  map_.voxelToPoint(block_coord, block_size, block_centre_point_M);
  const Eigen::Vector3f block_centre_point_C = (T_CM_ * (block_centre_point_M).homogeneous()).head(3);

  /// Compute the integration scale
  // The last integration scale
  const int last_scale = block_ptr->getCurrentScale();

  // The recommended integration scale
  const int computed_integration_scale =
          sensor_.computeIntegrationScale(block_centre_point_C, map_res_, last_scale, block_ptr->getMinScale(), block_ptr->getMaxScale());

  // The minimum integration scale (change to last if data has already been integrated)
  const int min_integration_scale = ((block_ptr->getMinScale() == -1 ||
                                      block_ptr->maxValue() < 0.95 * map_.getDataConfig().log_odd_min))
                                    ? map_.getDataConfig().fs_integr_scale : std::max(0, last_scale - 1);
  const int max_integration_scale = (block_ptr->getMinScale() == -1) ? BlockType::getMaxScale() :
                                    std::min(BlockType::getMaxScale(), last_scale + 1);

  // The final integration scale
  const int recommended_scale = std::min(std::max(min_integration_scale, computed_integration_scale),
                                         max_integration_scale);

  int integration_scale = last_scale;

  /// If no data has been integrated in the block before (block_ptr->getMinScale() == -1), use the computed integration scale.
  if (block_ptr->getMinScale() == -1)
  {
    // Make sure the block is allocated up to the integration scale
    integration_scale = recommended_scale;
    block_ptr->allocateDownTo(integration_scale);
    block_ptr->setCurrentScale(integration_scale);
    block_ptr->initCurrCout();
    block_ptr->setInitData(DataType());

  } else if (recommended_scale != last_scale) ///<< Potential double integration
  {
    if (recommended_scale != block_ptr->buffer_scale()) ///<< Start from scratch and initialise buffer
    {
      block_ptr->initBuffer(recommended_scale);

      if (recommended_scale < last_scale)
      {
        const int parent_scale = last_scale;
        const unsigned int size_at_parent_scale_li = block_size >> parent_scale;
        const unsigned int size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

        const unsigned int size_at_buffer_scale_li = size_at_parent_scale_li << 1;
        const unsigned int size_at_buffer_scale_sq = se::math::sq(size_at_buffer_scale_li);

        for (unsigned int z = 0; z < size_at_parent_scale_li; z++)
        {
          for (unsigned int y = 0; y < size_at_parent_scale_li; y++)
          {
#pragma omp simd // TODO: MOVE UP
            for (unsigned int x = 0; x < size_at_parent_scale_li; x++)
            {
              const int parent_idx = x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
              const auto &parent_data = block_ptr->currData(parent_idx); // TODO: CAN BE MADE FASTER

              for (unsigned int k = 0; k < 2; k++)
              {
                for (unsigned int j = 0; j < 2; j++)
                {
                  for (unsigned int i = 0; i < 2; i++)
                  {
                    const int buffer_idx = (2 * x + i) + (2 * y + j) * size_at_buffer_scale_li +
                                           (2 * z + k) * size_at_buffer_scale_sq;
                    auto &buffer_data = block_ptr->bufferData(buffer_idx); ///<< Fetch value from buffer.

                    buffer_data.occupancy = parent_data.occupancy;
                    buffer_data.weight    = parent_data.weight; // (parent_data.y > 0) ? 1 : 0;
                    buffer_data.observed  = false; ///<< Set falls such that the observe count can work properly

                  } // i
                } // j
              } // k

            } // x
          } // y
        } // z

      }

    }

    /// Integrate data into buffer.
    const unsigned int size_at_recommended_scale_li = BlockType::size >> recommended_scale;
    const unsigned int size_at_recommended_scale_sq = se::math::sq(size_at_recommended_scale_li);

    for (unsigned int z = 0; z < size_at_recommended_scale_li; z++)
    {
      for (unsigned int y = 0; y < size_at_recommended_scale_li; y++)
      {
#pragma omp simd
        for (unsigned int x = 0; x < size_at_recommended_scale_li; x++)
        {
          const int buffer_idx = x + y * size_at_recommended_scale_li + z * size_at_recommended_scale_sq;
          auto &buffer_data = block_ptr->bufferData(buffer_idx); /// \note pass by reference now.
          block_ptr->incrBufferObservedCount(updater::freeVoxel(buffer_data, map_.getDataConfig()));
        } // x
      } // y
    } // z

    block_ptr->incrBufferIntegrCount();

    if (block_ptr->switchData())
    {
      return;
    }

  } else
  {
    block_ptr->resetBuffer();
  }

  const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
  const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);

  for (unsigned int z = 0; z < size_at_integration_scale_li; z++)
  {
    for (unsigned int y = 0; y < size_at_integration_scale_li; y++)
    {
#pragma omp simd // TODO: Move UP
      for (unsigned int x = 0; x < size_at_integration_scale_li; x++)
      {
        // Update the voxel data based using the depth measurement
        const int voxel_idx = x + y * size_at_integration_scale_li + z * size_at_integration_scale_sq;
        auto& voxel_data = block_ptr->currData(voxel_idx); /// \note pass by reference now.
        block_ptr->incrCurrObservedCount(updater::freeVoxel(voxel_data, map_.getDataConfig()));
      } // x
    } // y
  } // z

  block_ptr->incrCurrIntegrCount();
}



template<typename MapT,
         typename SensorT
>
void MultiresOFusionUpdater<MapT, SensorT>::updateBlock(se::OctantBase* octant_ptr,
                                                        bool            low_variance,
                                                        bool            project_inside)
{
  // Compute the point of the block centre in the sensor frame
  BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
  const int block_size = BlockType::size;
  const Eigen::Vector3i block_coord = block_ptr->getCoord();

  Eigen::Vector3f block_centre_point_M;
  map_.voxelToPoint(block_coord, block_size, block_centre_point_M);
  const Eigen::Vector3f block_centre_point_C = (T_CM_ * (block_centre_point_M).homogeneous()).head(3);

  // Convert block centre to measurement >> PinholeCamera -> .z() | OusterLidar -> .norm()
  const float block_point_C_m = sensor_.measurementFromPoint(block_centre_point_C);

  // Compute one tau and 3x sigma value for the block
  float tau         = compute_tau(block_point_C_m, tau_min_, tau_max_, map_.getDataConfig());
  float three_sigma = compute_three_sigma(block_point_C_m, sigma_max_, sigma_max_, map_.getDataConfig());

  /// Compute the integration scale
  // The last integration scale
  const int last_scale = block_ptr->getCurrentScale();

  // The recommended integration scale
  const int computed_integration_scale =
          sensor_.computeIntegrationScale(block_centre_point_C, map_res_, last_scale, block_ptr->getMinScale(),
                                          block_ptr->getMaxScale());

  // The minimum integration scale (change to last if data has already been integrated)
  const int min_integration_scale =
          (low_variance &&
           (block_ptr->getMinScale() == -1 || block_ptr->maxValue() < 0.95 * map_.getDataConfig().log_odd_min))
          ? map_.getDataConfig().fs_integr_scale : std::max(0, last_scale - 1);
  const int max_integration_scale = (block_ptr->getMinScale() == -1) ? BlockType::getMaxScale() :
                                    std::min(BlockType::getMaxScale(), last_scale + 1);

  // The final integration scale
  const int recommended_scale = std::min(std::max(min_integration_scale, computed_integration_scale),
                                         max_integration_scale);

  int integration_scale = last_scale;

  /// If no data has been integrated in the block before (block_ptr->getMinScale() == -1), use the computed integration scale.
  if (block_ptr->getMinScale() == -1)
  {
    // Make sure the block is allocated up to the integration scale
    integration_scale = recommended_scale;
    block_ptr->allocateDownTo(integration_scale);
    block_ptr->setCurrentScale(integration_scale);
    block_ptr->initCurrCout();
    block_ptr->setInitData(DataType());

  } else if (recommended_scale != last_scale) ///<< Potential double integration
  {
    if (recommended_scale != block_ptr->buffer_scale()) ///<< Start from scratch and initialise buffer
    {
      block_ptr->initBuffer(recommended_scale);

      if (recommended_scale < last_scale)
      {
        const int parent_scale = last_scale;
        const unsigned int size_at_parent_scale_li = BlockType::size >> parent_scale;
        const unsigned int size_at_parent_scale_sq = se::math::sq(size_at_parent_scale_li);

        const unsigned int size_at_buffer_scale_li = size_at_parent_scale_li << 1;
        const unsigned int size_at_buffer_scale_sq = se::math::sq(size_at_buffer_scale_li);

        for (unsigned int z = 0; z < size_at_parent_scale_li; z++)
        {
          for (unsigned int y = 0; y < size_at_parent_scale_li; y++)
          {
#pragma omp simd
            for (unsigned int x = 0; x < size_at_parent_scale_li; x++)
            {
              const int parent_idx = x + y * size_at_parent_scale_li + z * size_at_parent_scale_sq;
              const auto &parent_data = block_ptr->currData(parent_idx); // TODO: CAN BE MADE FASTER

              for (unsigned int k = 0; k < 2; k++)
              {
                for (unsigned int j = 0; j < 2; j++)
                {
                  for (unsigned int i = 0; i < 2; i++)
                  {
                    const int buffer_idx = (2 * x + i) + (2 * y + j) * size_at_buffer_scale_li +
                                           (2 * z + k) * size_at_buffer_scale_sq;
                    auto& buffer_data = block_ptr->bufferData(buffer_idx); ///<< Fetch value from buffer.

                    buffer_data.occupancy = parent_data.occupancy;
                    buffer_data.weight    = parent_data.weight; // (parent_data.y > 0) ? 1 : 0;
                    buffer_data.observed  = false; ///<< Set falls such that the observe count can work properly

                  } // i
                } // j
              } // k

            } // x
          } // y
        } // z

      }

    }

    /// Integrate data into buffer.
    const unsigned int recommended_stride = 1 << recommended_scale;
    const unsigned int size_at_recommended_scale_li = BlockType::size >> recommended_scale;
    const unsigned int size_at_recommended_scale_sq = se::math::sq(size_at_recommended_scale_li);

    const Eigen::Vector3i voxel_coord_base = block_ptr->getCoord();
    Eigen::Vector3f sample_point_base_M;
    map_.voxelToPoint(voxel_coord_base, recommended_stride, sample_point_base_M);
    const Eigen::Vector3f sample_point_base_C = (T_CM_ * (sample_point_base_M).homogeneous()).head(3);

    const Eigen::Matrix3f sample_point_delta_matrix_C = (se::math::to_rotation(T_CM_) *
                                                         (map_res_ * (Eigen::Matrix3f() << recommended_stride, 0, 0,
                                                                                             0, recommended_stride, 0,
                                                                                             0, 0, recommended_stride).finished()));

    auto valid_predicate = [&](float depth_value) { return depth_value >= sensor_.near_plane; };

    for (unsigned int z = 0; z < size_at_recommended_scale_li; z++)
    {
      for (unsigned int y = 0; y < size_at_recommended_scale_li; y++)
      {
#pragma omp simd
        for (unsigned int x = 0; x < size_at_recommended_scale_li; x++)
        {
          const Eigen::Vector3f sample_point_C = sample_point_base_C + sample_point_delta_matrix_C * Eigen::Vector3f(x, y, z);

          // Fetch image value
          float depth_value(0);
          if (!sensor_.projectToPixelValue(sample_point_C, depth_image_, depth_value, valid_predicate))
          {
            continue;
          }

          const int buffer_idx = x + y * size_at_recommended_scale_li + z * size_at_recommended_scale_sq;
          auto &buffer_data = block_ptr->bufferData(buffer_idx); /// \note pass by reference now.

          if (low_variance)
          {
            block_ptr->incrBufferObservedCount(updater::freeVoxel(buffer_data, map_.getDataConfig()));
          } else
          {
            const float sample_point_C_m = sensor_.measurementFromPoint(sample_point_C);
            const float range = sample_point_C.norm();
            const float range_diff = (sample_point_C_m - depth_value) * (range / sample_point_C_m);
            block_ptr->incrBufferObservedCount(
                    updater::updateVoxel(buffer_data, range_diff, tau, three_sigma, map_.getDataConfig()));
          }
        } // x
      } // y
    } // z

    block_ptr->incrBufferIntegrCount(project_inside);

    if (block_ptr->switchData())
    {
      return;
    }

  } else
  {
    block_ptr->resetBuffer();
  }

  const unsigned int integration_stride = 1 << integration_scale;
  const unsigned int size_at_integration_scale_li = BlockType::size >> integration_scale;
  const unsigned int size_at_integration_scale_sq = se::math::sq(size_at_integration_scale_li);

  const Eigen::Vector3i voxel_coord_base = block_ptr->getCoord();
  Eigen::Vector3f sample_point_base_M;
  map_.voxelToPoint(voxel_coord_base, integration_stride, sample_point_base_M);
  const Eigen::Vector3f sample_point_base_C = (T_CM_ * (sample_point_base_M).homogeneous()).head(3);

  const Eigen::Matrix3f sample_point_delta_matrix_C = (se::math::to_rotation(T_CM_) *
                                                       (map_res_ * (Eigen::Matrix3f() << integration_stride, 0, 0,
                                                                                         0, integration_stride, 0,
                                                                                         0, 0, integration_stride).finished()));

  auto valid_predicate = [&](float depth_value) { return depth_value >= sensor_.near_plane; };

  for (unsigned int z = 0; z < size_at_integration_scale_li; z++)
  {
    for (unsigned int y = 0; y < size_at_integration_scale_li; y++)
    {
#pragma omp simd
      for (unsigned int x = 0; x < size_at_integration_scale_li; x++)
      {
        const Eigen::Vector3f sample_point_C = sample_point_base_C + sample_point_delta_matrix_C * Eigen::Vector3f(x, y, z);

        // Fetch image value
        float depth_value(0);
        if (!sensor_.projectToPixelValue(sample_point_C, depth_image_, depth_value, valid_predicate))
        {
          continue;
        }

        // Update the voxel data based using the depth measurement
        const int voxel_idx = x + y * size_at_integration_scale_li + z * size_at_integration_scale_sq;
        auto& voxel_data = block_ptr->currData(voxel_idx); /// \note pass by reference now.
        if (low_variance)
        {
          block_ptr->incrCurrObservedCount(updater::freeVoxel(voxel_data, map_.getDataConfig()));
        } else
        {
          const float sample_point_C_m = sensor_.measurementFromPoint(sample_point_C);
          const float range = sample_point_C.norm();
          const float range_diff = (sample_point_C_m - depth_value) * (range / sample_point_C_m);
          block_ptr->incrCurrObservedCount(updater::updateVoxel(voxel_data, range_diff, tau, three_sigma, map_.getDataConfig()));
        }
      } // x
    } // y
  } // z

  block_ptr->incrCurrIntegrCount();
}



template<typename MapT,
         typename SensorT
>
void MultiresOFusionUpdater<MapT, SensorT>::freeNodeRecurse(se::OctantBase* octant_ptr,
                                                            int             depth)
{
  NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);
  
  if (node_ptr->getChildrenMask() == 0)
  {
    /// CHANGED
    typename NodeType::DataType node_data = node_ptr->getData();
    updater::freeNode(node_data, map_.getDataConfig());
    node_ptr->setData(node_data);
#pragma omp critical (node_lock)
    { // Add node to node list for later up-propagation (finest node for this tree-branch)
      node_set_[depth - 1].insert(node_ptr->getParent());
    }
  } else
  {
    for (int child_idx = 0; child_idx < 8; child_idx++)
    {
      se::OctantBase* child_ptr = node_ptr->getChild(child_idx);
      if (!child_ptr)
      {
        child_ptr = octree_.allocate(node_ptr, child_idx);
      }

      if (child_ptr->isBlock())
      {
        // Voxel block has a low variance. Update data at a minimum
        // free space integration scale or finer/coarser (depending on later scale selection).
        freeBlock(child_ptr); // TODO: Add to block_list?
#pragma omp critical (node_lock)
        { // Add node to node list for later up-propagation (finest node for this tree-branch)
          node_set_[depth - 1].insert(child_ptr->getParent());
        }
#pragma omp critical (block_lock)
        { // Add node to node list for later up-propagation (finest node for this tree-branch)
          freed_block_list_.push_back(child_ptr);
        }
      } else
      {
        freeNodeRecurse(child_ptr, depth + 1);
      }

    }
  }
}



} // namespace se



#endif //SE_MULTIRES_OFUSION_UPDATER_IMPL_HPP
