// SPDX-FileCopyrightText: 2019-2020 Nils Funk, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause


#ifndef SE_MULTIRES_OFUSION_CORE_HPP
#define SE_MULTIRES_OFUSION_CORE_HPP

#include <limits>

#include "se/common/str_utils.hpp"
#include "se/common/math_util.hpp"

namespace se {



/**
 * \brief Compute the estimated uncertainty boundary for a given depth measurement.
 *
 * \param[in] depth_value The measured depth of the depth image.
 *
 * \return Three sigma uncertainty.
 */
template <typename ConfigT>
inline float compute_three_sigma(const se::field_t depth_value,
                                 const float       sigma_min,
                                 const float       sigma_max,
                                 const ConfigT     config)
{
  if (config.uncertainty_model == se::UncertaintyModel::Linear)
  {
    return 3 * se::math::clamp(config.k_sigma * depth_value, sigma_min, sigma_max); // Livingroom dataset
  } else
  {
    return 3 * se::math::clamp(config.k_sigma * se::math::sq(depth_value), sigma_min, sigma_max); // Cow and lady
  }
}



/**
 * \brief Compute the estimated wall thickness tau for a given depth measurement.
 *
 * \param[in] depth_value The measured depth of the depth image.
 *
 * \return The estimated wall thickness.
 */
template <typename ConfigT>
inline float compute_tau(const field_t depth_value,
                         const float   tau_min,
                         const float   tau_max,
                         const ConfigT config)
{
  if (config.const_surface_thickness)
  {
    return tau_max; ///<< e.g. used in ICL-NUIM livingroom dataset.
  } else
  {
    return se::math::clamp(config.k_tau * depth_value, tau_min, tau_max);
  }
}



namespace updater {


/**
 * \brief Update the weighted mean log-odd octant occupancy and set the octant to observed.
 *
 * \param[in,out] data         The data in the octant.
 * \param[in] sample_value The sample occupancy to be integrated.
 *
 * \return True/false if the voxel has been observed the first time
 */
template <typename DataT>
inline bool weightedMeanUpdate(DataT&             data,
                               const float        sample_value,
                               const se::weight_t max_weight)
{
  data.occupancy = (data.occupancy * data.weight + sample_value) / (data.weight + 1);
  data.weight    = std::min((data.weight + 1), max_weight);
  if (data.observed)
  {
    return false;
  } else
  {
    data.observed = true;
    return true;
  }
}



/**
 * \brief Update a field with a new measurement, a weighting of 1 is considered for the new measurement.
 *
 * \param[in]     range_diff  The range difference between the voxel sample point and the depth value of the reprojection.
 * \param[in]     tau         The estimated wall thickness.
 * \param[in]     three_sigma The 3x sigma uncertainty.
 * \param[in,out] voxel_data  The reference to the voxel data of the voxel to be updated.
 *
 * \return True/false if the node has been observed the first time
 */
template <typename DataT,
          typename ConfigT
>
inline bool updateVoxel(DataT&        data,
                        const float   range_diff,
                        const float   tau,
                        const float   three_sigma,
                        const ConfigT config)
{
  float sample_value;

  if (range_diff < -three_sigma)
  {
    sample_value = config.log_odd_min;
  } else if (range_diff < tau / 2)
  {
    sample_value = std::min(config.log_odd_min - config.log_odd_min / three_sigma * (range_diff + three_sigma), config.log_odd_max);
  } else if (range_diff < tau)
  {
    sample_value = std::min(-config.log_odd_min * tau / (2 * three_sigma), config.log_odd_max);
  } else
  {
    return false;
  }

  return weightedMeanUpdate(data, sample_value, config.max_weight);
}



/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template <typename DataT,
          typename ConfigT
>
inline void freeNode(DataT&        node_data,
                     const ConfigT config)
{
  weightedMeanUpdate(node_data, config.log_odd_min, config.max_weight);
}


/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template <typename DataT,
          typename ConfigT
>
inline bool freeVoxel(DataT&        voxel_data,
                      const ConfigT config)
{
  return weightedMeanUpdate(voxel_data, config.log_odd_min, config.max_weight);
}


/**
 * \brief Propagate a summary of the eight nodes children to its parent
 *
 * \param[in] node        Node to be summariesed
 * \param[in] voxel_depth Maximum depth of the octree
 * \param[in] frame       Current frame
 *
 * \return data Summary of the node
 */
template <typename NodeT, typename BlockT>
inline typename NodeT::DataType propagateToNoteAtCoarserScale(se::OctantBase*    octant_ptr,
                                                              const unsigned int /* voxel_depth */, // TODO:
                                                              const unsigned int frame)
{
  NodeT* node_ptr = static_cast<NodeT*>(octant_ptr);

  node_ptr->setTimeStamp(frame);

  float max_mean_occupancy = 0;
  short max_weight = 0;
  float max_occupancy = -std::numeric_limits<float>::max();
  unsigned int observed_count = 0;
  unsigned int data_count = 0;

  for(unsigned int child_idx = 0; child_idx < 8; ++child_idx)
  {
    se::OctantBase* child_ptr = node_ptr->getChild(child_idx);

    if (!child_ptr)
    {
      continue;
    }

    const auto& child_data = (child_ptr->isBlock()) ? static_cast<BlockT*>(child_ptr)->getMaxData() : static_cast<NodeT*>(child_ptr)->getMaxData();
    if (child_data.weight > 0 && child_data.occupancy * child_data.weight > max_occupancy) // At least 1 integration
    {
      data_count++;
      max_mean_occupancy = child_data.occupancy;
      max_weight = child_data.weight;
      max_occupancy = max_mean_occupancy * max_weight;
    }
    if (child_data.observed == true)
    {
      observed_count++;
    }
  }

  /// CHANGED
//  const unsigned int child_idx = se::child_idx(node_ptr->code(), se::keyops::depth(node_ptr->code()), voxel_depth);
//  auto& node_data = node_ptr->getParent()->childData(child_idx);
  typename NodeT::DataType node_data = node_ptr->getData();

  if (data_count > 0)
  {
    node_data.occupancy = max_mean_occupancy; // TODO: Need to check update?
    node_data.weight    = max_weight;
    if (observed_count == 8)
    {
      node_data.observed = true;
    }
    node_ptr->setData(node_data);
  }
  return node_data;
}



/**
 * \brief Summariese the values from the current integration scale recursively
 *        up to the block's max scale.
 *
 * \param[in] block         The block to be updated.
 * \param[in] initial_scale Scale from which propagate up voxel values
*/
template <typename BlockT>
inline void propagateBlockToCoarsestScale(se::OctantBase* octant_ptr)
{
  typedef typename BlockT::DataType DataType;

  BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);
  int          target_scale = block_ptr->getCurrentScale() + 1;
  unsigned int size_at_target_scale_li = BlockT::size >> target_scale;
  unsigned int size_at_target_scale_sq = se::math::sq(size_at_target_scale_li);

  int          child_scale  = target_scale - 1;
  unsigned int size_at_child_scale_li = BlockT::size >> child_scale;
  unsigned int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

  DataType min_data;
  float o_min;

  if (block_ptr->buffer_scale() > block_ptr->getCurrentScale())
  {
    DataType* max_data_at_target_scale = block_ptr->blockMaxDataAtScale(target_scale);
    DataType* max_data_at_child_scale =  block_ptr->blockDataAtScale(child_scale);

    min_data = max_data_at_child_scale[0];
    o_min = min_data.occupancy * min_data.weight;

    for(unsigned int z = 0; z < size_at_target_scale_li; z++)
    {
      for (unsigned int y = 0; y < size_at_target_scale_li; y++)
      {
        for (unsigned int x = 0; x < size_at_target_scale_li; x++)
        {
          const int target_max_data_idx   = x + y * size_at_target_scale_li + z * size_at_target_scale_sq;
          auto& target_max_data = max_data_at_target_scale[target_max_data_idx];

          float max_mean_occupancy = 0;
          short max_weight = 0;
          float max_occupancy = -std::numeric_limits<float>::max();

          int observed_count = 0;
          int data_count = 0;

          for (unsigned int k = 0; k < 2; k++)
          {
            for (unsigned int j = 0; j < 2; j++)
            {
              for (unsigned int i = 0; i < 2; i++)
              {
                const int child_max_data_idx = (2 * x + i) + (2 * y + j) * size_at_child_scale_li + (2 * z + k) * size_at_child_scale_sq;
                const auto child_data = max_data_at_child_scale[child_max_data_idx];

                float o = (child_data.occupancy * child_data.weight);

                if (child_data.weight > 0)
                {
                  if (o > max_occupancy)
                  {
                    data_count++;
                    // Update max
                    max_mean_occupancy = child_data.occupancy;
                    max_weight = child_data.weight;
                    max_occupancy = max_mean_occupancy * max_weight;
                  } else if (o < o_min)
                  {
                    min_data.occupancy = child_data.occupancy;
                    min_data.weight = child_data.weight;
                    o_min = o;
                  }
                }

                if (child_data.observed)
                {
                  observed_count++;
                }

              } // i
            } // j
          } // k

          if (data_count > 0)
          {
            target_max_data.occupancy = max_mean_occupancy;
            target_max_data.weight = max_weight;
            if (observed_count == 8)
            {
              target_max_data.observed = true; // TODO: We don't set the observed count to true for mean values
            }
          }

        } // x
      } // y
    } // z

  } else {

    DataType* max_data_at_target_scale = block_ptr->blockMaxDataAtScale(target_scale);
    DataType* data_at_target_scale     = block_ptr->blockDataAtScale(target_scale);
    DataType* data_at_child_scale      = block_ptr->blockDataAtScale(child_scale);

    min_data = data_at_child_scale[0];
    o_min = min_data.occupancy * min_data.weight;

    for(unsigned int z = 0; z < size_at_target_scale_li; z++)
    {
      for (unsigned int y = 0; y < size_at_target_scale_li; y++)
      {
        for (unsigned int x = 0; x < size_at_target_scale_li; x++)
        {
          const int target_data_idx   = x + y * size_at_target_scale_li + z * size_at_target_scale_sq;
          auto& target_data     = data_at_target_scale[target_data_idx];
          auto& target_max_data = max_data_at_target_scale[target_data_idx];

          float mean_occupancy = 0;
          short mean_weight = 0;

          float max_mean_occupancy = 0;
          short max_weight = 0;
          float max_occupancy = -std::numeric_limits<float>::max();

          int observed_count = 0;
          int data_count = 0;

          for (unsigned int k = 0; k < 2; k++)
          {
            for (unsigned int j = 0; j < 2; j++)
            {
              for (unsigned int i = 0; i < 2; i++)
              {
                const int child_data_idx = (2 * x + i) + (2 * y + j) * size_at_child_scale_li + (2 * z + k) * size_at_child_scale_sq;
                const auto child_data = data_at_child_scale[child_data_idx];

                if (child_data.weight > 0)
                {
                  // Update mean
                  data_count++;
                  mean_occupancy += child_data.occupancy;
                  mean_weight += child_data.weight;

                  float o = (child_data.occupancy * child_data.weight);

                  if (o > max_occupancy)
                  {
                    // Update max
                    max_mean_occupancy = child_data.occupancy;
                    max_weight = child_data.weight;
                    max_occupancy = max_mean_occupancy * max_weight;
                  } else if (o > o_min)
                  {
                    min_data.occupancy = child_data.occupancy;
                    min_data.weight = child_data.weight;
                    o_min = o;
                  }
                }

                if (child_data.observed)
                {
                  observed_count++;
                }

              } // i
            } // j
          } // k

          if (data_count > 0)
          {

            target_data.occupancy = mean_occupancy / data_count;
            target_data.weight = ceil((float) mean_weight) / data_count;
            target_data.observed = false;

//              target_data.occupancy = mean_occupancy / data_count;
//              target_data.weight = max_weight;
//              if (observed_count == 8) {
//                target_data.observed = true; // TODO: We don't set the observed count to true for mean values
//              }

//              target_data.occupancy = max_mean_occupancy;
//              target_data.weight = max_weight;
//              if (observed_count == 8) {
//                target_data.observed = true; // TODO: We don't set the observed count to true for mean values
//              }

            target_max_data.occupancy = max_mean_occupancy;
            target_max_data.weight = max_weight;
            if (observed_count == 8)
            {
              target_max_data.observed = true; // TODO: We don't set the observed count to true for mean values
            }

//              if (abs(target_data.occupancy - target_max_data.occupancy) > 1) {
//                std::cout << "-----" << std::endl;
//                std::cout << target_data.occupancy << "/" << target_max_data.occupancy << "/" << data_count << std::endl;
//                for (int k = 0; k < 2; k++) {
//                  for (int j = 0; j < 2; j++) {
//                    for (int i = 0; i < 2; i++) {
//
//                      const int child_data_idx = (2 * x + i) + (2 * y + j) * size_at_child_scale_li + (2 * z + k) * size_at_child_scale_sq;
//                      const auto child_data = data_at_child_scale[child_data_idx];
//
//                      std::cout << str_utils::value_to_pretty_str(child_data.occupancy, "child.x") << std::endl;
//                      std::cout << str_utils::value_to_pretty_str(child_data.weight, "child.y") << std::endl;
//
//                    } // i
//                  } // j
//                } // k
//              }
          }

        } // x
      } // y
    } // z
  }



  for(target_scale += 1; target_scale <= BlockT::getMaxScale(); ++target_scale)
  {
    unsigned int size_at_target_scale_li = BlockT::size >> target_scale;
    unsigned int size_at_target_scale_sq = se::math::sq(size_at_target_scale_li);

    int          child_scale            = target_scale - 1;
    unsigned int size_at_child_scale_li = BlockT::size >> child_scale;
    unsigned int size_at_child_scale_sq = se::math::sq(size_at_child_scale_li);

    DataType* max_data_at_target_scale = block_ptr->blockMaxDataAtScale(target_scale);
    DataType* data_at_target_scale     = block_ptr->blockDataAtScale(target_scale);
    DataType* max_data_at_child_scale  = block_ptr->blockMaxDataAtScale(child_scale);
    DataType* data_at_child_scale      = block_ptr->blockDataAtScale(child_scale);

    for(unsigned int z = 0; z < size_at_target_scale_li; z++)
    {
      for (unsigned int y = 0; y < size_at_target_scale_li; y++)
      {
        for (unsigned int x = 0; x < size_at_target_scale_li; x++)
        {
          const int target_data_idx   = x + y * size_at_target_scale_li + z * size_at_target_scale_sq;
          auto& target_data     = data_at_target_scale[target_data_idx];
          auto& target_max_data = max_data_at_target_scale[target_data_idx];

          float mean_occupancy = 0;
          short mean_weight = 0;

          float max_mean_occupancy = 0;
          short max_weight = 0;
          float max_occupancy = -std::numeric_limits<float>::max();

          int observed_count = 0;
          int data_count = 0;

          for (unsigned int k = 0; k < 2; k++)
          {
            for (unsigned int j = 0; j < 2; j++)
            {
              for (unsigned int i = 0; i < 2; i++)
              {

                const int child_data_idx = (2 * x + i) + (2 * y + j) * size_at_child_scale_li + (2 * z + k) * size_at_child_scale_sq;
                const auto child_data     = data_at_child_scale[child_data_idx];
                const auto child_max_data = max_data_at_child_scale[child_data_idx];

                if (child_max_data.weight > 0) {
                  // Update mean
                  data_count++;
                  mean_occupancy += child_data.occupancy;
                  mean_weight += child_data.weight;

                  if ((child_max_data.occupancy * child_max_data.weight) > max_occupancy)
                  {
                    // Update max
                    max_mean_occupancy = child_max_data.occupancy;
                    max_weight = child_max_data.weight;
                    max_occupancy = max_mean_occupancy * max_weight;
                  }

                }

                if (child_max_data.observed)
                {
                  observed_count++;
                }

              } // i
            } // j
          } // k

          if (data_count > 0)
          {
            target_data.occupancy = mean_occupancy / data_count;
            target_data.weight = ceil((float) mean_weight) / data_count;
            target_data.observed = false;

//              target_data.occupancy = mean_occupancy / data_count;
//              target_data.weight = ceil((float) mean_weight) / data_count;
//              if (observed_count == 8) {
//                target_data.observed = true; // TODO: We don't set the observed count to true for mean values
//              }

//              target_data.occupancy = max_mean_occupancy;
//              target_data.weight = max_weight;
//              if (observed_count == 8) {
//                target_data.observed = true; // TODO: We don't set the observed count to true for mean values
//              }

            target_max_data.occupancy = max_mean_occupancy;
            target_max_data.weight = max_weight;
            if (observed_count == 8)
            {
              target_max_data.observed = true; // TODO: We don't set the observed count to true for mean values
            }
          }

        } // x
      } // y
    } // z

  }

  block_ptr->setMinData(min_data);

}



/**
 * \brief Propagate the maximum log-odd occupancy of the eight children up to the next scale.
 *
 * \note UNUSED FUNCTION
 *
 * \note The maximum log-odd occupancy is the maximum partly observed log-odd occupancy.
 *       To check if the node is safe for planning, verify if the log-odd occupancy is below a chosen threshold
 *       and that the node is observed (data.observed == true -> node or all children have been seen).
 *
 * \note The maximum log-odd occupancy is the maximum product of data.occupancy * data.weight and
 *       not the maximum mean log-odd Occupancy data.occupancy.
 *
 * \param[in,out] block           The block in which the target voxel is included.
 * \param[in]     voxel_coord     The coordinates of the target voxel (corner).
 * \param[in]     target_scale    The scale to which to propage the data to.
 * \param[in]     target_stride   The stride in voxel units of the target scale.
 * \param[in,out] target_data     The data reference to the target voxel.
 */
template <typename BlockT>
inline void maxCoarsePropagation(const se::OctantBase*      octant_ptr,
                                 const Eigen::Vector3i      target_coord,
                                 const int                  target_scale,
                                 const unsigned int         target_stride,
                                 typename BlockT::DataType& target_data)
{
  BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

  float max_mean_occupancy = -std::numeric_limits<float>::max();
  short max_weight = 0;
  float max_occupancy = -std::numeric_limits<float>::max();
  unsigned int observed_count = 0;
  unsigned int data_count = 0;

  int          child_scale  = target_scale - 1;
  unsigned int child_stride = target_stride >> 1; ///<< Halfen target stride

  for (unsigned int k = 0; k < target_stride; k += child_stride)
  {
    for (unsigned int j = 0; j < target_stride; j += child_stride)
    {
      for (unsigned int i = 0; i < target_stride; i += child_stride)
      {
        const auto child_data = block_ptr->data(target_coord + Eigen::Vector3i(i, j, k), child_scale);
        /// Only compare partly observed children (child_data.weight > 0)
        /// max_mean_occupancy is the product of data.occupancy * data.weight (i.e. not the mean log-odd occupancy)
        if (child_data.weight > 0 && ((child_data.occupancy * child_data.weight) > max_occupancy))
        {
          data_count++;
          max_mean_occupancy = child_data.occupancy;
          max_weight = child_data.weight;
          max_occupancy = max_mean_occupancy * max_weight;
        }

        if (child_data.observed)
        {
          observed_count++;
        }

      } // i
    } // j
  } // k

  if (data_count > 0)
  {
    target_data.occupancy = max_mean_occupancy;
    target_data.weight    = max_weight;
    if (observed_count == 8)
    { ///<< If all children have been observed, set parent/target to observed.
      target_data.observed = true;
    }
  }
}



template <typename BlockT>
inline void meanCoarsePropagation(const se::OctantBase*      octant_ptr,
                                  const Eigen::Vector3i      target_coord,
                                  const int                  target_scale,
                                  const unsigned int         target_stride,
                                  typename BlockT::DataType& target_data)
{
  BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);

  float mean_occupancy        = 0;
  short mean_weight           = 0;
  unsigned int observed_count = 0;
  unsigned int data_count     = 0;

  int          child_scale  = target_scale - 1;
  unsigned int child_stride = target_stride >> 1;

  for (unsigned int k = 0; k < target_stride; k += child_stride)
  {
    for (unsigned int j = 0; j < target_stride; j += child_stride)
    {
      for (unsigned int i = 0; i < target_stride; i += child_stride)
      {
        auto child_data = block_ptr->data(target_coord + Eigen::Vector3i(i, j, k), child_scale);
        if (child_data.weight > 0)
        {
          data_count++;
          mean_occupancy += child_data.occupancy;
          mean_weight += child_data.weight;
        }
        if (child_data.observed)
        {
          observed_count++;
        }
      }
    }
  }

  if (data_count == 8)
  {
    target_data.occupancy = mean_occupancy / data_count;
    target_data.weight = ((float) mean_weight) / data_count;
    target_data.observed = true;

//      // TODO: ^SWITCH 2 - Set observed if all children are known.
//      if (observed_count == 8) {
//        target_data.observed = true;
//      }
  }
}



template <typename BlockT>
inline void propagateToVoxelAtCoarserScale(const se::OctantBase*      octant_ptr,
                                           const Eigen::Vector3i      voxel_coord,
                                           const int                  target_scale,
                                           const unsigned int         target_stride,
                                           typename BlockT::DataType& voxel_data)
{
  BlockT* block_ptr = static_cast<BlockT*>(octant_ptr);
  maxCoarsePropagation(block_ptr, voxel_coord, target_scale, target_stride, voxel_data);
}



} // namespace updater
} // namespace se

#endif //SE_MULTIRES_OFUSION_CORE_HPP
