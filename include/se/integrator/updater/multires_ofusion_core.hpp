/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_OFUSION_CORE_HPP
#define SE_MULTIRES_OFUSION_CORE_HPP

#include <limits>

#include "se/common/str_utils.hpp"
#include "se/map/data.hpp"
#include "se/map/octant/octant.hpp"

namespace se {



/**
 * \brief Compute the estimated uncertainty boundary for a given depth measurement.
 *
 * \param[in] depth_value The measured depth of the depth image.
 *
 * \return Three sigma uncertainty.
 */
template<typename ConfigT>
float compute_three_sigma(const field_t depth_value,
                          const float sigma_min,
                          const float sigma_max,
                          const ConfigT& config);



/**
 * \brief Compute the estimated wall thickness tau for a given depth measurement.
 *
 * \param[in] depth_value The measured depth of the depth image.
 *
 * \return The estimated wall thickness.
 */
template<typename ConfigT>
float compute_tau(const field_t depth_value,
                  const float tau_min,
                  const float tau_max,
                  const ConfigT& config);



namespace updater {


/**
 * \brief Update the weighted mean log-odd octant occupancy and set the octant to observed.
 *
 * \param[in,out] data     The data in the octant.
 * \param[in] sample_value The sample occupancy to be integrated.
 *
 * \return True/false if the voxel has been observed the first time
 */
template<typename DataT>
bool weighted_mean_update(DataT& data, const field_t sample_value, const weight_t max_weight);



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
template<typename DataT, typename ConfigT>
bool update_voxel(DataT& data,
                  const float range_diff,
                  const float tau,
                  const float three_sigma,
                  const ConfigT& config);



/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template<typename DataT, typename ConfigT>
void update_node_free(DataT& node_data, const ConfigT& config);


/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template<typename DataT, typename ConfigT>
bool update_voxel_free(DataT& voxel_data, const ConfigT& config);


/**
 * \brief Propagate a summary of the eight nodes children to its parent
 *
 * \param[in] node        Node to be summariesed
 * \param[in] voxel_depth Maximum depth of the octree
 * \param[in] frame       Current frame
 *
 * \return data Summary of the node
 */
template<typename NodeT, typename BlockT>
typename NodeT::DataType propagate_to_parent_node(OctantBase* octant_ptr, const unsigned int frame);



/**
 * \brief Summariese the values from the current integration scale recursively
 *        up to the block's max scale.
 *
 * \param[in] block         The block to be updated.
 * \param[in] initial_scale Scale from which propagate up voxel values
*/
template<typename BlockT>
void propagate_block_to_coarsest_scale(OctantBase* octant_ptr);



} // namespace updater
} // namespace se

#include "impl/multires_ofusion_core_impl.hpp"

#endif // SE_MULTIRES_OFUSION_CORE_HPP
