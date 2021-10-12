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
inline float compute_three_sigma(const se::field_t depth_value,
                                 const float sigma_min,
                                 const float sigma_max,
                                 const ConfigT config);



/**
 * \brief Compute the estimated wall thickness tau for a given depth measurement.
 *
 * \param[in] depth_value The measured depth of the depth image.
 *
 * \return The estimated wall thickness.
 */
template<typename ConfigT>
inline float compute_tau(const field_t depth_value,
                         const float tau_min,
                         const float tau_max,
                         const ConfigT config);



namespace updater {


/**
 * \brief Update the weighted mean log-odd octant occupancy and set the octant to observed.
 *
 * \param[in,out] data         The data in the octant.
 * \param[in] sample_value The sample occupancy to be integrated.
 *
 * \return True/false if the voxel has been observed the first time
 */
template<typename DataT>
inline bool
weightedMeanUpdate(DataT& data, const float sample_value, const se::weight_t max_weight);



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
inline bool updateVoxel(DataT& data,
                        const float range_diff,
                        const float tau,
                        const float three_sigma,
                        const ConfigT config);



/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template<typename DataT, typename ConfigT>
inline void freeNode(DataT& node_data, const ConfigT config);


/**
 * \brief Reduce the node data by the minimum log-odd occupancy update per iteration.
 *        This function can be used to faster update a octant if it's know that it is in free space.
 *        The aim is to increase computation time by avoiding to compute the sample value from scratch.
 *
 * \param[in,out] node_data The reference to the node data.
 */
template<typename DataT, typename ConfigT>
inline bool freeVoxel(DataT& voxel_data, const ConfigT config);


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
inline typename NodeT::DataType
propagateToNoteAtCoarserScale(se::OctantBase* octant_ptr,
                              const unsigned int /* voxel_depth */, // TODO:
                              const unsigned int frame);



/**
 * \brief Summariese the values from the current integration scale recursively
 *        up to the block's max scale.
 *
 * \param[in] block         The block to be updated.
 * \param[in] initial_scale Scale from which propagate up voxel values
*/
template<typename BlockT>
inline void propagateBlockToCoarsestScale(se::OctantBase* octant_ptr);



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
template<typename BlockT>
inline void maxCoarsePropagation(const se::OctantBase* octant_ptr,
                                 const Eigen::Vector3i target_coord,
                                 const int target_scale,
                                 const unsigned int target_stride,
                                 typename BlockT::DataType& target_data);



template<typename BlockT>
inline void meanCoarsePropagation(const se::OctantBase* octant_ptr,
                                  const Eigen::Vector3i target_coord,
                                  const int target_scale,
                                  const unsigned int target_stride,
                                  typename BlockT::DataType& target_data);



template<typename BlockT>
inline void propagateToVoxelAtCoarserScale(const se::OctantBase* octant_ptr,
                                           const Eigen::Vector3i voxel_coord,
                                           const int target_scale,
                                           const unsigned int target_stride,
                                           typename BlockT::DataType& voxel_data);



} // namespace updater
} // namespace se

#include "impl/multires_ofusion_core_impl.hpp"

#endif // SE_MULTIRES_OFUSION_CORE_HPP
