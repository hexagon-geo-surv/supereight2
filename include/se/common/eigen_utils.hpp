/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_EIGEN_UTILS_HPP
#define SE_COMMON_EIGEN_UTILS_HPP

#include <Eigen/Geometry>

namespace se {
/** Helper functions for Eigen objects. */
namespace eigen {

/** Morphologically dilate or erode an axis aligned \p box by some \p distance. If \p distance is
 * positive then \p box will be dilated (increasing its volume), otherwise it will be eroded
 * (decreasing its volume).
 *
 * \warning Erosion may result in an empty axis aligned box. Make sure to test with
 * Eigen::AlignedBox::isEmpty() before using it to avoid undefined behavior.
 */
template<typename ScalarT, int Dim>
Eigen::AlignedBox<ScalarT, Dim> dilate_erode(const Eigen::AlignedBox<ScalarT, Dim>& box,
                                             const ScalarT distance);

/** Transform a 3D axis aligned box \p box_A expressed in frame A to frame B using the transform \p
 * T_BA.
 */
template<typename ScalarT, int Mode, int Options>
Eigen::AlignedBox<ScalarT, 3> transform(const Eigen::Transform<ScalarT, 3, Mode, Options>& T_BA,
                                        const Eigen::AlignedBox<ScalarT, 3>& box_A);

/** Clamp the coefficients of \p v between those of \p low and \p high. */
template<typename T, typename U, typename V>
void clamp(Eigen::MatrixBase<T>& x,
           const Eigen::MatrixBase<U>& low,
           const Eigen::MatrixBase<V>& high);

/** \overload */
template<typename T, typename U, typename V>
void clamp(Eigen::ArrayBase<T>& x, const Eigen::ArrayBase<U>& low, const Eigen::ArrayBase<V>& high);

} // namespace eigen
} // namespace se

#include "impl/eigen_utils_impl.hpp"

#endif // SE_COMMON_EIGEN_UTILS_HPP
