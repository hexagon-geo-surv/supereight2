/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MATH_UTIL_HPP
#define SE_MATH_UTIL_HPP



#include <Eigen/Geometry>
#include <cmath>
#include <vector>



namespace se {
namespace math {

/** The value used for a normal vector that can't be computed. */
inline const Eigen::Vector3f g_invalid_normal = Eigen::Vector3f::Zero();

template<typename T>
constexpr bool is_power_of_two(const T x);

constexpr int log2_const(int n);

static inline unsigned power_two_up(const float x);

template<typename T>
T fracf(const T& v);

template<typename T>
T floorf(const T& v);

template<typename T>
T fabs(const T& v);

template<typename Scalar>
constexpr Scalar sq(Scalar a);

template<typename Scalar>
constexpr Scalar cu(Scalar a);

template<typename Scalar>
bool in(const Scalar v, const Scalar a, const Scalar b);

/*! \brief Compute the normal vector of a plane defined by 3 points.
 * The direction of the normal depends on the order of the points.
 */
static inline Eigen::Vector3f
plane_normal(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);

/**
 * \brief hat-operator
 *
 * It takes in the 3-vector representation ``omega`` (= rotation vector) and
 * returns the corresponding matrix representation of Lie algebra element.
 *
 * Formally, the hat()-operator of SO(3) is defined as
 *
 *   ``hat(.): R^3 -> R^{3x3},  hat(omega) = sum_i omega_i * G_i``
 *   (for i=0,1,2)
 *
 * with ``G_i`` being the ith infinitesimal generator of SO(3).
 *
 * The corresponding inverse is the vee()-operator, see below.
 *
 * \param[in] omega rotation vector
 *
 * \return Corresponding matrix representation of Lie algebra element.
 */
static inline Eigen::Matrix3f hat(const Eigen::Vector3f& omega);

static inline Eigen::Matrix3f exp_and_theta(const Eigen::Vector3f& omega, float& theta);

/**
 * \brief Group exponential
 *
 * This functions takes in an element of tangent space (= twist ``a``) and
 * returns the corresponding element of the group SE(3).
 *
 * The first three components of ``a`` represent the translational part
 * ``upsilon`` in the tangent space of SE(3), while the last three components
 * of ``a`` represents the rotation vector ``omega``.
 * To be more specific, this function computes ``expmat(hat(a))`` with
 * ``expmat(.)`` being the matrix exponential and ``hat(.)`` the hat-operator
 * of SE(3), see below.
 */
static inline Eigen::Matrix4f exp(const Eigen::Matrix<float, 6, 1>& a);



} // namespace math
} // namespace se



#include "impl/math_util_impl.hpp"



#endif // SE_MATH_UTIL_HPP
