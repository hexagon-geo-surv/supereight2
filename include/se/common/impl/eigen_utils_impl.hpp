/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_EIGEN_UTILS_IMPL_HPP
#define SE_COMMON_EIGEN_UTILS_IMPL_HPP

namespace se {
namespace eigen {

template<typename ScalarT, int Dim>
Eigen::AlignedBox<ScalarT, Dim> dilate_erode(const Eigen::AlignedBox<ScalarT, Dim>& box,
                                             const ScalarT distance)
{
    if (box.isEmpty()) {
        return box;
    }
    return Eigen::AlignedBox<ScalarT, Dim>(box.min().array() - distance,
                                           box.max().array() + distance);
}



template<typename ScalarT, int Mode, int Options>
Eigen::AlignedBox<ScalarT, 3> transform(const Eigen::Transform<ScalarT, 3, Mode, Options>& T_BA,
                                        const Eigen::AlignedBox<ScalarT, 3>& box_A)
{
    if (box_A.isEmpty()) {
        // Empty aligned boxes are the same in all frames.
        return box_A;
    }
    // Transform the corners of box_A to frame B and compute their AABB. There's no other way to
    // compute the axis-aligned box in frame B since T_BA may contain a rotation.
    // clang-format off
    const Eigen::Matrix<ScalarT, 3, 8> corners_A = (Eigen::Matrix<ScalarT, 3, 8>() <<
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::BottomLeftFloor),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::BottomRightFloor),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::TopLeftFloor),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::TopRightFloor),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::BottomLeftCeil),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::BottomRightCeil),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::TopLeftCeil),
        box_A.corner(Eigen::AlignedBox<ScalarT, 3>::CornerType::TopRightCeil)).finished();
    // clang-format on
    const Eigen::Matrix<ScalarT, 3, 8> corners_B = T_BA * corners_A;
    return Eigen::AlignedBox<ScalarT, 3>(corners_B.rowwise().minCoeff(),
                                         corners_B.rowwise().maxCoeff());
}



template<typename T, typename U, typename V>
void clamp(Eigen::MatrixBase<T>& x,
           const Eigen::MatrixBase<U>& low,
           const Eigen::MatrixBase<V>& high)
{
    x = x.array().max(low.array()).min(high.array()).matrix();
}

} // namespace eigen
} // namespace se

#endif // SE_COMMON_EIGEN_UTILS_IMPL_HPP
