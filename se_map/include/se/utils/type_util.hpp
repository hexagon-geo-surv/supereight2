#ifndef SE_TYPE_UTIL_HPP
#define SE_TYPE_UTIL_HPP

#include <cstdint>
#include <vector>
#include <set>
#include <unordered_set>
#include <array>
#include <Eigen/Dense>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

namespace se {

/**
 * \brief key = 1 bit buffer + 57 bits of morton code + 6 bits of scale information
 *        The maxium scale is limited by 57 / 3 = 19 scales
 *
 *
 * \note uint64_t has 64 bits
 *       We'll use the key to store both, the Morton code and scale information
 *       In 3D three bits are required to encode the Morton code at each scale.
 *       21 scales -> 3 bits * 21 = 63 bits; 64 bits - 63 bits = 1 bit  to encode unsigned int up to 21 [not possible]
 *       20 scales -> 3 bits * 20 = 60 bits; 64 bits - 60 bits = 4 bits to encode unsigned int up to 20 [not possible]
 *       19 scales -> 3 bits * 19 = 57 bits; 64 bits - 57 bits = 7 bits to encode unsigend int up to 19 [possible]
 *
 *       The tree cannot allocate any depth further than 19 allowing a map size = 524288 * map resolution
 *       That is a maximum map size of 1 x 1 x 1 km^3 at 2 mm resolution
 */
typedef uint64_t key_t;       ///< The type of the Key i.e. code | scale
typedef uint64_t code_t;      ///< The type of the Morton code
typedef uint64_t scale_t;     ///< The type of the scale in the morton code

typedef unsigned int idx_t;   ///< Child or voxel index type

typedef float depth_t;        ///< The type of the processed depth measurements

typedef float field_t;        ///< The type of the stored field (e.g. TSDF, ESDF or occupancy)

typedef Eigen::Matrix<field_t, 3, 1> field_vec_t;

} // namespace se


#endif // SE_TYPE_UTIL_HPP
