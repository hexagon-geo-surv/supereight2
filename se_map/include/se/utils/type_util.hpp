#ifndef SE_TYPE_UTIL_HPP
#define SE_TYPE_UTIL_HPP

#include <cstdint>
#include <vector>
#include <set>
#include <array>
#include <Eigen/Dense>
#include <Eigen/StdVector>

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


/**
 *
 * Define se containers with appropriate allocators for Eigen types.
 *
 */

template<typename T>
struct vector_impl
{
    typedef std::vector<T> type;
};

template<>
struct vector_impl<Eigen::Vector3i>
{
    typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> type;
};

template<>
struct vector_impl<Eigen::Vector3f>
{
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> type;
};

// Define dynamic alias template
template <typename T>
using vector = typename vector_impl<T>::type;


template<typename T>
struct set_impl
{
    typedef std::set<T> type;
};

template<>
struct set_impl<Eigen::Vector3i>
{
    typedef std::set<Eigen::Vector3i, std::less<Eigen::Vector3i>, Eigen::aligned_allocator<Eigen::Vector3i>> type;
};

template<>
struct set_impl<Eigen::Vector3f>
{
    typedef std::set<Eigen::Vector3f, std::less<Eigen::Vector3f>, Eigen::aligned_allocator<Eigen::Vector3f>> type;
};

// Define dynamic alias template
template <typename T>
using set = typename set_impl<T>::type;

// Define static alias template
template <typename T, size_t N>
using array = typename std::array<T, N>;

} // namespace se


#endif // SE_TYPE_UTIL_HPP
