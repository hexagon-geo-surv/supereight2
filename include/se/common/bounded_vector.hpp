/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COMMON_BOUNDED_VECTOR_HPP
#define SE_COMMON_BOUNDED_VECTOR_HPP

#include <vector>

#include "math_util.hpp"

namespace se {

namespace detail {

// An allocator using a static array of N elements of type T. It's not expected to work correctly
// for any container other than std::vector.
template<typename T, std::size_t N>
class ArrayAllocator {
    static_assert(math::is_power_of_two(N), "std::vector typically allocates powers of 2");

    T data_[N];

    public:
    typedef T value_type;
    typedef std::true_type propagate_on_container_copy_assignment;
    typedef std::true_type propagate_on_container_move_assignment;
    typedef std::true_type propagate_on_container_swap;

    static constexpr std::size_t capacity = N;

    template<typename U>
    struct rebind {
        using other = ArrayAllocator<U, N>;
    };

    T* allocate(std::size_t n)
    {
        if (n > N) {
            throw std::bad_alloc();
        }
        return data_;
    }

    void deallocate(T*, std::size_t)
    {
    }

    template<typename U, std::size_t M>
    bool operator==(const ArrayAllocator<U, M>& rhs)
    {
        // Each instance of ArrayAllocator can only deallocate objects that were allocated by
        // itself so it should compare equal only with itself.
        return this == &rhs;
    }

    template<typename U, std::size_t M>
    bool operator!=(const ArrayAllocator<U, M>& rhs)
    {
        return !(*this == rhs);
    }
};

} // namespace detail



/** A statically-allocated std::vector that can store at most \p N elements. Allows avoiding
 * dynamic memory allocation when the maximum size of a vector is known at compile-time.
 *
 * \note \p N must be a power of 2 due to how std::vector typically grows the internal buffer.
 */
template<typename T, std::size_t N>
using BoundedVector = std::vector<T, detail::ArrayAllocator<T, N>>;

} // namespace se

#endif // SE_COMMON_BOUNDED_VECTOR_HPP
