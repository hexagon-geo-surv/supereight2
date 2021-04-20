/*
 * Copyright 2016 Emanuele Vespa, Imperial College London
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * */

#ifndef ACTIVE_LIST_HPP
#define ACTIVE_LIST_HPP

#include "se/utils/math_utils.h"
#include "se/node.hpp"
#include "se/utils/memory_pool.hpp"
#include "se/utils/morton_utils.hpp"
#include "se/sensor_implementation.hpp"

namespace se {
namespace algorithms {

  template <typename VoxelBlockType>
    static inline bool in_frustum(const VoxelBlockType*  block,
                                  const float            voxel_dim,
                                  const Eigen::Matrix4f& T_CM,
                                  const SensorImpl&      sensor) {

      const int block_size = VoxelBlockType::size_li;
      const static Eigen::Matrix<int, 4, 8> corner_rel_steps =
        (Eigen::Matrix<int, 4, 8>() << 0, block_size, 0   , block_size, 0   , block_size, 0   , block_size,
                                       0, 0   , block_size, block_size, 0   , 0   , block_size, block_size,
                                       0, 0   , 0   , 0   , block_size, block_size, block_size, block_size,
                                       0, 0   , 0   , 0   , 0   , 0   , 0   , 0   ).finished();
      const Eigen::Matrix3Xf block_corner_points_C = (T_CM * Eigen::Vector4f(voxel_dim, voxel_dim, voxel_dim, 1.f).asDiagonal() *
          (corner_rel_steps.colwise() + block->coordinates().homogeneous()).template cast<float>()).topRows(3);
      Eigen::Matrix2Xf proj_corner_pixels_f(2, 8);
      std::vector<srl::projection::ProjectionStatus> proj_corner_stati;
      sensor.model.projectBatch(block_corner_points_C, &proj_corner_pixels_f, &proj_corner_stati);
      return std::all_of(proj_corner_stati.begin(), proj_corner_stati.end(),
          [](const auto it){ return it == srl::projection::ProjectionStatus::Successful; });
    }

  template <typename ValueType, typename P>
    bool satisfies(const ValueType& el, P predicate) {
      return predicate(el);
    }

  template <typename ValueType, typename P, typename... Ps>
    bool satisfies(const ValueType& el, P predicate, Ps... others) {
      return predicate(el) || satisfies(el, others...);
    }

  template <typename BufferType, typename... Predicates>
  void filter(std::vector<BufferType *>&               out,
              const se::PagedMemoryBuffer<BufferType>& buffer,
              Predicates...                            ps) {
#ifdef _OPENMP
#pragma omp declare reduction (merge : std::vector<BufferType *> : omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge: out)
    for (unsigned int i = 0; i < buffer.size(); ++i) {
      if (satisfies(buffer[i], ps...)) {
        out.push_back(buffer[i]);
      }
    }
#else
    for (unsigned int i = 0; i < buffer.size(); ++i) {
      if (satisfies(buffer[i], ps...)) {
        out.push_back(buffer[i]);
      }
    }
#endif
  }
}
}
#endif
