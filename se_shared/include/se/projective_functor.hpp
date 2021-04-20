/*
    Copyright 2016 Emanuele Vespa, Imperial College London
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
    may be used to endorse or promote products derived from this software without
    specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef PROJECTIVE_FUNCTOR_HPP
#define PROJECTIVE_FUNCTOR_HPP
#include <functional>
#include <vector>

#include "se/utils/math_utils.h"
#include "filter.hpp"
#include "se/node.hpp"
#include "se/functors/data_handler.hpp"
#include "se/sensor_implementation.hpp"

namespace se {
namespace functor {

template <typename DataType, template <typename DataT> class OctreeT,
        typename UpdateF>
class projective_functor {

using VoxelBlockType = typename DataType::VoxelBlockType;

public:
  projective_functor(OctreeT<DataType>&      octree,
                     UpdateF&                update_funct,
                     const Eigen::Matrix4f&  T_CM,
                     const SensorImpl        sensor,
                     const se::Image<float>& image,
                     const Eigen::Vector3f&  sample_offset_frac) :
    octree_(octree),
    update_funct_(update_funct),
    T_CM_(T_CM),
    sensor_(sensor),
    image_(image),
    sample_offset_frac_(sample_offset_frac) {
  }



  /*! \brief Get all the blocks that are active or inside the camera
   * frustum. The blocks are stored in projective_functor::active_list_.
   */
  void build_active_list() {
    using namespace std::placeholders;
    /* Retrieve the active list */
    const typename DataType::template MemoryBufferType<VoxelBlockType>& block_buffer = octree_.pool().blockBuffer();

    /* Predicates definition */
    const float voxel_dim = octree_.dim() / octree_.size();
    auto in_frustum_predicate =
      std::bind(algorithms::in_frustum<VoxelBlockType>,
          std::placeholders::_1, voxel_dim, T_CM_, sensor_);
    auto is_active_predicate = [](const VoxelBlockType* block) {
      return block->active();
    };

    /* Get all the blocks that are active or inside the camera frustum. */
    algorithms::filter(active_list_, block_buffer, is_active_predicate,
        in_frustum_predicate);
  }



  void update_block(VoxelBlockType* block,
                    const float     voxel_dim) {

    update_funct_.reset(block);
    block->current_scale(0);

    const Eigen::Vector3i voxel_coord_base = block->coordinates();
    const unsigned int scale_voxel_size    = block->scaleVoxelSize(block->current_scale());
    auto valid_predicate = [&](float depth_value){ return depth_value >= sensor_.near_plane; };

    const Eigen::Vector3f voxel_sample_coord_base_f   = se::get_sample_coord(voxel_coord_base, scale_voxel_size, sample_offset_frac_);
    const Eigen::Vector3f sample_point_base_C         = (T_CM_ * (voxel_dim * voxel_sample_coord_base_f).homogeneous()).head(3);
    const Eigen::Matrix3f sample_point_delta_matrix_C = (se::math::to_rotation(T_CM_) * voxel_dim * Eigen::Matrix3f::Identity());

    const unsigned int scale_size = block->scaleSize(block->current_scale());

    bool is_visible = false;

    for (unsigned int k = 0; k < scale_size; k++) {
      for (unsigned int j = 0; j < scale_size; j++) {
#pragma omp simd
        for (unsigned int i = 0; i < scale_size; i++) {

          // Set voxel coordinates
          Eigen::Vector3i voxel_coord = voxel_coord_base + scale_voxel_size * Eigen::Vector3i(i, j, k);

          // Set sample point in camera frame
          Eigen::Vector3f sample_point_C = sample_point_base_C + sample_point_delta_matrix_C * Eigen::Vector3f(i, j, k);

          if (sample_point_C.norm() > sensor_.farDist(sample_point_C)) {
            continue;
          }

          // Fetch image value
          float image_value(0);
          if (!sensor_.projectToPixelValue(sample_point_C, image_, image_value, valid_predicate)) {
            continue;
          }

          is_visible = true;

          /* Update the voxel. */
          VoxelBlockHandler<DataType> handler = {block, voxel_coord};
          update_funct_(handler, sample_point_C, image_value);

        } // i
      } // j
    } // k

    update_funct_(block, is_visible);
  }



  void update_node(se::Node<DataType>* node,
                   const float         voxel_dim) {

    const Eigen::Vector3i node_coord = node->coordinates();

    /* Iterate over the Node children. */
#pragma omp simd
    for(int child_idx = 0; child_idx < 8; ++child_idx) {
      const unsigned int child_size  = node->size() / 2;
      const Eigen::Vector3i rel_step = Eigen::Vector3i((child_idx & 1) > 0, (child_idx & 2) > 0, (child_idx & 4) > 0);
      const Eigen::Vector3i child_coord = node_coord + child_size * rel_step;
      const Eigen::Vector3f child_point_C = (T_CM_ * (voxel_dim * (child_coord.cast<float>() + child_size *
          sample_offset_frac_)).homogeneous()).head(3);

      if (child_point_C.norm() > sensor_.farDist(child_point_C)) {
        continue;
      }

      float image_value(0);
      if (!sensor_.projectToPixelValue(child_point_C, image_, image_value,
          [&](float depth_value){ return depth_value >= sensor_.near_plane; })) {
        continue;
      }

      /* Update the child Node. */
      NodeHandler<DataType> handler = {node, child_idx};
      update_funct_(handler, child_point_C, image_value);
    }
  }



  void apply() {

    const float voxel_dim = octree_.voxelDim();

    /* Update the leaf Octree nodes (VoxelBlock). */
    build_active_list();
#pragma omp parallel for
    for (unsigned int i = 0; i < active_list_.size(); ++i) {
      update_block(active_list_[i], voxel_dim);
    }
    active_list_.clear();

    /* Update the intermediate Octree nodes (Node). */
    typename DataType::template MemoryBufferType<se::Node<DataType>>& node_buffer = octree_.pool().nodeBuffer();
#pragma omp parallel for
      for (unsigned int i = 0; i < node_buffer.size(); ++i) {
        update_node(node_buffer[i], voxel_dim);
     }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  OctreeT<DataType>& octree_;
  UpdateF& update_funct_;
  const Eigen::Matrix4f& T_CM_;
  const SensorImpl sensor_;
  const se::Image<float>& image_;
  const Eigen::Vector3f sample_offset_frac_;
  std::vector<VoxelBlockType*> active_list_;
};

/*! \brief Create a projective_functor and call projective_functor::apply.
 */
template <typename DataType, template <typename DataT> class OctreeT,
          typename UpdateF>
void projective_octree(OctreeT<DataType>&      octree,
                       const Eigen::Vector3f&  sample_offset_frac,
                       const Eigen::Matrix4f&  T_CM,
                       const SensorImpl&       sensor,
                       const se::Image<float>& image,
                       UpdateF&                funct) {

  projective_functor<DataType, OctreeT, UpdateF>
    it(octree, funct, T_CM, sensor, image, sample_offset_frac);
  it.apply();
}
}
}
#endif
