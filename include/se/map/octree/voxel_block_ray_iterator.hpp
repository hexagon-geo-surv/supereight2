/*
* Copyright (c) 2009-2011, NVIDIA Corporation
* Copyright 2016 Emanuele Vespa, Imperial College London
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SE_VOXEL_BLOCK_RAY_ITERATOR_HPP
#define SE_VOXEL_BLOCK_RAY_ITERATOR_HPP

#include "se/map/octree/octree.hpp"

#define CAST_STACK_DEPTH 23

namespace se {



template <typename MapT>
class VoxelBlockRayIterator {
  typedef typename MapT::OctreeType::NodeType NodeType;
  typedef typename MapT::OctreeType::BlockType BlockType;

public:
  VoxelBlockRayIterator(const MapT &map,
                        const Eigen::Vector3f &ray_origin_M,
                        const Eigen::Vector3f &ray_dir_M,
                        const float near_plane,
                        const float far_plane);

  /** \brief Return a pointer to the next se::VoxelBlock along the ray.
   *
   * \return A pointer to an se::VoxelBlock or nullptr if all se::VoxelBlock
   * along the ray have been iterated through.
   */
  BlockType *next();

  /**
   * \brief The distance along the ray until the octree is entered.
   *
   * \return The distance in meters.
   */
  float tmin() const;

  /**
   * \brief The distance along the ray until the octree is exited.
   *
   * \return The distance in meters.
   */
  float tmax() const;

  /**
   * \brief The distance along the ray until the current se::VoxelBlock is
   * entered.
   *
   * \note Returns the same value as se::VoxelBlockRayIterator::tmin() if
   * se::VoxelBlockRayIterator::next() has not been called yet.
   *
   * \return The distance in meters.
   */
  float tcmin() const;

  /**
   * \brief The distance along the ray until the current se::VoxelBlock is
   * exited.
   *
   * \warning Returns an undefined value if se::VoxelBlockRayIterator::next()
   * has not been called yet.
   *
   * \return The distance in meters.
   */
  float tcmax() const;


private:
  struct StackEntry {
    int scale;
    NodeType *parent;
    float t_max;
  };


  enum STATE {
    INIT,
    ADVANCE,
    FINISHED
  };


  MapT &map_;
  const typename MapT::OctreeType &octree_;
  Eigen::Vector3f ray_origin_M_;
  Eigen::Vector3f ray_dir_M_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f t_coef_;
  Eigen::Vector3f t_bias_;
  Eigen::Vector3f t_corner_;
  StackEntry stack_[CAST_STACK_DEPTH];
  NodeType *parent_ptr_;
  se::OctantBase *child_ptr_;
  int idx_;
  int scale_;
  float scaling_;
  int min_scale_;
  int octant_mask_;
  float scale_exp2_;
  float t_min_;
  float t_min_init_;
  float t_max_;
  float t_max_init_;
  float tc_max_;
  float h_;
  STATE state_;


  /**
   * \brief Reinterpret the binary representation of a float as an int.
   */
  static inline int floatAsInt(const float value)
  {
    union float_as_int
    {
      float f;
      int i;
    };

    float_as_int u;
    u.f = value;
    return u.i;
  }


  /**
   * \brief Reinterpret the binary representation of an int as a float.
   */
  static inline float intAsFloat(const int value)
  {
    union int_as_float
    {
      int i;
      float f;
    };

    int_as_float u;
    u.i = value;
    return u.f;
  }

  /** \brief Advance the ray.
   */
  inline void advance_ray();

  /**
   * \brief Descend the hiararchy and compute the next child position.
   */
  inline void descend();
};



} // namespace se



#include "impl/voxel_block_ray_iterator_impl.hpp"



#endif // SE_VOXEL_BLOCK_RAY_ITERATOR_HPP

