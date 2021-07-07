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


#ifndef SE_MARCHING_CUBE_HPP
#define SE_MARCHING_CUBE_HPP

#include "edge_tables.hpp"
#include "se/common/timings.hpp"
#include "se/map/octree/fetcher.hpp"
#include "se/map/octree/iterator.hpp"
#include "se/map/octree/visitor.hpp"



namespace se {



typedef struct Triangle {
    Eigen::Vector3f vertexes[3];
    Eigen::Vector3f vnormals[3];
    Eigen::Vector3f normal;
    float color;
    float surface_area;
    int8_t max_vertex_scale;

    Triangle(){
      vertexes[0] = Eigen::Vector3f::Constant(0);
      vertexes[1] = Eigen::Vector3f::Constant(0);
      vertexes[2] = Eigen::Vector3f::Constant(0);
      normal = Eigen::Vector3f::Constant(0);
      surface_area = -1.f;
      max_vertex_scale = 0;
    }

    inline bool iszero(const Eigen::Vector3f& v)
    {
      return !(v.array() == 0).all();
    }

    inline bool valid()
    {
      return !(iszero(vertexes[0]) && iszero(vertexes[1]) && iszero(vertexes[2]));
    }

    inline void compute_normal()
    {
      normal = (vertexes[1] - vertexes[0]).cross(vertexes[2] - vertexes[1]);
    }

    inline void compute_boundingbox(Eigen::Vector3f& minV, Eigen::Vector3f& maxV) const
    {
      minV = vertexes[0];
      maxV = vertexes[0];
      minV = minV.cwiseMin(vertexes[0]);
      minV = minV.cwiseMin(vertexes[1]);
      minV = minV.cwiseMin(vertexes[2]);
      maxV = maxV.cwiseMax(vertexes[0]);
      maxV = maxV.cwiseMax(vertexes[1]);
      maxV = maxV.cwiseMax(vertexes[2]);
    }

    inline float area()
    {
      // Use the cached value if available
      if(surface_area > 0) return surface_area;
      Eigen::Vector3f a = vertexes[1] - vertexes[0];
      Eigen::Vector3f b = vertexes[2] - vertexes[1];
      Eigen::Vector3f v = a.cross(b);
      surface_area = v.norm() / 2;
      return surface_area;
    }

    Eigen::Vector3f* uniform_sample(int num)
    {
      Eigen::Vector3f* points = new Eigen::Vector3f[num];
      for(int i = 0; i < num; ++i){
        float u = ((float)rand()) / (float)RAND_MAX;
        float v = ((float)rand()) / (float)RAND_MAX;
        if(u + v > 1)
        {
          u = 1 - u;
          v = 1 - v;
        }
        float w = 1 - (u + v);
        points[i] = u * vertexes[0] + v * vertexes[1] + w * vertexes[2];
      }

      return points;
    }

    Eigen::Vector3f * uniform_sample(int num, unsigned int& seed) const
    {
      Eigen::Vector3f * points = new Eigen::Vector3f[num];
      for(int i = 0; i < num; ++i){
        float u = ((float)rand_r(&seed)) / (float)RAND_MAX;
        float v = ((float)rand_r(&seed)) / (float)RAND_MAX;
        if(u + v > 1)
        {
          u = 1 - u;
          v = 1 - v;
        }
        float w = 1 - (u + v);
        points[i] = u * vertexes[0] + v * vertexes[1] + w * vertexes[2];
      }
      return points;
    }

} Triangle;



namespace meshing {

enum status : uint8_t {
    OUTSIDE = 0x0,
    UNKNOWN = 0xFE, // 254
    INSIDE = 0xFF, // 255
};

/// Single-res marching cube implementation

template <typename OctreeT>
inline Eigen::Vector3f compute_intersection(const OctreeT&         octree,
                                            const Eigen::Vector3i& source_coord,
                                            const Eigen::Vector3i& dest_coord);

template <typename OctreeT>
inline Eigen::Vector3f interp_vertexes(const OctreeT& octree,
                                       const unsigned x,
                                       const unsigned y,
                                       const unsigned z,
                                       const int      edge);

template <typename BlockT>
inline void gather_data(const BlockT*             block,
                        typename BlockT::DataType data[8],
                        const int                 x,
                        const int                 y,
                        const int                 z);

template <typename OctreeT>
inline void gather_data(const OctreeT&             octree,
                        typename OctreeT::DataType data[8],
                        const int                  x,
                        const int                  y,
                        const int                  z);

template <typename OctreeT>
uint8_t compute_index(const OctreeT&                     octree,
                      const typename OctreeT::BlockType* block_ptr,
                      const unsigned                     x,
                      const unsigned                     y,
                      const unsigned                     z);



/// Multires-res marching cube implementation

inline Eigen::Vector3f compute_dual_intersection(const float            value_0,
                                                 const float            value_1,
                                                 const Eigen::Vector3f& dual_corner_coord_0,
                                                 const Eigen::Vector3f& dual_corner_coord_1,
                                                 const float            voxel_dim,
                                                 const int              /* edge_case */);

template <typename DataT,
        typename ValueSelector>
inline Eigen::Vector3f interp_dual_vertexes(const int                                   edge,
                                            const DataT                                 data[8],
                                            const std::vector<Eigen::Vector3f,
                                                    Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f,
                                            const float                                 voxel_dim,
                                            ValueSelector                               select_value);

/*
 * Normalised offsets of the dual corners from the primal corner
 */
static const Eigen::Vector3f norm_dual_offset_f[8] =
        {{-1, -1, -1}, {+1, -1, -1}, {+1, -1, +1}, {-1, -1, +1},
         {-1, +1, -1}, {+1, +1, -1}, {+1, +1, +1}, {-1, +1, +1}};

template <typename BlockT,
          typename DataT
>
inline void gather_dual_data(const BlockT*               block,
                             const int                   scale,
                             const Eigen::Vector3f&      primal_corner_coord_f,
                             DataT                       data[8],
                             std::vector<Eigen::Vector3f,
                             Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f);

/*! \brief The following strategy is derived from I. Wald, A Simple, General,
 *  and GPU Friendly Method for Computing Dual Mesh and Iso-Surfaces of Adaptive Mesh Refinement (AMR) Data, 2020
 *
 * We validate the scale of all neighbouring blocks need to access the 8 dual corners for each primal corner
 * For each we compute the dual coordinates for primal coordinates with a relative block offset x,y,z in [0, block_size]
 * Due to the fact that block_size is still contained in the offset (rather than [0, block_size - 1], each primal corner
 * is contained in 1 (inside block), 2 (face), 4 (edge) or 8 (corner) neighbouring blocks. We prioritse the block neighbours
 * based on their value (heigher value = higher priority), where the value is calculated via
 * v = 4 << (x is +1) + 2 << (y is +1) + 1 << (z is +1).
 * The minium value is computed for all neighbours for dual corners (c0-8). This means if multiple dual corners fall
 * inside the same neighbouring block we take the minimum value of all the dual corners inside the block.
 * The threshold for populate the lower or higher priority list is the lowest dual corner cost of the corners falling
 * inside the block the relative primal coordinate belongs to.
 *
 * \param[in] primal_corner_coord_rel     relative voxel offset of the primal corner from the block coordinates
 * \param[in] block_size                  size of a voxel block in voxel units
 * \param[in] lower_priority_neighbours   blocks with lower priority, i.e. will be neglected if scale >= scale neighbour
 * \param[in] higher_priority_neighbours  blocks with higher priority, i.e. will only be neglected if scale > scale neighbour
 * \param[in] neighbours                  vector containing a vector with all corner offsets for a neighbouring block.
 *                                        First index is the main block.
 */
inline void norm_dual_corner_idxs(const Eigen::Vector3i&         primal_corner_coord_rel,
                                  const int                      block_size,
                                  std::vector<int>&              lower_priority_neighbours,
                                  std::vector<int>&              higher_priority_neighbours,
                                  std::vector<std::vector<int>>& neighbours);

/*
 * Normalised offsets of the primal corners from the primal corner
 * The correct coordinates would be {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, ...
 * However this would cause a lot of extra computations to cast the voxels back and forth and
 * to make sure that the absolute coordinates {-0.5, y, z} won't be casted to {0, y, z} (and for y, z accordingly).
 */
static const Eigen::Vector3i logical_dual_offset[8] =
        {{-1, -1, -1}, {+0, -1, -1}, {+0, -1, +0}, {-1, -1, +0},
         {-1, +0, -1}, {+0, +0, -1}, {+0, +0, +0}, {-1, +0, +0}};

template <typename  OctreeT,
          typename DataT
>
inline void gather_dual_data(const OctreeT&                     octree,
                             const typename OctreeT::BlockType* block,
                             const int                          scale,
                             const Eigen::Vector3i&             primal_corner_coord,
                             DataT                              data[8],
                             std::vector<Eigen::Vector3f,
                                Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f);

template <typename OctreeT,
          typename DataT
>
void compute_dual_index(const OctreeT&                     octree,
                        const typename OctreeT::BlockType* block_ptr,
                        const int                          scale,
                        const Eigen::Vector3i&             primal_corner_coord,
                        uint8_t&                           edge_pattern_idx,
                        DataT                              data[8],
                        std::vector<Eigen::Vector3f,
                            Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f);

inline bool checkVertex(const Eigen::Vector3f& vertex_M, const float dim);

} // namespace meshing



namespace algorithms {

template <typename OctreeT,
          typename TriangleT
>
void marching_cube(OctreeT&                octree,
                   std::vector<TriangleT>& triangles);

template <typename OctreeT,
          typename TriangleT
>
void marching_cube(OctreeT&                octree,
                   std::vector<TriangleT>& triangles,
                   const int               time_stamp);

template <typename OctreeT,
          typename TriangleT
>
void dual_marching_cube(OctreeT&                octree,
                        std::vector<TriangleT>& triangles);

template <typename OctreeT,
          typename TriangleT
>
void dual_marching_cube(OctreeT&                octree,
                        std::vector<TriangleT>& triangles,
                        const int               frame);

} // namespace algorithms
} // namespace se

#include "impl/marching_cube_impl.hpp"

#endif // SE_MARCHING_CUBE_HPP

