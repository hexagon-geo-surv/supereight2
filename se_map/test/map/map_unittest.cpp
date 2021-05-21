/*

Copyright 2021 Nils Funk, Imperial College London

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

#include <gtest/gtest.h>

#include "se/utils/type_util.hpp"
#include "se/utils/key_util.hpp"
#include "se/utils/octant_util.hpp"
#include "se/utils/math_util.hpp"
#include "se/octree/octree.hpp"
#include "se/octree/allocator.hpp"
#include "se/octree/visitor.hpp"
#include "se/data.hpp"
#include "se/map.hpp"

// Helper function to create ought values.
Eigen::Vector3i adapt_to_scale(const Eigen::Vector3i& coord, const se::scale_t scale) {
  Eigen::Vector3i adapted_coord;
  adapted_coord.x() = (coord.x() >> scale) << scale;
  adapted_coord.y() = (coord.y() >> scale) << scale;
  adapted_coord.z() = (coord.z() >> scale) << scale;
  return adapted_coord;
}

TEST(Map, Interpolation)
{
  const Eigen::Vector3f map_dim(32.f, 32.f, 32.f);
  const float map_res(1.f);
  se::TSDFMap<se::Res::Single> map_tsdf(map_dim, map_res);

  Eigen::Vector3i block_coord;
  Eigen::Vector3i coord_ought;
  Eigen::Vector3i coord_is;

  typedef typename se::TSDFMap<se::Res::Single>::DataType DataType;
  typedef typename se::TSDFMap<se::Res::Single>::OctreeType OctreeType;
  typedef typename se::TSDFMap<se::Res::Single>::OctreeType::BlockType BlockType;

  unsigned int octree_size = 32;

  int block_size = BlockType::size;

  std::vector<Eigen::Vector3i> block_coords =
  {
    Eigen::Vector3i(         0,          0,          0),
    Eigen::Vector3i(block_size,          0,          0),
    Eigen::Vector3i(         0, block_size,          0),
    Eigen::Vector3i(block_size, block_size,          0),
    Eigen::Vector3i(         0,          0, block_size),
    Eigen::Vector3i(block_size,          0, block_size),
    Eigen::Vector3i(         0, block_size, block_size),
    Eigen::Vector3i(block_size, block_size, block_size)
  };

  std::shared_ptr<OctreeType> octree_ptr = std::shared_ptr<OctreeType>(new OctreeType(octree_size));

  BlockType* block_ptr = nullptr;

  for (size_t i = 0; i < block_coords.size(); i++)
  {
    const Eigen::Vector3i block_coord = block_coords[i];
    coord_ought = adapt_to_scale(block_coord, octree_ptr->max_block_scale);
    se::key_t voxel_key;
    se::keyops::encode_key(block_coord, 0, voxel_key);
    block_ptr = static_cast<BlockType*>(se::allocator::block(voxel_key, *octree_ptr, octree_ptr->getRoot()));
    coord_is = block_ptr->getCoord();
    EXPECT_EQ(coord_ought, coord_is);
    for (size_t voxel_idx = 0; voxel_idx < block_ptr->size_cu; voxel_idx++)
    {
      DataType data;
      data.tsdf = i;
      data.weight = 1;
      block_ptr->setData(voxel_idx, data);
    }
  }

  map_tsdf.setOctree(octree_ptr);
  
  auto interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -12, -12));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(0, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -12, -12));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(0.5, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -8, -12));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(1, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -8, -12));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(1.5, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -12, -8));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(2, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -12, -8));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(2.5, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-12, -8, -8));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(3, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(-8, -8, -8));
  EXPECT_TRUE(interp_field_value);
  EXPECT_EQ(3.5, *interp_field_value);

  interp_field_value = map_tsdf.getFieldInterp(Eigen::Vector3f(+2, +2, +2));
  EXPECT_FALSE(interp_field_value);
}
