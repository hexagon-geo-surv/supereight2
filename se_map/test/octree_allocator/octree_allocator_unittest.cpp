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

#include <algorithm>

#include "se/utils/type_util.hpp"
#include "se/utils/key_util.hpp"
#include "se/utils/octant_util.hpp"
#include "se/utils/math_util.hpp"
#include "se/octree/octree.hpp"
#include "se/octree/allocator.hpp"
#include "se/data.hpp"

// Helper function to create ought values.
Eigen::Vector3i adapt_to_scale(const Eigen::Vector3i& coord, const se::scale_t scale) {
  Eigen::Vector3i adapted_coord;
  adapted_coord.x() = (coord.x() >> scale) << scale;
  adapted_coord.y() = (coord.y() >> scale) << scale;
  adapted_coord.z() = (coord.z() >> scale) << scale;
  return adapted_coord;
}

bool vector3i_less(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs)
{
  if (lhs.x() < rhs.x()) {
    return true;
  } else if (lhs.x() == rhs.x() && lhs.y() < rhs.y()) {
    return true;
  } else if (lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() < rhs.z()) {
    return true;
  } else {
    return false;
  }
}



TEST(SingleResAllocation, BlockKey)
{
  Eigen::Vector3i voxel_coord;
  Eigen::Vector3i coord_ought;
  Eigen::Vector3i coord_is;

  typedef se::TSDFData DataType;
  typedef se::Node<DataType, se::Res::Single> NodeType;

  const NodeType* node_ptr = nullptr;
  se::OctantBase* octant_ptr = nullptr;

  std::vector<Eigen::Vector3i> voxel_coords =
  {
    Eigen::Vector3i(253,  74, 135),
    Eigen::Vector3i(114, 244,  65),
    Eigen::Vector3i( 38, 104,  85)
  };

  se::scale_t max_tree_scale = 8;
  unsigned int octree_size = 1 << max_tree_scale;

  // SCALE 0
  constexpr se::scale_t max_block_scale_0 = 0;
  constexpr size_t      block_size_0      = 1 << max_block_scale_0;

  typedef se::Octree<DataType, se::Res::Single, block_size_0> OctreeType0;
  typedef OctreeType0::BlockType BlockType0;

  OctreeType0::Ptr octree_ptr_0 = OctreeType0::Ptr(new OctreeType0(octree_size));

  BlockType0* block_ptr_0 = nullptr;

  for (const auto voxel_coord : voxel_coords)
  {
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr_0->max_block_scale);
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key);
    block_ptr_0 = static_cast<BlockType0*>(se::allocator::block(voxel_key, *octree_ptr_0, octree_ptr_0->getRoot()));
    coord_is = block_ptr_0->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr_0->getParent();
    for (se::scale_t s = max_block_scale_0 + 1; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }


  // SCALE 1
  constexpr se::scale_t max_block_scale_1 = 1;
  constexpr size_t      block_size_1      = 1 << max_block_scale_1;

  typedef se::Octree<DataType, se::Res::Single, block_size_1> OctreeType1;
  typedef OctreeType1::BlockType BlockType1;

  OctreeType1::Ptr octree_ptr_1 = OctreeType1::Ptr(new OctreeType1(octree_size));

  BlockType1* block_ptr_1 = nullptr;

  for (const auto voxel_coord : voxel_coords)
  {
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr_1->max_block_scale);
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key);
    block_ptr_1 = static_cast<BlockType1*>(se::allocator::block(voxel_key, *octree_ptr_1, octree_ptr_1->getRoot()));
    coord_is = block_ptr_1->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr_1->getParent();
    for (se::scale_t s = max_block_scale_1 + 1; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }


  // SCALE 2
  constexpr se::scale_t max_block_scale_2 = 2;
  constexpr size_t      block_size_2      = 1 << max_block_scale_2;

  typedef se::Octree<DataType, se::Res::Single, block_size_2> OctreeType2;
  typedef OctreeType2::BlockType BlockType2;

  OctreeType2::Ptr octree_ptr_2 = OctreeType2::Ptr(new OctreeType2(octree_size));

  BlockType2* block_ptr_2 = nullptr;

  for (const auto voxel_coord : voxel_coords)
  {
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr_2->max_block_scale);
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key);
    block_ptr_2 = static_cast<BlockType2*>(se::allocator::block(voxel_key, *octree_ptr_2, octree_ptr_2->getRoot()));
    coord_is = block_ptr_2->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr_2->getParent();
    for (se::scale_t s = max_block_scale_2 + 1; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }


  // SCALE 3
  constexpr se::scale_t max_block_scale_3 = 3;
  constexpr size_t      block_size_3      = 1 << max_block_scale_3;

  typedef se::Octree<DataType, se::Res::Single, block_size_3> OctreeType3;
  typedef OctreeType3::BlockType BlockType3;

  OctreeType3::Ptr octree_ptr_3 = OctreeType3::Ptr(new OctreeType3(octree_size));

  BlockType3* block_ptr_3 = nullptr;

  for (const auto voxel_coord : voxel_coords)
  {
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr_3->max_block_scale);
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key);
    block_ptr_3 = static_cast<BlockType3*>(se::allocator::block(voxel_key, *octree_ptr_3, octree_ptr_3->getRoot()));
    coord_is = block_ptr_3->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr_3->getParent();
    for (se::scale_t s = max_block_scale_3 + 1; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }


  // SCALE 4
  constexpr se::scale_t max_block_scale_4 = 4;
  constexpr size_t      block_size_4      = 1 << max_block_scale_4;

  typedef se::Octree<DataType, se::Res::Single, block_size_4> OctreeType4;
  typedef OctreeType4::BlockType BlockType4;

  OctreeType4::Ptr octree_ptr_4 = OctreeType4::Ptr(new OctreeType4(octree_size));

  BlockType4* block_ptr_4 = nullptr;

  for (const auto voxel_coord : voxel_coords)
  {
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr_4->max_block_scale);
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key);
    block_ptr_4 = static_cast<BlockType4*>(se::allocator::block(voxel_key, *octree_ptr_4, octree_ptr_4->getRoot()));
    coord_is = block_ptr_4->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr_4->getParent();
    for (se::scale_t s = max_block_scale_4 + 1; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }
}



TEST(SingleResAllocation, BlockCoord)
{
  se::scale_t max_tree_scale = 8;
  unsigned int octree_size = 1 << max_tree_scale;

  typedef se::Octree<se::TSDFData, se::Res::Single> OctreeType;
  typedef OctreeType::NodeType NodeType;
  typedef OctreeType::BlockType BlockType;

  OctreeType::Ptr octree_ptr = OctreeType::Ptr(new OctreeType(octree_size));

  Eigen::Vector3i voxel_coord;
  Eigen::Vector3i coord_ought;
  Eigen::Vector3i coord_is;

  BlockType* block_ptr = nullptr;
  const NodeType*  node_ptr = nullptr;
  se::OctantBase* octant_ptr = nullptr;

  std::vector<Eigen::Vector3i> voxel_coords =
  {
    Eigen::Vector3i(233,  44, 255),
    Eigen::Vector3i(113, 144, 155),
    Eigen::Vector3i( 33, 104,  55)
  };

  for (const auto voxel_coord : voxel_coords)
  {
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr->max_block_scale);
    block_ptr = static_cast<BlockType*>(se::allocator::block(voxel_coord, *octree_ptr, octree_ptr->getRoot()));
    coord_is = block_ptr->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr->getParent();
    for (se::scale_t s = 4; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }
}



TEST(SingleResAllocation, BlockKeys)
{
  typedef se::Octree<se::TSDFData, se::Res::Single> OctreeType;
  typedef OctreeType::NodeType NodeType;
  typedef OctreeType::BlockType BlockType;

  // Create an octree.
  constexpr se::scale_t max_tree_scale = 8;
  constexpr int octree_size = 1 << max_tree_scale;
  OctreeType::Ptr octree_ptr = OctreeType::Ptr(new OctreeType(octree_size));

  // The coordinates of some voxels.
  std::vector<Eigen::Vector3i> voxel_coords =
  {
    Eigen::Vector3i( 33, 104,  55),
    Eigen::Vector3i(113, 144, 155),
    Eigen::Vector3i(233,  44, 255)
  };
  // Sort the voxel coordinates to make later comparisons easier.
  std::sort(voxel_coords.begin(), voxel_coords.end(), vector3i_less);

  // Get the coordinates of the corresponding Blocks from the voxel coordinates.
  std::vector<Eigen::Vector3i> desired_block_coords (voxel_coords.size());
  std::transform(voxel_coords.begin(), voxel_coords.end(), desired_block_coords.begin(),
      [=](const Eigen::Vector3i& x) { return adapt_to_scale(x, octree_ptr->max_block_scale); });
  // Sort the Block coordinates to make later comparisons easier.
  std::sort(desired_block_coords.begin(), desired_block_coords.end(), vector3i_less);

  // Allocate the Blocks.
  std::vector<se::OctantBase*> block_ptrs = se::allocator::blocks(voxel_coords, *octree_ptr, octree_ptr->getRoot());

  // Sort the returned Blocks based on their coordinates.
  std::sort(block_ptrs.begin(), block_ptrs.end(),
      [](auto b1, auto b2){ return vector3i_less(b1->getCoord(), b2->getCoord()); });

  EXPECT_EQ(desired_block_coords.size(), block_ptrs.size());

  // Loop over all returned Blocks.
  for (se::idx_t i = 0; i < block_ptrs.size(); ++i)
  {
    auto block_ptr = static_cast<BlockType*>(block_ptrs[i]);
    const Eigen::Vector3i actual_block_coord = block_ptr->getCoord();
    EXPECT_EQ(desired_block_coords[i], actual_block_coord);

    const se::OctantBase* octant_ptr = block_ptr->getParent();
    for (se::scale_t s = 4; s <= 8; ++s)
    {
      const NodeType* node_ptr = static_cast<const NodeType*>(octant_ptr);
      const Eigen::Vector3i coord_is = node_ptr->getCoord();
      const Eigen::Vector3i coord_ought = adapt_to_scale(voxel_coords[i], s);
      EXPECT_EQ(coord_ought, coord_is);
      octant_ptr = octant_ptr->getParent();
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }
}



TEST(SingleResAllocation, BlockCoords)
{
  se::scale_t max_tree_scale = 8;
  unsigned int octree_size = 1 << max_tree_scale;

  typedef se::Octree<se::TSDFData, se::Res::Single> OctreeType;
  typedef OctreeType::NodeType NodeType;
  typedef OctreeType::BlockType BlockType;

  OctreeType::Ptr octree_ptr = OctreeType::Ptr(new OctreeType(octree_size));

  Eigen::Vector3i coord_ought;
  Eigen::Vector3i coord_is;

  const NodeType* node_ptr = nullptr;
  se::OctantBase* octant_ptr = nullptr;

  std::vector<Eigen::Vector3i> voxel_coords =
  {
    Eigen::Vector3i(233,  44, 255),
    Eigen::Vector3i(113, 144, 155),
    Eigen::Vector3i( 33, 104,  55)
  };

  std::vector<se::key_t> voxel_keys;
  for (const auto voxel_coord : voxel_coords)
  {
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key);
    voxel_keys.push_back(voxel_key);
  }

  std::vector<se::OctantBase*> block_ptrs = se::allocator::blocks(voxel_coords, *octree_ptr, octree_ptr->getRoot());

  se::keyops::sort_keys(voxel_keys);
  se::octantops::sort_blocks<BlockType>(block_ptrs);

  for (se::idx_t i = 0; i < block_ptrs.size(); ++i)
  {
    Eigen::Vector3i voxel_coord;
    se::scale_t     voxel_scale;
    se::keyops::decode_key(voxel_keys[i], voxel_coord, voxel_scale);
    coord_ought = adapt_to_scale(voxel_coord, octree_ptr->max_block_scale);
    auto block_ptr   = block_ptrs[i];
    coord_is = block_ptr->getCoord();
    EXPECT_EQ(coord_ought, coord_is);

    octant_ptr = block_ptr->getParent();
    for (se::scale_t s = 4; s <= 8; ++s)
    {
      node_ptr = static_cast<const NodeType*>(octant_ptr);
      coord_is    = node_ptr->getCoord();
      coord_ought = adapt_to_scale(voxel_coord, s);
      EXPECT_EQ(coord_ought, coord_is);
      se::OctantBase* octant_tmp_ptr = nullptr;
      octant_tmp_ptr = octant_ptr->getParent();;
      octant_ptr = octant_tmp_ptr;
    }
    EXPECT_EQ(nullptr, octant_ptr);
  }
}

