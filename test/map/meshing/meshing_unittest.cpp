/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <random>

#include "se/map/map.hpp"

TEST(MeshingTest, EqualScaleNeighbour) {

  typedef se::TSDFData DataType;
  const int block_size = 8;
  const int blocks_per_side = 2;
  const int octree_size = blocks_per_side * block_size;
  typedef se::Octree<DataType, se::Res::Single, block_size> OctreeType;
  typename OctreeType::Ptr octree_ptr = std::shared_ptr<OctreeType >(new OctreeType(octree_size));;
  std::vector<se::key_t> allocation_list;
  allocation_list.reserve(blocks_per_side * blocks_per_side);
  std::vector<Eigen::Vector3i> block_coords;
  for (unsigned y = 0; y < octree_size; y += block_size) {
    for (unsigned z = 0; z < octree_size; z += block_size) {
      Eigen::Vector3i block_coord = Eigen::Vector3i(block_size, y, z);
      se::key_t key;
      se::keyops::encode_key(block_coord, octree_ptr->max_block_scale, key);
      allocation_list.push_back(key);
      block_coords.push_back(block_coord);
    }
  }

  std::sort(allocation_list.begin(), allocation_list.end());
  se::allocator::blocks(allocation_list, *octree_ptr, octree_ptr->getRoot());

  for (auto block_ptr_itr = se::BlocksIterator<OctreeType>(octree_ptr.get());
       block_ptr_itr != se::BlocksIterator<OctreeType>(); ++block_ptr_itr)
  {
    auto block_ptr = static_cast<typename OctreeType::BlockType*>(*block_ptr_itr);
    Eigen::Vector3i block_coord = block_ptr->getCoord();
    for (unsigned x = 0; x < block_size; x++)
    {
      for (unsigned y = 0; y < block_size; y++)
      {
        for (unsigned z = 0; z < block_size; z++)
        {
          Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
          typename OctreeType::DataType data;
          if (x < block_size / 2)
          {
            data.tsdf   = 1;
            data.weight = 1;
            block_ptr->setData(voxel_coord, data);
          } else
          {
            data.tsdf   = -1;
            data.weight = 1;
            block_ptr->setData(voxel_coord, data);
          }
        }
      }
    }
  }

  std::string filename = "/PATH/TO/out/multires-mesh-equal-neighbour-single.vtk";
  std::cout << "Saving triangle mesh to file :" << filename  << std::endl;

  se::TriangleMesh mesh;
  se::algorithms::dual_marching_cube(*octree_ptr, mesh);
  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
  se::io::save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));

}

TEST(MeshingTest, EqualScaleNeighbour2) {

  typedef se::TSDFData DataType;
  const int block_size = 8;
  const int blocks_per_side = 2;
  const int octree_size = blocks_per_side * block_size;
  typedef se::Octree<DataType, se::Res::Multi, block_size> OctreeType;
  typename OctreeType::Ptr octree_ptr = std::shared_ptr<OctreeType >(new OctreeType(octree_size));;
  std::vector<se::key_t> allocation_list;
  allocation_list.reserve(blocks_per_side * blocks_per_side);
  std::vector<Eigen::Vector3i> block_coords;

  for (unsigned y = 0; y < octree_size; y += block_size)
  {
    for (unsigned z = 0; z < octree_size; z += block_size)
    {
      Eigen::Vector3i block_coord = Eigen::Vector3i(block_size, y, z);
      se::key_t key;
      se::keyops::encode_key(block_coord, octree_ptr->max_block_scale, key);
      allocation_list.push_back(key);
      block_coords.push_back(block_coord);
    }
  }

  std::sort(allocation_list.begin(), allocation_list.end());
  se::allocator::blocks(allocation_list, *octree_ptr, octree_ptr->getRoot());

  const int curr_scale  = 1;
  const int curr_stride = 1 << curr_scale;

  for (auto block_ptr_itr = se::BlocksIterator<OctreeType>(octree_ptr.get());
       block_ptr_itr != se::BlocksIterator<OctreeType>(); ++block_ptr_itr)
  {
    auto block_ptr = static_cast<typename OctreeType::BlockType*>(*block_ptr_itr);
    Eigen::Vector3i block_coord = block_ptr->getCoord();

    block_ptr->setCurrentScale(curr_scale);
    for (unsigned x = 0; x < block_size; x += curr_stride)
    {
      for (unsigned y = 0; y < block_size; y += curr_stride)
      {
        for (unsigned z = 0; z < block_size; z += curr_stride)
        {
          Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
          typename OctreeType::DataType data;
          if (x < block_size / 2)
          {
            data.tsdf   = 1;
            data.weight = 1;
            block_ptr->setData(voxel_coord, data);
          } else
          {
            data.tsdf   = -1;
            data.weight = 1;
            block_ptr->setData(voxel_coord, data);
          }
        }
      }
    }
  }

  std::string filename = "PATH/TO/out/multires-mesh-equal-neighbour-multi.vtk";
  std::cout << "Saving triangle mesh to file :" << filename  << std::endl;

  se::TriangleMesh mesh;
  se::algorithms::dual_marching_cube(*octree_ptr, mesh);
  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
  se::io::save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));

}




//
//TEST(MeshingTest, CoarserScaleNeighbour) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[27];
//  int list_idx = 0;
//  std::vector<Eigen::Vector3i> block_coords;
//  for (unsigned x = 0; x <= 16; x += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y <= 16; y += VoxelBlockType::size_li) {
//      for (unsigned z = 0; z <= 16; z += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        block_coords.push_back(Eigen::Vector3i(x, y, z));
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 27);
//
//  Eigen::Vector3i block_coord_c = Eigen::Vector3i(8, 8, 8);
//  int block_scale_c = 0;
//  int block_stride_c = 1 << block_scale_c;
//  int block_scale_n = 1;
//
//  for (auto block_coord : block_coords) {
//    VoxelBlockType* block = octree.fetch(block_coord.x(), block_coord.y(), block_coord.z());
//    block->current_scale(block_scale_n);
//    block->min_scale(block_scale_n);
//  }
//
//  VoxelBlockType* block_c = octree.fetch(block_coord_c.x(), block_coord_c.y(), block_coord_c.z());
//  block_c->current_scale(block_scale_c);
//  block_c->min_scale(block_scale_c);
//  for (unsigned x = 0; x < VoxelBlockType::size_li - 1; x += block_stride_c) {
//    for (unsigned y = 0; y < VoxelBlockType::size_li - 1; y += block_stride_c) {
//      for (unsigned z = 0; z < VoxelBlockType::size_li - 1; z += block_stride_c) {
//        Eigen::Vector3i voxel_coord = block_coord_c + Eigen::Vector3i(x, y, z);
//        block_c->setData(voxel_coord, block_scale_c, {1.f, 1.f});
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-coarser-neighbour-unittest.vtk";
//  std::cout << "Saving triangle mesh to file :" << filename  << std::endl;
//
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, FinerScaleNeighbour) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[27];
//  int list_idx = 0;
//  std::vector<Eigen::Vector3i> block_coords;
//  for (unsigned x = 0; x <= 16; x += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y <= 16; y += VoxelBlockType::size_li) {
//      for (unsigned z = 0; z <= 16; z += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        block_coords.push_back(Eigen::Vector3i(x, y, z));
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 27);
//
//  Eigen::Vector3i block_coord_c = Eigen::Vector3i(8, 8, 8);
//  int block_scale_c = 1;
//  int block_stride_c = 1 << block_scale_c;
//  int block_scale_n = 0;
//
//  for (auto block_coord : block_coords) {
//    VoxelBlockType* block = octree.fetch(block_coord.x(), block_coord.y(), block_coord.z());
//    block->current_scale(block_scale_n);
//    block->min_scale(block_scale_n);
//  }
//
//  VoxelBlockType* block_c = octree.fetch(block_coord_c.x(), block_coord_c.y(), block_coord_c.z());
//  block_c->current_scale(block_scale_c);
//  block_c->min_scale(block_scale_c);
//  for (unsigned x = 0; x < VoxelBlockType::size_li - 1; x += block_stride_c) {
//    for (unsigned y = 0; y < VoxelBlockType::size_li - 1; y += block_stride_c) {
//      for (unsigned z = 0; z < VoxelBlockType::size_li - 1; z += block_stride_c) {
//        Eigen::Vector3i voxel_coord = block_coord_c + Eigen::Vector3i(x, y, z);
//        block_c->setData(voxel_coord, block_scale_c, {1.f, 1.f});
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-finer-neighbour-unittest.vtk";
//  std::cout << "Saving triangle mesh to file :" << filename  << std::endl;
//
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, Wall) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  std::vector<Eigen::Vector3i> block_coords_1;
//  std::vector<Eigen::Vector3i> block_coords_2;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      allocation_list[list_idx] = octree.hash(8, y, z);
//      block_coords_1.push_back(Eigen::Vector3i(8, y, z));
//      list_idx++;
//      allocation_list[list_idx] = octree.hash(16, y, z);
//      block_coords_2.push_back(Eigen::Vector3i(16, y, z));
//      list_idx++;
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//  int   block_scale_1  = 0;
//  int   block_stride_1 = 1 << block_scale_1;
//  float block_value_1  = 0.5f;
//
//  int   block_scale_2  = 0;
//  int   block_stride_2 = 1 << block_scale_1;
//  float block_value_2  = -0.5f;
//
//  for (auto block_coord : block_coords_1) {
//    VoxelBlockType* block = octree.fetch(block_coord.x(), block_coord.y(), block_coord.z());
//    block->current_scale(block_scale_1);
//    block->min_scale(block_scale_1);
//    for (unsigned x = 0; x < VoxelBlockType::size_li; x += block_stride_1) {
//      for (unsigned y = 0; y < VoxelBlockType::size_li; y += block_stride_1) {
//        for (unsigned z = 0; z < VoxelBlockType::size_li; z += block_stride_1) {
//          Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x, y, z);
//          block->setData(voxel_coord, block_scale_1, {block_value_1, 1.f});
//        }
//      }
//    }
//  }
//
//  for (auto block_coord : block_coords_2) {
//    VoxelBlockType* block = octree.fetch(block_coord.x(), block_coord.y(), block_coord.z());
//    block->current_scale(block_scale_2);
//    block->min_scale(block_scale_2);
//    for (unsigned x = 0; x < VoxelBlockType::size_li; x += block_stride_2) {
//      for (unsigned y = 0; y < VoxelBlockType::size_li; y += block_stride_2) {
//        for (unsigned z = 0; z < VoxelBlockType::size_li; z += block_stride_2) {
//          Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x, y, z);
//          block->setData(voxel_coord, block_scale_2, {block_value_2, 1.f});
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall1-unittest.vtk";
//  std::cout << "Saving triangle mesh to file :" << filename  << std::endl;
//
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesXFineToCoarseAlongY) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-y-scale-0-1-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-y-scale-0-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-y-scale-0-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-y-scale-1-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-y-scale-1-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-y-scale-2-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesXFineToCoarseAlongZ) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-z-scale-0-1-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-z-scale-0-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-z-scale-0-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-z-scale-1-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-z-scale-1-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-fine-to-coarse-along-z-scale-2-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesXCoarseToFineAlongY) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-y-scale-1-0-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-y-scale-2-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-y-scale-3-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-y-scale-2-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-y-scale-3-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-y-scale-3-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesXCoarseToFineAlongZ) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-z-scale-1-0-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-z-scale-2-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-z-scale-3-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-z-scale-2-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-z-scale-3-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 8; x < 24; x += VoxelBlockType::size_li) {
//        if (z >= 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (x >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-x-axis-coarse-to-fine-along-z-scale-3-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesYFineToCoarseAlongX) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-x-scale-0-1-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-x-scale-0-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-x-scale-0-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-x-scale-1-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-x-scale-1-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-x-scale-2-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesYFineToCoarseAlongZ) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-z-scale-0-1-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-z-scale-0-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-z-scale-0-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-z-scale-1-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-z-scale-1-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (z < 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-fine-to-coarse-along-z-scale-2-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesYCoarseToFineAlongX) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-x-scale-1-0-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-x-scale-2-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-x-scale-3-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-x-scale-2-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-x-scale-3-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-x-scale-3-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesYCoarseToFineAlongZ) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-z-scale-1-0-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-z-scale-2-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-z-scale-3-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-z-scale-2-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-z-scale-3-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 0; z < 32; z += VoxelBlockType::size_li) {
//    for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//      for (unsigned y = 8; y < 24; y += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (y >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-y-axis-coarse-to-fine-along-z-scale-3-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesZFineToCoarseAlongX) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-x-scale-0-1-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-x-scale-0-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-x-scale-0-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-x-scale-1-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-x-scale-1-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x < 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-x-scale-2-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesZFineToCoarseAlongY) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-y-scale-0-1-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-y-scale-0-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-y-scale-0-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-y-scale-1-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-y-scale-1-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y < 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z < 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-fine-to-coarse-along-y-scale-2-3-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesZCoarseToFineAlongX) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-x-scale-1-0-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-x-scale-2-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-x-scale-3-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-x-scale-2-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-x-scale-3-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (x >= 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-x-scale-3-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
//
//TEST(MeshingTest, WallCrossesZCoarseToFineAlongY) {
//  typedef se::Octree<TestVoxelT> OctreeF;
//  OctreeF octree;
//  octree.init(32, 32);
//  se::key_t allocation_list[32];
//  int list_idx = 0;
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        allocation_list[list_idx] = octree.hash(x, y, z);
//        list_idx++;
//      }
//    }
//  }
//  octree.allocate(allocation_list, 32);
//
//
//  int   block_scale_0     = 0;
//  int   block_stride_0    = 1 << block_scale_0;
//  float block_value_0_out = 0.5f;
//  float block_value_0_in  = -0.5f;
//
//  int   block_scale_1     = 1;
//  int   block_stride_1    = 1 << block_scale_0;
//  float block_value_1_out = 1.f;
//  float block_value_1_in  = -1.f;
//
//  int   block_scale_2     = 2;
//  int   block_stride_2    = 1 << block_scale_0;
//  float block_value_2_out = 2.f;
//  float block_value_2_in  = -2.f;
//
//  int   block_scale_3     = 3;
//  int   block_stride_3    = 1 << block_scale_0;
//  float block_value_3_out = 4.f;
//  float block_value_3_in  = -4.f;
//
//  Eigen::Matrix4f T_MW = Eigen::Matrix4f::Identity();
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  std::string filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-y-scale-1-0-unittest.vtk";
//  se::TriangleMesh mesh;
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-y-scale-2-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 0
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_0);
//          block->min_scale(block_scale_0);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_0) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_0) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_0) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_0, {block_value_0_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-y-scale-3-0-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-y-scale-2-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 1
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_1);
//          block->min_scale(block_scale_1);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_1) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_1) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_1) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_1, {block_value_1_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-y-scale-3-1-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//
//  for (unsigned z = 8; z < 24; z += VoxelBlockType::size_li) {
//    for (unsigned y = 0; y < 32; y += VoxelBlockType::size_li) {
//      for (unsigned x = 0; x < 32; x += VoxelBlockType::size_li) {
//        if (y >= 16) { // Scale 2
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_2);
//          block->min_scale(block_scale_2);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_2) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_2) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_2) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_2, {block_value_2_out, 1.f});
//                }
//              }
//            }
//          }
//        } else {      // Scale 3
//          VoxelBlockType *block = octree.fetch(x, y, z);
//          block->current_scale(block_scale_3);
//          block->min_scale(block_scale_3);
//          for (unsigned z_rel = 0; z_rel < VoxelBlockType::size_li; z_rel += block_stride_3) {
//            for (unsigned y_rel = 0; y_rel < VoxelBlockType::size_li; y_rel += block_stride_3) {
//              for (unsigned x_rel = 0; x_rel < VoxelBlockType::size_li; x_rel += block_stride_3) {
//                if (z >= 16) { // Inside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_in, 1.f});
//                } else {      // Outside
//                  Eigen::Vector3i voxel_coord = block->coordinates() + Eigen::Vector3i(x_rel, y_rel, z_rel);
//                  block->setData(voxel_coord, block_scale_3, {block_value_3_out, 1.f});
//                }
//              }
//            }
//          }
//        }
//      }
//    }
//  }
//
//  filename = "../../out/multires-mesh-wall-crosses-z-axis-coarse-to-fine-along-y-scale-3-2-unittest.vtk";
//  mesh.clear();
//  se::algorithms::dual_marching_cube(octree, TestVoxelT::selectValue, TestVoxelT::isInside, mesh);
//  save_mesh_vtk(mesh, filename.c_str(), se::math::to_inverse_transformation(T_MW));
//}
