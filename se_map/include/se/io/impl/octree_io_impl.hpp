#ifndef SE_OCTREE_IO_IMPL_HPP
#define SE_OCTREE_IO_IMPL_HPP

#include <fstream>
#include <sstream>
#include <iostream>

#include "se/octree/visitor.hpp"



namespace se {
namespace io {

template <typename OctreeT>
int save_3d_slice_vtk(const OctreeT&         octree,
                      const std::string&     filename,
                      const Eigen::Vector3i& lower_coord,
                      const Eigen::Vector3i& upper_coord)
{

  auto get_value = [&](int x, int y, int z)
  {
    return se::get_field(se::visitor::getData(octree, Eigen::Vector3i(x, y, z)));
  };

  // Open the file for writing.
  std::ofstream file(filename.c_str());
  if (!file.is_open())
  {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  std::stringstream ss_x_coord, ss_y_coord, ss_z_coord, ss_scalars;

  const int stride = 1;
  const int dimX = std::max(1, (upper_coord.x() - lower_coord.x()) / stride);
  const int dimY = std::max(1, (upper_coord.y() - lower_coord.y()) / stride);
  const int dimZ = std::max(1, (upper_coord.z() - lower_coord.z()) / stride);

  file << "# vtk DataFile Version 1.0" << std::endl;
  file << "vtk mesh generated from KFusion" << std::endl;
  file << "ASCII" << std::endl;
  file << "DATASET RECTILINEAR_GRID" << std::endl;
  file << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

  for (int x = lower_coord.x(); x < upper_coord.x(); x += stride)
  {
    ss_x_coord << x << " ";
  }
  for (int y = lower_coord.y(); y < upper_coord.y(); y += stride)
  {
    ss_y_coord << y << " ";
  }
  for (int z = lower_coord.z(); z < upper_coord.z(); z += stride)
  {
    ss_z_coord << z << " ";
  }

  for (int z = lower_coord.z(); z < upper_coord.z(); z += stride)
  {
    for (int y = lower_coord.y(); y < upper_coord.y(); y += stride)
    {
      for (int x = lower_coord.x(); x < upper_coord.x(); x += stride)
      {
        const auto value = get_value(x, y, z);
        ss_scalars << value << std::endl;
      } // x
    } // y
  } // z

  file << "X_COORDINATES " << dimX << " int " << std::endl;
  file << ss_x_coord.str() << std::endl;

  file << "Y_COORDINATES " << dimY << " int " << std::endl;
  file << ss_y_coord.str() << std::endl;

  file << "Z_COORDINATES " << dimZ << " int " << std::endl;
  file << ss_z_coord.str() << std::endl;

  file << "POINT_DATA " << dimX * dimY * dimZ << std::endl;
  file << "SCALARS scalars float 1" << std::endl;
  file << "LOOKUP_TABLE default" << std::endl;
  file << ss_scalars.str() << std::endl;

  file.close();
  return 0;
}




template <typename OctreeT>
int save_octree_structure_ply(OctreeT&           octree,
                              const std::string& filename)
{

  // Open the file for writing.
  std::ofstream file (filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  std::stringstream ss_nodes_corners;
  std::stringstream ss_faces;
  int nodes_corners_count = 0;
  int faces_count  = 0;
  for (auto octant_ptr_itr = octree.begin(); octant_ptr_itr != octree.end(); ++octant_ptr_itr) {
    auto octant_ptr = *octant_ptr_itr;
    const Eigen::Vector3i node_coord = octant_ptr->getCoord();
    int node_size;
    if (octant_ptr->isBlock())
    {
      node_size = static_cast<typename OctreeT::BlockType*>(octant_ptr)->getSize();
    } else
    {
      node_size = static_cast<typename OctreeT::NodeType*>(octant_ptr)->getSize();
    }

    Eigen::Vector3f node_corners[8];
    node_corners[0] =  node_coord.cast<float>();
    node_corners[1] = (node_coord + Eigen::Vector3i(node_size, 0, 0)).cast<float>();
    node_corners[2] = (node_coord + Eigen::Vector3i(0, node_size, 0)).cast<float>();
    node_corners[3] = (node_coord + Eigen::Vector3i(node_size, node_size, 0)).cast<float>();
    node_corners[4] = (node_coord + Eigen::Vector3i(0, 0, node_size)).cast<float>();
    node_corners[5] = (node_coord + Eigen::Vector3i(node_size, 0, node_size)).cast<float>();
    node_corners[6] = (node_coord + Eigen::Vector3i(0, node_size, node_size)).cast<float>();
    node_corners[7] = (node_coord + Eigen::Vector3i(node_size, node_size, node_size)).cast<float>();

    for(int i = 0; i < 8; ++i) {
      ss_nodes_corners << node_corners[i].x() << " "
                       << node_corners[i].y() << " "
                       << node_corners[i].z() << std::endl;
    }

    ss_faces << "4 " << nodes_corners_count     << " " << nodes_corners_count + 1
             << " "  << nodes_corners_count + 3 << " " << nodes_corners_count + 2 << std::endl;

    ss_faces << "4 " << nodes_corners_count + 1 << " " << nodes_corners_count + 5
             << " "  << nodes_corners_count + 7 << " " << nodes_corners_count + 3 << std::endl;

    ss_faces << "4 " << nodes_corners_count + 5 << " " << nodes_corners_count + 7
             << " "  << nodes_corners_count + 6 << " " << nodes_corners_count + 4 << std::endl;

    ss_faces << "4 " << nodes_corners_count     << " " << nodes_corners_count + 2
             << " "  << nodes_corners_count + 6 << " " << nodes_corners_count + 4 << std::endl;

    ss_faces << "4 " << nodes_corners_count     << " " << nodes_corners_count + 1
             << " "  << nodes_corners_count + 5 << " " << nodes_corners_count + 4 << std::endl;

    ss_faces << "4 " << nodes_corners_count + 2 << " " << nodes_corners_count + 3
             << " "  << nodes_corners_count + 7 << " " << nodes_corners_count + 6 << std::endl;

    nodes_corners_count += 8;
    faces_count  += 6;
  }

  file << "ply" << std::endl;
  file << "format ascii 1.0" << std::endl;
  file << "comment octree structure" << std::endl;
  file << "element vertex " << nodes_corners_count <<  std::endl;
  file << "property float x" << std::endl;
  file << "property float y" << std::endl;
  file << "property float z" << std::endl;
  file << "element face " << faces_count << std::endl;
  file << "property list uchar int vertex_index" << std::endl;
  file << "end_header" << std::endl;
  file << ss_nodes_corners.str();
  file << ss_faces.str();

  file.close();
  return 0;
}

}
}

#endif // SE_OCTREE_IO_IMPL_HPP
