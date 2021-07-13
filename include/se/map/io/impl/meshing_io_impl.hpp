// SPDX-FileCopyrightText: 2018-2020 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2016 Emanuele Vespa, ETH Zürich
// SPDX-FileCopyrightText: 2019-2020 Nils Funk, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_MESHING_IO_IMPL_HPP
#define SE_MESHING_IO_IMPL_HPP

namespace se {
namespace io {

int save_mesh_vtk(const std::vector<Triangle>& mesh,
                  const std::string&           filename,
                  const Eigen::Matrix4f&       T_WM,
                  const float*                 point_data,
                  const float*                 cell_data) {

  // Open the file for writing.
  std::ofstream file (filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  const bool has_point_data = point_data != nullptr;
  const bool has_cell_data = cell_data != nullptr;
  const size_t num_faces = mesh.size();
  const size_t num_vertices = 3 * num_faces;

  // Write the header.
  file << "# vtk DataFile Version 1.0\n";
  file << "vtk mesh generated from supereight 2\n";
  file << "ASCII\n";
  file << "DATASET POLYDATA\n";

  // Write the vertices.
  file << "POINTS " << num_vertices << " FLOAT\n";
  for (size_t i = 0; i < num_faces; ++i ) {
    const Triangle& triangle_M = mesh[i];
    const Eigen::Vector3f vertex_0_W = (T_WM * triangle_M.vertexes[0].homogeneous()).head(3);
    const Eigen::Vector3f vertex_1_W = (T_WM * triangle_M.vertexes[1].homogeneous()).head(3);
    const Eigen::Vector3f vertex_2_W = (T_WM * triangle_M.vertexes[2].homogeneous()).head(3);
    file << vertex_0_W.x() << " " << vertex_0_W.y() << " " << vertex_0_W.z() << "\n";
    file << vertex_1_W.x() << " " << vertex_1_W.y() << " " << vertex_1_W.z() << "\n";
    file << vertex_2_W.x() << " " << vertex_2_W.y() << " " << vertex_2_W.z() << "\n";
  }

  // Write the faces.
  file << "POLYGONS " << num_faces << " " << num_faces * 4 << "\n";
  for (size_t i = 0; i < num_faces; ++i ) {
    file << "3 " << 3*i << " " << 3*i + 1 << " " << 3*i + 2 << "\n";
  }

  // Write the vertex data.
  if (has_point_data) {
    file << "POINT_DATA " << num_vertices << "\n";
    file << "SCALARS vertex_scalars float 1\n";
    file << "LOOKUP_TABLE default\n";
    for (size_t i = 0; i < num_faces; ++i ) {
      file << point_data[i*3] << "\n";
      file << point_data[i*3 + 1] << "\n";
      file << point_data[i*3 + 2] << "\n";
    }
  }

  // Write the face scale colours.
  file << "CELL_DATA " << num_faces << "\n";
  file << "COLOR_SCALARS RGBA 4\n";
  for (size_t i = 0; i < num_faces; ++i ) {
    const Triangle& triangle_M = mesh[i];
    // Colour the triangle depending on its scale.
    const Eigen::Vector3f RGB = se::colours::scale[triangle_M.max_vertex_scale] / 255.0f;
    file << RGB[0] << " " << RGB[1] << " " << RGB[2] << " 1\n";
  }

  // Write the face data.
  if(has_cell_data){
    file << "SCALARS cell_scalars float 1\n";
    file << "LOOKUP_TABLE default\n";
    for (size_t i = 0; i < num_faces; ++i ) {
      file << cell_data[i] << "\n";
    }
  }

  file.close();
  return 0;
}



int save_mesh_ply(const std::vector<Triangle>& mesh,
                  const std::string&           filename,
                  const Eigen::Matrix4f&       T_WM,
                  const float*                 point_data,
                  const float*                 cell_data) {

  // Open the file for writing.
  std::ofstream file (filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  const bool has_point_data = point_data != nullptr;
  const bool has_cell_data = cell_data != nullptr;
  const size_t num_faces = mesh.size();
  const size_t num_vertices = 3 * num_faces;

  // Write header
  file << "ply\n";
  file << "format ascii 1.0\n";
  file << "comment Generated by supereight\n";
  file << "element vertex " << num_vertices << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";
  if (has_point_data) {
    file << "property float vertex_value\n";
  }
  file << "element face " << num_faces << "\n";
  file << "property list uchar int vertex_index\n";
  file << "property uchar red\n";
  file << "property uchar green\n";
  file << "property uchar blue\n";
  if (has_cell_data) {
    file << "property float face_value\n";
  }
  file << "end_header\n";

  // Write vertices and vertex data
  for (size_t i = 0; i < num_faces; ++i ) {
    const Triangle& triangle_M = mesh[i];
    // Write each triangle vertex
    for (int v = 0; v < 3; ++v) {
      const Eigen::Vector3f vertex_W = (T_WM * triangle_M.vertexes[v].homogeneous()).head(3);
      file << vertex_W.x() << " " << vertex_W.y() << " " << vertex_W.z();
      if (has_point_data) {
        file << " " << point_data[3*i + v] << "\n";
      } else {
        file << "\n";
      }
    }
  }

  // Write faces and face data
  for (size_t i = 0; i < num_faces; ++i ) {
    const Triangle& triangle_M = mesh[i];
    file << "3 " << 3*i << " " << 3*i + 1 << " " << 3*i + 2;
    // Write the triangle scale colour.
    const Eigen::Vector3i RGB = se::colours::scale[triangle_M.max_vertex_scale].cast<int>();
    file << " " << RGB[0] << " " << RGB[1] << " " << RGB[2];
    if (has_cell_data) {
      file << " " << cell_data[i] << "\n";
    } else {
      file << "\n";
    }
  }

  file.close();
  return 0;
}



int save_mesh_obj(const std::vector<Triangle>& mesh,
                  const std::string&           filename){

  // Open the file for writing.
  std::ofstream file (filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  std::stringstream points_M;
  std::stringstream faces;
  int point_count = 0;
  int face_count = 0;

  for(unsigned int i = 0; i < mesh.size(); i++){
    const Triangle& triangle_M = mesh[i];
    points_M << "v " << triangle_M.vertexes[0].x() << " "
             << triangle_M.vertexes[0].y() << " "
             << triangle_M.vertexes[0].z() << std::endl;
    points_M << "v " << triangle_M.vertexes[1].x() << " "
             << triangle_M.vertexes[1].y() << " "
             << triangle_M.vertexes[1].z() << std::endl;
    points_M << "v " << triangle_M.vertexes[2].x() << " "
             << triangle_M.vertexes[2].y() << " "
             << triangle_M.vertexes[2].z() << std::endl;

    faces  << "f " << (face_count*3)+1 << " " << (face_count*3)+2
           << " " << (face_count*3)+3 << std::endl;

    point_count +=3;
    face_count += 1;
  }

  file << "# OBJ file format with ext .obj" << std::endl;
  file << "# vertex count = " << point_count << std::endl;
  file << "# face count = " << face_count << std::endl;
  file << points_M.str();
  file << faces.str();

  file.close();
  return 0;
}


} // namespace io
} // namespace se

#endif // SE_MESHING_IO_IMPL_HPP
