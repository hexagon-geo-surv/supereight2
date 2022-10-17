/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2018-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESHING_IO_HPP
#define SE_MESHING_IO_HPP

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include "se/common/colour_utils.hpp"
#include "se/map/algorithms/marching_cube.hpp"

namespace se {
namespace io {

/**
 * \brief Save a mesh as a VTK file.
 *
 * Documentation for the VTK file format available here
 * https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf.
 *
 * \note The resulting mesh is unoptimized and contains many duplicate
 * vertices.
 *
 * \param[in] mesh       The mesh in map frame to be saved.
 * \param[in] filename   The output filename.
 * \param[in] T_OM       The transformation from the map frame in units of voxels to the output frame. maybe?
 * \param[in] point_data The scalar values of the points/vertices.
 * \param[in] cell_data  The scalar values of the cells/faces.
 * \return 0 on success, nonzero on error.
 */
template<typename FaceT>
int save_mesh_vtk(const Mesh<FaceT>& mesh,
                  const std::string& filename,
                  const Eigen::Matrix4f& T_OM,
                  const float* point_data = nullptr,
                  const float* cell_data = nullptr);

/**
 * \brief Save a mesh as a PLY file.
 *
 * Documentation for the PLY file format available here
 * http://paulbourke.net/dataformats/ply
 *
 * \note The resulting mesh is unoptimized and contains many duplicate
 * vertices.
 *
 * \param[in] mesh       The mesh in map frame to be saved.
 * \param[in] filename   The output filename.
 * \param[in] T_OM       The transformation from the map frame in units of voxels to the output frame. maybe?
 * \param[in] point_data The scalar values of the points/vertices.
 * \param[in] cell_data  The scalar values of the cells/faces.
 * \return 0 on success, nonzero on error.
 */
template<typename FaceT>
int save_mesh_ply(const Mesh<FaceT>& mesh,
                  const std::string& filename,
                  const Eigen::Matrix4f& T_OM,
                  const float* point_data = nullptr,
                  const float* cell_data = nullptr);

/**
 * \brief Save a mesh as an OBJ file.
 *
 * \param[in] mesh     The mesh to be saved.
 * \param[in] filename The output filename.
 * \param[in] T_OM     The transformation from the map frame in units of voxels to the output frame. maybe?
 * \return 0 on success, nonzero on error.
 */
template<typename FaceT>
int save_mesh_obj(const Mesh<FaceT>& mesh,
                  const std::string& filename,
                  const Eigen::Matrix4f& T_OM);

} // namespace io
} // namespace se

#include "impl/mesh_io_impl.hpp"

#endif // SE_MESHING_IO_HPP
