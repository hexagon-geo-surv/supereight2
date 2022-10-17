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

#include "se/common/colour_utils.hpp"
#include "se/map/algorithms/mesh.hpp"

namespace se {
namespace io {


/** \brief Save a mesh as a VTK file.
 * The VTK file format is documented here:
 * https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf
 *
 * \param[in] mesh_M   The mesh to be saved expressed in some mesh frame M.
 * \param[in] filename The file where the mesh will be saved.
 * \param[in] T_OM     The transformation from the mesh frame M to some output frame O. The
 *                     transformation will be applied to each mesh vertex before saving it.
 * \return Zero on success, non-zero on error.
 */
template<typename FaceT>
int save_mesh_vtk(const Mesh<FaceT>& mesh_M,
                  const std::string& filename,
                  const Eigen::Matrix4f& T_OM = Eigen::Matrix4f::Identity());

/** \brief Save a mesh as a PLY file.
 * The PLY file format is documented here:
 * http://paulbourke.net/dataformats/ply
 *
 * \param[in] mesh_M   The mesh to be saved expressed in some mesh frame M.
 * \param[in] filename The file where the mesh will be saved.
 * \param[in] T_OM     The transformation from the mesh frame M to some output frame O. The
 *                     transformation will be applied to each mesh vertex before saving it.
 * \return Zero on success, non-zero on error.
 */
template<typename FaceT>
int save_mesh_ply(const Mesh<FaceT>& mesh_M,
                  const std::string& filename,
                  const Eigen::Matrix4f& T_OM = Eigen::Matrix4f::Identity());

/** \brief Save a mesh as an Wavefront OBJ file.
 * The Wavefront OBJ file format is documented here:
 * http://fegemo.github.io/cefet-cg/attachments/obj-spec.pdf
 *
 * \param[in] mesh_M   The mesh to be saved expressed in some mesh frame M.
 * \param[in] filename The file where the mesh will be saved.
 * \param[in] T_OM     The transformation from the mesh frame M to some output frame O. The
 *                     transformation will be applied to each mesh vertex before saving it.
 * \return Zero on success, non-zero on error.
 */
template<typename FaceT>
int save_mesh_obj(const Mesh<FaceT>& mesh_M,
                  const std::string& filename,
                  const Eigen::Matrix4f& T_OM = Eigen::Matrix4f::Identity());

} // namespace io
} // namespace se

#include "impl/mesh_io_impl.hpp"

#endif // SE_MESHING_IO_HPP