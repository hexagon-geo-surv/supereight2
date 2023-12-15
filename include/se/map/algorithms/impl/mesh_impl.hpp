/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESH_IMPL_HPP
#define SE_MESH_IMPL_HPP

namespace se {

static inline TriangleMesh quad_to_triangle_mesh(const QuadMesh& quad_mesh)
{
    TriangleMesh triangle_mesh;
    triangle_mesh.reserve(2 * quad_mesh.size());
    for (const auto& f : quad_mesh) {
        triangle_mesh.push_back(
            {{f.vertexes[0], f.vertexes[1], f.vertexes[2]}, f.max_vertex_scale});
        triangle_mesh.push_back(
            {{f.vertexes[0], f.vertexes[2], f.vertexes[3]}, f.max_vertex_scale});
    }
    return triangle_mesh;
}

} // namespace se

#endif // SE_MESH_IMPL_HPP
