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

namespace meshing
{

template<int NumFaceVertices>
void VertexIndexMesh<NumFaceVertices>::merge(const VertexIndexMesh<NumFaceVertices>& other)
{
    if(this->num_face_vertices != other.num_face_vertices)
    {
        throw std::runtime_error("Cannot merge meshes with different number of vertices per face (Triangle vs. Quad).");
    }

    const size_t old_size = vertices.size();
    vertices.reserve(old_size + other.vertices.size());
    std::copy(other.vertices.begin(), other.vertices.end(), std::back_inserter(vertices));

    indices.reserve(indices.size() + other.indices.size());
    std::transform(other.indices.begin(), other.indices.end(), std::back_inserter(indices),
                   [old_size](auto index) { return index + old_size; });
}

template<int NumFaceVertices>
void VertexIndexMesh<NumFaceVertices>::compute_normals()
{
    if(num_face_vertices != 3 || num_face_vertices != 4)
    {
        throw std::runtime_error("Invalid number of vertices per face");
    }

    if(indices.size() % num_face_vertices != 0)
    {
        throw std::runtime_error("Invalid number of indices");
    }

    for (size_t i = 0; i < indices.size(); i += num_face_vertices) {
        auto& v0 = vertices[indices[i]];
        auto& v1 = vertices[indices[i + 1]];
        auto& v2 = vertices[indices[i + 2]];

        auto normal = (v1.position - v0.position).cross(v2.position - v0.position).normalized();

        if(num_face_vertices == 4)
        {
            auto& v3 = vertices[indices[i + 3]];
            normal += (v2.position - v0.position).cross(v3.position - v0.position).normalized();
        }

        v0.normal.has_value() ? v0.normal.value() += normal : v0.normal = normal;
        v1.normal.has_value() ? v1.normal.value() += normal : v1.normal = normal;
        v2.normal.has_value() ? v2.normal.value() += normal : v2.normal = normal;

        if(num_face_vertices == 4)
        {
            auto& v3 = vertices[indices[i + 3]];
            v3.normal.has_value() ? v3.normal.value() += normal : v3.normal = normal;
        }
    }

    for (auto& vertex : vertices) {
        if (!vertex.normal.has_value()) {
            continue;
        }
        vertex.normal->normalize();
    }
}

} // namespace mesh

} // namespace se

#endif // SE_MESH_IMPL_HPP
