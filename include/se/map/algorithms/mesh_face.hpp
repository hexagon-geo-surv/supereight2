// SPDX-FileCopyrightText: 2016 Emanuele Vespa, Imperial College London
// SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab
// SPDX-FileCopyrightText: 2021 Nils Funk, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_MESH_FACE_HPP
#define SE_MESH_FACE_HPP

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <vector>

namespace se {

template<size_t NumVertexes>
struct MeshFace {
    std::array<Eigen::Vector3f, NumVertexes> vertexes;
    int8_t max_vertex_scale;
    static constexpr size_t num_vertexes = NumVertexes;

    MeshFace() : max_vertex_scale(0)
    {
        vertexes.fill(Eigen::Vector3f::Zero());
    }
};

template<typename FaceT>
using Mesh = std::vector<FaceT>;

typedef MeshFace<3> Triangle;
typedef Mesh<Triangle> TriangleMesh;

typedef MeshFace<4> Quad;
typedef Mesh<Quad> QuadMesh;

} // namespace se

#endif // SE_MESH_FACE_HPP
