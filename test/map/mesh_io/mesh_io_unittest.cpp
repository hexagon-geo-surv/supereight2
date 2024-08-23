/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/map/io/mesh_io.hpp>

TEST(MeshIO, hasSupportedMeshExtension)
{
    const std::vector<std::pair<std::string, bool>> test_data = {{"foo.ply", true},
                                                                 {"/foo/bar.vtk", true},
                                                                 {"foo/bar.obj", true},
                                                                 {".ply", true},
                                                                 {"foo.png", false},
                                                                 {"", false}};
    for (const auto& d : test_data) {
        EXPECT_EQ(se::io::has_supported_mesh_extension(d.first), d.second);
    }
}
