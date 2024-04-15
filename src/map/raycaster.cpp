/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/raycaster.hpp"

#include <se/common/eigen_utils.hpp>

namespace se {
namespace raycaster {



void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals,
                           const se::Image<Eigen::Vector3f>& point_cloud,
                           const bool /* is_lhc */)
{
    const int width = point_cloud.width();
    const int height = point_cloud.height();

#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const Eigen::Vector3f point = point_cloud[x + width * y];
            if (point.z() == 0.f) {
                normals[x + y * width] = math::g_invalid_normal;
                continue;
            }

            const Eigen::Vector2i p_left = Eigen::Vector2i(std::max(int(x) - 1, 0), y);
            const Eigen::Vector2i p_right = Eigen::Vector2i(std::min(x + 1, (int) width - 1), y);

            // Swapped to match the left-handed coordinate system of ICL-NUIM
            Eigen::Vector2i p_up, p_down;
            if (false) {
                p_up = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_down = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }
            else {
                p_down = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_up = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }

            const Eigen::Vector3f left = point_cloud[p_left.x() + width * p_left.y()];
            const Eigen::Vector3f right = point_cloud[p_right.x() + width * p_right.y()];
            const Eigen::Vector3f up = point_cloud[p_up.x() + width * p_up.y()];
            const Eigen::Vector3f down = point_cloud[p_down.x() + width * p_down.y()];

            if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0) {
                normals[x + y * width] = math::g_invalid_normal;
                continue;
            }
            const Eigen::Vector3f dv_x = right - left;
            const Eigen::Vector3f dv_y = up - down;
            normals[x + y * width] = dv_x.cross(dv_y).normalized();
        } // x
    }     // y
}



void render_volume(se::Image<RGBA>& render,
                   const Eigen::Vector3f& light_M,
                   const Eigen::Vector3f& ambient_M,
                   const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                   const se::Image<Eigen::Vector3f>& surface_normals_M,
                   const se::Image<int8_t>& surface_scale)
{
    assert(render.width() == surface_point_cloud_M.width());
    assert(render.height() == surface_point_cloud_M.height());
    assert(render.width() == surface_normals_M.width());
    assert(render.height() == surface_normals_M.height());
    assert(render.width() == surface_scale.width());
    assert(render.height() == surface_scale.height());
#pragma omp parallel for
    for (size_t pixel_idx = 0; pixel_idx < render.size(); pixel_idx++) {
        const Eigen::Vector3f surface_point_M = surface_point_cloud_M[pixel_idx];
        const Eigen::Vector3f surface_normal_M = surface_normals_M[pixel_idx];

        if (surface_normal_M != math::g_invalid_normal && surface_normal_M.norm() > 0.f) {
            const Eigen::Vector3f diff = (surface_point_M - light_M).normalized();
            const Eigen::Vector3f dir =
                Eigen::Vector3f::Constant(std::max(surface_normal_M.normalized().dot(diff), 0.f));
            Eigen::Vector3f col = dir + ambient_M;
            se::eigen::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones());
            const RGB rgb = scale_colour(surface_scale[pixel_idx]);
            col = col.cwiseProduct(Eigen::Vector3f(rgb.r, rgb.g, rgb.b));
            const Eigen::Matrix<std::uint8_t, 3, 1> rgb8 = col.cast<std::uint8_t>();
            render[pixel_idx] = {rgb8.x(), rgb8.y(), rgb8.z(), 0xFF};
        }
        else {
            render[pixel_idx] = {0x00, 0x00, 0x00, 0xFF};
        }
    }
}

} // namespace raycaster
} // namespace se
