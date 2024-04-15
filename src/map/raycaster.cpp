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
                   const se::Image<Eigen::Vector3f>& surface_points_W,
                   const se::Image<Eigen::Vector3f>& surface_normals_W,
                   const se::Image<int8_t>& surface_scale,
                   const Eigen::Vector3f& light_source_W,
                   const RGB ambient_light)
{
    assert(render.width() == surface_points_W.width());
    assert(render.height() == surface_points_W.height());
    assert(render.width() == surface_normals_W.width());
    assert(render.height() == surface_normals_W.height());
    assert(render.width() == surface_scale.width());
    assert(render.height() == surface_scale.height());
    const Eigen::Vector3f ambient_light_f(ambient_light.r, ambient_light.g, ambient_light.b);
#pragma omp parallel for
    for (size_t pixel_idx = 0; pixel_idx < render.size(); pixel_idx++) {
        RGBA colour;
        const Eigen::Vector3f& surface_normal_W = surface_normals_W[pixel_idx];
        if (surface_normal_W != math::g_invalid_normal && surface_normal_W.norm() > 0.f) {
            const Eigen::Vector3f& surface_point_W = surface_points_W[pixel_idx];
            const Eigen::Vector3f light_dir_W = (surface_point_W - light_source_W).normalized();
            assert(surface_normal_W.isApprox(surface_normal_W.normalized()));
            // The intensity must be 0 if the light is opposite the surface (negative dot product).
            const float intensity = std::max(surface_normal_W.dot(light_dir_W), 0.0f);
            const RGB rgb = scale_colour(surface_scale[pixel_idx]);
            const Eigen::Vector3f diffuse = intensity * Eigen::Vector3f(rgb.r, rgb.g, rgb.b);
            Eigen::Vector3f col = diffuse + ambient_light_f;
            se::eigen::clamp(col, Eigen::Vector3f::Zero(), Eigen::Vector3f::Constant(255.0f));
            colour.r = col.x();
            colour.g = col.y();
            colour.b = col.z();
        }
        render[pixel_idx] = colour;
    }
}

} // namespace raycaster
} // namespace se
