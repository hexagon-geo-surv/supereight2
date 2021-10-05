/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/str_utils.hpp"
#include "se/sensor/sensor.hpp"



se::OusterLidarConfig::OusterLidarConfig() : SensorConfigBase()
{
}



se::OusterLidarConfig::OusterLidarConfig(const std::string& yaml_file) : SensorConfigBase(yaml_file)
{
    // Open the file for reading.
    cv::FileStorage fs;
    fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];

    se::yaml::subnode_as_eigen_vector_x(node, "elevation_angles", beam_elevation_angles);
    se::yaml::subnode_as_eigen_vector_x(node, "azimuth_angles", beam_azimuth_angles);
}



std::ostream& se::operator<<(std::ostream& os, const se::OusterLidarConfig& c)
{
    os << str_utils::value_to_pretty_str(c.width, "width") << " px\n";
    os << str_utils::value_to_pretty_str(c.height, "height") << " px\n";
    os << str_utils::value_to_pretty_str(c.near_plane, "near_plane") << " m\n";
    os << str_utils::value_to_pretty_str(c.far_plane, "far_plane") << " m\n";
    os << str_utils::str_to_pretty_str((c.left_hand_frame ? "yes" : "no"), "left_hand_frame")
       << "\n";
    os << str_utils::eigen_matrix_to_pretty_str(c.T_BS, "T_BS") << "\n";
    return os;
}



se::OusterLidar::OusterLidar(const OusterLidarConfig& c) :
        se::SensorBase<se::OusterLidar>(c),
        model(c.width, c.height, c.beam_azimuth_angles, c.beam_elevation_angles)
{
    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane >= 0.f);
    assert(c.far_plane > c.near_plane);
    assert(c.beam_azimuth_angles.size() > 0);
    assert(c.beam_elevation_angles.size() > 0);
    float min_elevation_angle = fabsf(c.beam_elevation_angles[1] - c.beam_elevation_angles[0]);
    for (int i = 2; i < c.beam_elevation_angles.size(); i++) {
        const float diff = fabsf(c.beam_elevation_angles[i - 1] - c.beam_elevation_angles[i]);
        if (diff < min_elevation_angle) {
            min_elevation_angle = diff;
        }
    }
    const float azimuth_angle = 360.0f / c.width;
    min_ray_angle = std::min(min_elevation_angle, azimuth_angle);
    horizontal_fov = 2.0f * M_PI;
    constexpr float deg_to_rad = M_PI / 180.0f;
    const float max_elevation = c.beam_elevation_angles.maxCoeff();
    const float min_elevation = c.beam_elevation_angles.minCoeff();
    vertical_fov = deg_to_rad * (max_elevation - min_elevation);
}



se::OusterLidar::OusterLidar(const OusterLidarConfig& c, const float dsf) :
        se::SensorBase<se::OusterLidar>(c),
        model(c.width / dsf, c.height / dsf, c.beam_azimuth_angles, c.beam_elevation_angles)
{
    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane >= 0.f);
    assert(c.far_plane > c.near_plane);
    assert(c.beam_azimuth_angles.size() > 0);
    assert(c.beam_elevation_angles.size() > 0);
    float min_elevation_angle = fabsf(c.beam_elevation_angles[1] - c.beam_elevation_angles[0]);
    for (int i = 2; i < c.beam_elevation_angles.size(); i++) {
        const float diff = fabsf(c.beam_elevation_angles[i - 1] - c.beam_elevation_angles[i]);
        if (diff < min_elevation_angle) {
            min_elevation_angle = diff;
        }
    }
    const float azimuth_angle = 360.0f / c.width;
    min_ray_angle = std::min(min_elevation_angle, azimuth_angle);
    horizontal_fov = 2.0f * M_PI;
    constexpr float deg_to_rad = M_PI / 180.0f;
    const float max_elevation = c.beam_elevation_angles.maxCoeff();
    const float min_elevation = c.beam_elevation_angles.minCoeff();
    vertical_fov = deg_to_rad * (max_elevation - min_elevation);
}



se::OusterLidar::OusterLidar(const OusterLidar& ol, const float dsf) :
        se::SensorBase<se::OusterLidar>(ol),
        model(ol.model.imageWidth() / dsf,
              ol.model.imageHeight() / dsf,
              ol.model.beamAzimuthAngles(),
              ol.model.beamElevationAngles()) // TODO: Does the beam need to be scaled too?
{
}



int se::OusterLidar::computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre,
                                                 const float map_res,
                                                 const int last_scale,
                                                 const int min_scale,
                                                 const int max_block_scale) const
{
    constexpr float deg_to_rad = M_PI / 180.0f;
    const float dist = block_centre.norm();
    // Compute the side length in metres of a pixel projected dist metres from
    // the camera. This computes the chord length corresponding to the ray angle
    // at distance dist.
    const float pixel_dim = 2.0f * dist * std::tan(min_ray_angle / 2.0f * deg_to_rad);
    // Compute the ratio using the worst case voxel_dim (space diagonal)
    const float pv_ratio = pixel_dim / (std::sqrt(3) * map_res);
    int scale = 0;
    if (pv_ratio < 1.5f) {
        scale = 0;
    }
    else if (pv_ratio < 3.0f) {
        scale = 1;
    }
    else if (pv_ratio < 6.0f) {
        scale = 2;
    }
    else {
        scale = 3;
    }
    scale = std::min(scale, max_block_scale);

    Eigen::Vector3f block_centre_hyst = block_centre;
    bool recompute = false;
    if (scale > last_scale && min_scale != -1) {
        block_centre_hyst -= 0.25 * block_centre_hyst.normalized();
        recompute = true;
    }
    else if (scale < last_scale && min_scale != -1) {
        block_centre_hyst += 0.25 * block_centre_hyst.normalized();
        recompute = true;
    }

    if (recompute) {
        return computeIntegrationScale(block_centre_hyst, map_res, last_scale, -1, max_block_scale);
    }
    else {
        return scale;
    }
}



float se::OusterLidar::nearDistImpl(const Eigen::Vector3f&) const
{
    return near_plane;
}



float se::OusterLidar::farDistImpl(const Eigen::Vector3f&) const
{
    return far_plane;
}



float se::OusterLidar::measurementFromPointImpl(const Eigen::Vector3f& point_S) const
{
    return point_S.norm();
}



bool se::OusterLidar::pointInFrustumImpl(const Eigen::Vector3f& /*point_S*/) const
{
    // TODO Implement
    return false;
}



bool se::OusterLidar::pointInFrustumInfImpl(const Eigen::Vector3f& /*point_S*/) const
{
    // TODO Implement
    return false;
}



bool se::OusterLidar::sphereInFrustumImpl(const Eigen::Vector3f& /*centre_S*/,
                                          const float /*radius*/) const
{
    // TODO Implement
    return false;
}



bool se::OusterLidar::sphereInFrustumInfImpl(const Eigen::Vector3f& /*centre_S*/,
                                             const float /*radius*/) const
{
    // TODO Implement
    return false;
}
