/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OUSTER_LIDAR_HPP
#define SE_OUSTER_LIDAR_HPP



namespace se {



struct OusterLidarConfig : public SensorConfigBase {
    Eigen::VectorXf beam_elevation_angles = Eigen::VectorXf(1);
    Eigen::VectorXf beam_azimuth_angles = Eigen::VectorXf(1);

    /** Initializes the config to an invalid sensor model with 0 and NaN parameters.
     */
    OusterLidarConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be initialized
     * as in SensorConfig::SensorConfig().
     */
    OusterLidarConfig(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



std::ostream& operator<<(std::ostream& os, const OusterLidarConfig& c);



class OusterLidar : public SensorBase<OusterLidar> {
    public:
    OusterLidar(const OusterLidarConfig& config);

    OusterLidar(const OusterLidarConfig& config, const float downsampling_factor);

    OusterLidar(const OusterLidar& ouster_lidar, const float downsampling_factor);

    Eigen::Matrix3f K() const
    {
        return Eigen::Matrix3f::Zero();
    }

    int computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre,
                                    const float map_res,
                                    const int last_scale,
                                    const int min_scale,
                                    const int max_block_scale) const;

    float nearDistImpl(const Eigen::Vector3f& ray_S) const;

    float farDistImpl(const Eigen::Vector3f& ray_S) const;

    float measurementFromPointImpl(const Eigen::Vector3f& point_S) const;

    bool pointInFrustumImpl(const Eigen::Vector3f& point_S) const;

    bool pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const;

    bool sphereInFrustumImpl(const Eigen::Vector3f& centre_S, const float radius) const;

    bool sphereInFrustumInfImpl(const Eigen::Vector3f& centre_S, const float radius) const;

    static std::string type()
    {
        return "ousterlidar";
    }

    srl::projection::OusterLidar model;
    float min_ray_angle;

    /** \brief The horizontal field of view in radians. */
    float horizontal_fov;
    /** \brief The vertical field of view in radians. */
    float vertical_fov;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



} // namespace se

#endif // SE_OUSTER_LIDAR_HPP
