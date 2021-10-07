/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SENSOR_HPP
#define SE_SENSOR_HPP

#include "se/common/image_utils.hpp"
#include "se/common/math_util.hpp"
#include "se/common/projection.hpp"
#include "se/common/yaml.hpp"
#include "se/image/image.hpp"



namespace se {



struct SensorConfigBase {
    // General
    int width;
    int height;
    float near_plane;
    float far_plane;
    bool left_hand_frame;
    Eigen::Matrix4f T_BS;

    /** Initializes the config to an invalid sensor model with 0 and NaN parameters.
     */
    SensorConfigBase();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be initialized
     * as in SensorConfig::SensorConfig().
     */
    SensorConfigBase(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



template<typename DerivedT>
class SensorBase {
    public:
    template<typename ConfigT>
    SensorBase(const ConfigT& c) :
            left_hand_frame(c.left_hand_frame),
            near_plane(c.near_plane),
            far_plane(c.far_plane),
            T_BS(c.T_BS)
    {
    }

    SensorBase(const DerivedT& d) :
            left_hand_frame(d.left_hand_frame),
            near_plane(d.near_plane),
            far_plane(d.far_plane),
            T_BS(d.T_BS)
    {
    }


    /**
     * \brief Project a point in sensor frame to its image value.
     *
     * \tparam ValidPredicate
     * \param[in]  point_S          The point to project to the image
     * \param[in]  img              The image to get the value from
     * \param[out] img_value        The image value the point projects to
     * \param[in]  valid_predicate  The lambda function verifying if the value is valid (e.g. infront of far dist)
     *
     * \return True if the image value is valid, false otherwise
     */
    template<typename ValidPredicate>
    bool projectToPixelValue(const Eigen::Vector3f& point_S,
                             const se::Image<float>& img,
                             float& img_value,
                             ValidPredicate valid_predicate) const;

    /**
     * \brief Get the image value for a given pixel coordindate.
     *
     * \tparam ValidPredicate
     * \param[in]  pixel_f          The pixel coordinates to get the image value from
     * \param[in]  img              The image to get the value from
     * \param[out] img_value        The image value corresponding the the pixel
     * \param[in]  valid_predicate  The lambda function verifying if the value is valid (e.g. infront of far dist)
     *
     * \return True if the image value is valid, false otherwise
     */
    template<typename ValidPredicate>
    bool getPixelValue(const Eigen::Vector2f& pixel_f,
                       const se::Image<float>& img,
                       float& img_value,
                       ValidPredicate valid_predicate) const;
    /**
     * \brief Computes the scale corresponding to the back-projected pixel size
     * in voxel space.
     *
     * \param[in] block_centre_S  The coordinates of the block
     *                            centre in the sensor frame.
     * \param[in] map_res         The resolution of the map in [meter].
     * \param[in] last_scale      Scale from which propagate up voxel
     *                            values.
     * \param[in] min_scale       Finest scale at which data has been
     *                            integrated into the voxel block (-1 if no
     *                            data has been integrated yet).
     * \param[in] max_block_scale The maximum allowed scale within a
     *                            VoxelBlock.
     * \return The scale that should be used for the integration.
     */
    int computeIntegrationScale(const Eigen::Vector3f& block_centre_S,
                                const float map_res,
                                const int last_scale,
                                const int min_scale,
                                const int max_block_scale) const;

    /**
     * \brief Return the minimum distance at which measurements are available
     * along the ray passing through pixels x and y.
     *
     * This differs from the PinholeCamera::near_plane since the near_plane is
     * a z-value while nearDist is a distance along a ray.
     *
     * \param[in] ray_S The ray starting from the sensor centre and expressed
     *                  in the sensor frame along which nearDist
     *                  will be computed.
     * \return The minimum distance along the ray through the pixel at which
     *         valid measurements may be encountered.
     */
    float nearDist(const Eigen::Vector3f& ray_S) const;

    /**
     * \brief Return the maximum distance at which measurements are available
     * along the ray passing through pixels x and y.
     *
     * This differs from the PinholeCamera::far_plane since the far_plane is a
     * z-value while farDist is a distance along a ray.
     *
     * \param[in] ray_S The ray starting from the sensor centre and expressed
     *                  in the sensor frame along which nearDist
     *                  will be computed.
     * \return The maximum distance along the ray through the pixel at which
     *         valid measurements may be encountered.
     */
    float farDist(const Eigen::Vector3f& ray_S) const;

    /**
     * \brief Convert a point in the sensor frame into a depth measurement.
     * For the PinholeCamera this means returning the z-coordinate
     * of the point.
     *
     * \param[in] point_C A point observed by the sensor expressed in the
     *                    sensor frame.
     * \return The depth value that the sensor would get from this point.
     */
    float measurementFromPoint(const Eigen::Vector3f& point_S) const;

    /**
     * \brief Test whether a 3D point in sensor coordinates is inside the
     * sensor frustum.
     */
    bool pointInFrustum(const Eigen::Vector3f& point_S) const;

    /**
     * \brief Test whether a 3D point in sensor coordinates is inside the
     * sensor frustum.
     *
     * The difference from PinholeCamera::pointInFrustum is that it is assumed
     * that the far plane is at infinity.
     */
    bool pointInFrustumInf(const Eigen::Vector3f& point_S) const;

    /**
     * \brief Test whether a sphere in sensor coordinates is inside the sensor
     * frustum.
     *
     * It is tested whether the sphere's centre is inside the sensor frustum
     * offest outwards by the sphere's radius. This is a quick test that in
     * some rare cases may return a sphere as being visible although it isn't.
     */
    bool sphereInFrustum(const Eigen::Vector3f& centre_S, const float radius) const;

    /**
     * \brief Test whether a sphere in sensor coordinates is inside the sensor
     * frustum.
     *
     * The difference from PinholeCamera::sphereInFrustum is that it is assumed
     * that the far plane is at infinity.
     */
    bool sphereInFrustumInf(const Eigen::Vector3f& centre_S, const float radius) const;

    //    static std::string type()
    //    {
    //      return this->underlying().typeImpl(); // TODO:
    //    }

    bool left_hand_frame;
    float near_plane;
    float far_plane;
    Eigen::Matrix4f T_BS;

    private:
    // Make sure the derived class and the template parameter are the same (i.e. prevent class D1 : Base<D2>)
    SensorBase(){};
    friend DerivedT;

    // Simplify access to derived member functions
    DerivedT& underlying();
    const DerivedT& underlying() const;
};



} // namespace se

#include "impl/sensor_impl.hpp"
#include "ouster_lidar.hpp"
#include "pinhole_camera.hpp"

#endif // SE_SENSOR_HPP
