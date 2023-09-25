/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_TRACKER_HPP
#define SE_TRACKER_HPP

#include "se/map/preprocessor.hpp"
#include "se/map/raycaster.hpp"
#include "se/sensor/sensor.hpp"

namespace se {

constexpr float e_delta = 0.1f;

struct TrackerConfig {
    std::vector<int> iterations{10, 5, 4};
    float dist_threshold = 0.1f;
    float normal_threshold = 0.8f;
    float track_threshold = 0.15f;
    float icp_threshold = 0.00001f;

    /** Reads the struct members from the "tracker" node of a YAML file. Members not present in the
     * YAML file aren't modified.
     */
    void readYaml(const std::string& filename);
};

std::ostream& operator<<(std::ostream& os, const TrackerConfig& c);



struct TrackData {
    int result;
    float error;
    float J[6];

    TrackData() : result(0), error(0.0f), J{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    {
    }
};



static inline Eigen::Matrix<float, 6, 6> makeJTJ(const Eigen::Matrix<float, 1, 21>& v)
{
    Eigen::Matrix<float, 6, 6> C = Eigen::Matrix<float, 6, 6>::Zero();
    C.row(0) = v.segment(0, 6);
    C.row(1).segment(1, 5) = v.segment(6, 5);
    C.row(2).segment(2, 4) = v.segment(11, 4);
    C.row(3).segment(3, 3) = v.segment(15, 3);
    C.row(4).segment(4, 2) = v.segment(18, 2);
    C(5, 5) = v(20);

    for (int r = 1; r < 6; ++r)
        for (int c = 0; c < r; ++c)
            C(r, c) = C(c, r);
    return C;
}



static inline Eigen::Matrix<float, 6, 1> solve(const Eigen::Matrix<float, 1, 27>& vals)
{
    const Eigen::Matrix<float, 6, 1> b = vals.segment(0, 6);
    const Eigen::Matrix<float, 6, 6> C = makeJTJ(vals.segment(6, 21));
    Eigen::LLT<Eigen::Matrix<float, 6, 6>> llt;
    llt.compute(C);
    Eigen::Matrix<float, 6, 1> res = llt.solve(b);
    return llt.info() == Eigen::Success ? res : Eigen::Matrix<float, 6, 1>::Zero();
}



template<typename MapT, typename SensorT>
class Tracker {
    public:
    Tracker(MapT& map, const SensorT& sensor, const TrackerConfig config = TrackerConfig()) :
            map_(map),
            sensor_(sensor),
            config_(config),
            tracking_result(sensor_.model.imageWidth() * sensor_.model.imageHeight(), TrackData())
    {
    }

    /**
     * \brief Track the current pose using ICP.
     *
     * \param[in]     depth_image_    The depth image fitting the sensor model
     * \param[in,out] T_WS            [in] The previous best pose estimate, [out] The new best pose estimate
     *
     * \return The tracking success.
     */
    bool track(const se::Image<float>& depth_img, Eigen::Matrix4f& T_WS);

    /**
     * \brief Track the current pose using ICP.
     *
     * \param[in]     depth_image_            The depth image fitting the sensor model
     * \param[in,out] T_WS                    [in] The previous best pose estimate T_WS_ref, [out] The new best pose estimate T_WS
     * \param[in]     surface_point_cloud_W   Surface point cloud in world frame
     * \param[in]     surface_normals_W       Surface normals in world frame
     *
     * \return The tracking success.
     */
    bool track(const se::Image<float>& depth_img,
               Eigen::Matrix4f& T_WS,
               se::Image<Eigen::Vector3f>& surface_point_cloud_W,
               se::Image<Eigen::Vector3f>& surface_normals_W);

    void renderTrackingResult(uint32_t* tracking_img_data);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    void newReduce(const int block_idx,
                   float* output_data,
                   const Eigen::Vector2i& output_res,
                   TrackData* J_data,
                   const Eigen::Vector2i& J_res);

    void reduceKernel(float* output_data,
                      const Eigen::Vector2i& output_res,
                      TrackData* J_data,
                      const Eigen::Vector2i& J_res);

    void trackKernel(TrackData* output_data,
                     const se::Image<Eigen::Vector3f>& input_point_cloud_S,
                     const se::Image<Eigen::Vector3f>& input_normals_S,
                     const se::Image<Eigen::Vector3f>& surface_point_cloud_M_ref,
                     const se::Image<Eigen::Vector3f>& surface_normals_M_ref,
                     const Eigen::Matrix4f& T_WS,
                     const Eigen::Matrix4f& T_WS_ref,
                     const float dist_threshold,
                     const float normal_threshold);

    bool updatePoseKernel(Eigen::Matrix4f& T_WS,
                          const float* reduction_output_data,
                          const float icp_threshold);

    bool checkPoseKernel(Eigen::Matrix4f& T_WS,
                         Eigen::Matrix4f& previous_T_WS,
                         const float* reduction_output_data,
                         const Eigen::Vector2i& reduction_output_res,
                         const float track_threshold);

    MapT& map_;
    const SensorT& sensor_;
    const TrackerConfig config_;
    std::vector<TrackData> tracking_result;
};

} // namespace se

#include "impl/tracker_impl.hpp"

#endif // SE_TRACKER_HPP
