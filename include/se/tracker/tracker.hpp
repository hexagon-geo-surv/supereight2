/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.


 Copyright 2016 Emanuele Vespa, Imperial College London

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SE_TRACKER_HPP
#define SE_TRACKER_HPP

#include "se/sensor/sensor.hpp"
#include "se/map/preprocessor.hpp"
#include "se/map/raycaster.hpp"

namespace se {

constexpr float e_delta = 0.1f;

struct TrackerConfig
{
  std::vector<int> iterations;
  float            dist_threshold;
  float            normal_threshold;
  float            track_threshold;
  float            icp_threshold;

  /** Initializes the config to some valid values.
   */
  TrackerConfig();

  /** Initializes the config from a YAML file. Data not present in the YAML file will be initialized
   * as in TrackerConfig::TrackerConfig().
   */
  TrackerConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const TrackerConfig& c);



struct TrackData {
  int result;
  float error;
  float J[6];

  TrackData()
          : result(0),
            error(0.0f),
            J{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {}
};



static inline Eigen::Matrix<float, 6, 6> makeJTJ(const Eigen::Matrix<float, 1, 21>& v)
{

  Eigen::Matrix<float, 6, 6> C = Eigen::Matrix<float, 6, 6>::Zero();
  C.row(0) = v.segment(0, 6);
  C.row(1).segment(1, 5) = v.segment(6,  5);
  C.row(2).segment(2, 4) = v.segment(11, 4);
  C.row(3).segment(3, 3) = v.segment(15, 3);
  C.row(4).segment(4, 2) = v.segment(18, 2);
  C(5,5) = v(20);

  for (int r = 1; r < 6; ++r)
    for (int c = 0; c < r; ++c)
      C(r, c) = C(c, r);
  return C;
}



static inline Eigen::Matrix<float, 6, 1> solve(const Eigen::Matrix<float, 1, 27>& vals)
{
  const Eigen::Matrix<float, 6, 1> b = vals.segment(0, 6);
  const Eigen::Matrix<float, 6, 6> C = makeJTJ(vals.segment(6, 21));
  Eigen::LLT <Eigen::Matrix<float, 6, 6> > llt;
  llt.compute(C);
  Eigen::Matrix<float, 6, 1> res = llt.solve(b);
  return llt.info() == Eigen::Success ? res : Eigen::Matrix<float, 6, 1>::Zero();
}



template <typename MapT, typename SensorT>
class Tracker
{
public:
  Tracker(MapT&               map,
          const SensorT&      sensor,
          const TrackerConfig config = TrackerConfig()) :
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
  bool track(const se::Image<float>& depth_img,
             Eigen::Matrix4f&        T_WS);

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
  bool track(const se::Image<float>&     depth_img,
             Eigen::Matrix4f&            T_WS,
             se::Image<Eigen::Vector3f>& surface_point_cloud_W,
             se::Image<Eigen::Vector3f>& surface_normals_W);

  void renderTrackingResult(uint32_t* tracking_img_data);

private:
  void newReduce(const int              block_idx,
                 float*                 output_data,
                 const Eigen::Vector2i& output_res,
                 TrackData*             J_data,
                 const Eigen::Vector2i& J_res);

  void reduceKernel(float*                 output_data,
                    const Eigen::Vector2i& output_res,
                    TrackData*             J_data,
                    const Eigen::Vector2i& J_res);

  void trackKernel(TrackData*                        output_data,
                   const se::Image<Eigen::Vector3f>& input_point_cloud_S,
                   const se::Image<Eigen::Vector3f>& input_normals_S,
                   const se::Image<Eigen::Vector3f>& surface_point_cloud_M_ref,
                   const se::Image<Eigen::Vector3f>& surface_normals_M_ref,
                   const Eigen::Matrix4f&            T_WS,
                   const Eigen::Matrix4f&            T_WS_ref,
                   const float                       dist_threshold,
                   const float                       normal_threshold);

  bool updatePoseKernel(Eigen::Matrix4f& T_WS,
                        const float*     reduction_output_data,
                        const float      icp_threshold);

  bool checkPoseKernel(Eigen::Matrix4f&       T_WS,
                       Eigen::Matrix4f&       previous_T_WS,
                       const float*           reduction_output_data,
                       const Eigen::Vector2i& reduction_output_res,
                       const float            track_threshold);

  MapT&               map_;
  const SensorT&      sensor_;
  const TrackerConfig config_;
  std::vector<TrackData> tracking_result;
};

}

#include "impl/tracker_impl.hpp"

#endif // SE_TRACKER_HPP

