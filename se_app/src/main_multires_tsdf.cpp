#include "se/utils/type_util.hpp"
#include "se/octree/octree.hpp"
#include "se/map.hpp"
#include "se/map_integrator.hpp"
#include "se/raycaster.hpp"
#include "se/tracker.hpp"
#include "se/sensor.hpp"
#include "se/preprocessor.hpp"
#include "se/perfstats.hpp"
#include "se/timings.hpp"

#include "draw.h"
#include "reader.hpp"

#define TRACK true //< Use ICP tracking or ground truth

int main(int argc, char** argv)
{
  std::string output_path = "/home/nils/workspace_/projects/supereight-2-srl/out";

  // Setup log stream
  std::ofstream log_file_stream;
  log_file_stream.open(output_path + "/log.txt");
  se::perfstats.setFilestream(&log_file_stream);

  // Creating a single-res TSDF map
  const Eigen::Vector3f map_dim(10.f, 10.f, 10.f);
  const float           map_res(0.01f);

  // Data config
  se::TSDFDataConfig data_config;
  data_config.truncation_boundary = 16 * map_res;
  data_config.max_weight          = 100;

  se::TSDFMap<se::Res::Multi> map_multires_tsdf(map_dim, map_res, data_config);

  // Setup input images
  Eigen::Vector2i input_img_res(640, 480);
  se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  se::Image<uint32_t>    input_rgba_img(input_img_res.x(), input_img_res.y());

  int downsampling_factor = 2;

  // Setup processed images
  Eigen::Vector2i processed_img_res = input_img_res / downsampling_factor;
  se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());
  se::Image<uint32_t>    processed_rgba_img(processed_img_res.x(), processed_img_res.y());

  // Setup output images / renders
  uint32_t* output_rgba_img_data     =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  uint32_t* output_depth_img_data    =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  uint32_t* output_tracking_img_data =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  uint32_t* output_volume_img_data   =  new uint32_t[processed_img_res.x() * processed_img_res.y()];

  // Setup sensor
  se::SensorConfig sensor_config;
  sensor_config.width           =  input_img_res.x();
  sensor_config.height          =  input_img_res.y();
  sensor_config.fx              =  481.2;
  sensor_config.fy              = -480.0;
  sensor_config.cx              =  319.5;
  sensor_config.cy              =  239.5;
  sensor_config.left_hand_frame =  sensor_config.fy < 0;
  sensor_config.near_plane      =  0.4f;
  sensor_config.far_plane       =  6.f;

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(sensor_config, downsampling_factor);

  // Create the reader configuration from the general configuration
  se::ReaderConfig reader_config;
  reader_config.reader_type       = se::ReaderType::RAW;
  reader_config.fps               = 24;
  reader_config.drop_frames       = false;
  reader_config.verbose           = 0;
  reader_config.sequence_path     = (argc >= 2) ? argv[1] : "/home/nils/workspace_/projects/supereight-2-srl/datasets/icl_nuim/traj_2/scene.raw";
  reader_config.ground_truth_file = (argc >= 3) ? argv[2] : "/home/nils/workspace_/projects/supereight-2-srl/datasets/icl_nuim/traj_2/scene.raw.txt";;

  // ========= READER INITIALIZATION  =========
  se::Reader* reader = nullptr;
  reader = se::create_reader(reader_config);

  if (reader == nullptr) {
    exit(EXIT_FAILURE);
  }

  // Setup input, processed and output imgs
  se::ReaderStatus read_ok = se::ReaderStatus::ok;
  Eigen::Matrix4f T_MS;

  se::TrackerConfig tracker_config;
  tracker_config.iterations = {10, 5, 4};
  se::Tracker tracker(map_multires_tsdf, sensor, tracker_config);

  // Integrated depth at given pose
  se::MapIntegrator integrator(map_multires_tsdf);

  unsigned int frame = 0;

  se::Image<Eigen::Vector3f> surface_point_cloud_M(processed_img_res.x(), processed_img_res.y());
  se::Image<Eigen::Vector3f> surface_normals_M(processed_img_res.x(), processed_img_res.y());

  while (read_ok == se::ReaderStatus::ok) {
    se::perfstats.setIter(frame++);

    TICK("total")

    TICK("read")
    if constexpr (TRACK)
    {
      if (frame == 1)
      {
        read_ok = reader->nextData(input_depth_img, input_rgba_img, T_MS);
      } else
      {
        read_ok = reader->nextData(input_depth_img, input_rgba_img);
      }
    } else
    {
      read_ok = reader->nextData(input_depth_img, input_rgba_img, T_MS);
    }
    TOCK("read")

    // Preprocess depth
    TICK("ds-depth")
    se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
    TOCK("ds-depth")
    TICK("ds-rgba")
    se::preprocessor::downsample_rgba(input_rgba_img,  processed_rgba_img);
    TOCK("ds-rgba")

    // Render volume
    TICK("tracking")
    if (frame > 1)
    {
      tracker.track(processed_depth_img, T_MS, surface_point_cloud_M, surface_normals_M);
    }
    TOCK("tracking")

    TICK("integration")
    integrator.integrateDepth(processed_depth_img, sensor, T_MS, frame);
    TOCK("integration")

    TICK("raycast")
    se::raycaster::raycastVolume(map_multires_tsdf, surface_point_cloud_M, surface_normals_M, T_MS, sensor);
    TOCK("raycast")

    const Eigen::Vector3f ambient{ 0.1, 0.1, 0.1};

    TICK("render")
    se::raycaster::renderVolumeKernel(output_volume_img_data, processed_img_res, se::math::to_translation(T_MS), ambient, surface_point_cloud_M, surface_normals_M);
    convert_to_output_rgba_img(processed_rgba_img, output_rgba_img_data);
    convert_to_output_depth_img(processed_depth_img, output_depth_img_data);
    tracker.renderTrackingResult(output_tracking_img_data);
    TOCK("render")



    TICK("draw")
    drawthem(output_rgba_img_data,     processed_img_res,
             output_depth_img_data,    processed_img_res,
             output_tracking_img_data, processed_img_res,
             output_volume_img_data,   processed_img_res);
    TOCK("draw")

    TOCK("total")

    if (frame == 1 || frame % 100 == 0)
    {
      map_multires_tsdf.saveMesh(output_path + "/mesh", std::to_string(frame));
//      map_multires_tsdf.saveSlice(output_path + "/slice", se::math::to_translation(T_MS), std::to_string(frame));
//      map_multires_tsdf.saveStrucutre(output_path + "/struct", std::to_string(frame));
    }

    se::perfstats.writeToFilestream();

    unsigned int final_frame = 800;
    if (frame == final_frame)
    {
      break;
    }
  }

  delete[] output_rgba_img_data;
  delete[] output_depth_img_data;
  delete[] output_tracking_img_data;
  delete[] output_volume_img_data;

  return 0;
}
