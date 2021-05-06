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

int main()
{
  // Setup log stream
  std::ofstream log_file_stream;
  log_file_stream.open("./log.txt");
  se::perfstats.setFilestream(&log_file_stream);

  // Creating a single-res TSDF map
  const Eigen::Vector3f map_dim(10.f, 10.f, 20.f);
  const float           map_res(0.02f);
  se::TSDFMap<se::Res::Single> map_tsdf(map_dim, map_res);

  // Setup input images
  Eigen::Vector2i input_img_res(640, 480);
  static se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  static se::Image<uint32_t>    input_rgba_img(input_img_res.x(), input_img_res.y());

  int downsampling_factor = 2;

  // Setup processed images
  Eigen::Vector2i processed_img_res = input_img_res / downsampling_factor;
  static se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());
  static se::Image<uint32_t>    processed_rgba_img(processed_img_res.x(), processed_img_res.y());

  // Setup output images / renders
  static uint32_t* output_rgba_img_data     =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  static uint32_t* output_depth_img_data    =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  static uint32_t* output_tracking_img_data =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  static uint32_t* output_volume_img_data   =  new uint32_t[processed_img_res.x() * processed_img_res.y()];

  // Setup sensor
  se::SensorConfig sensor_config;
  sensor_config.width  =  input_img_res.x();
  sensor_config.height =  input_img_res.y();
  sensor_config.fx     =  481.2;
  sensor_config.fy     = -480.0;
  sensor_config.cx     =  319.5;
  sensor_config.cy     =  239.5;
  sensor_config.left_hand_frame = sensor_config.fy < 0;
  sensor_config.near_plane      = 0.4f;
  sensor_config.far_plane       = 6.f;

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(sensor_config, downsampling_factor);

  // Setup reader
  se::Configuration config;
  config.fps                = 24;
  config.drop_frames        = false;
  config.enable_benchmark   = false;
  config.sequence_path      = "/home/nils/workspace_/projects/supereight_2/se_2/datasets/icl_nuim/traj_2/scene.raw";
  config.ground_truth_file  = "/home/nils/workspace_/projects/supereight_2/se_2/datasets/icl_nuim/traj_2/scene.raw.txt";

  // ========= READER INITIALIZATION  =========
  static se::Reader* reader = nullptr;
  reader = se::create_reader(config);

  if (reader == nullptr) {
    exit(EXIT_FAILURE);
  }

  // Setup input, processed and output imgs
  se::ReaderStatus read_ok = se::ReaderStatus::ok;
  Eigen::Matrix4f T_MS;

  se::TrackerConfig tracker_config;
  tracker_config.iterations = {10, 5, 4};
  se::Tracker tracker(map_tsdf, sensor, tracker_config);

  // Integrated depth at given pose
  struct IntegrConfig {};
  IntegrConfig integr_config;
  se::MapIntegrator integrator(map_tsdf, integr_config);

#define TRACK true

  unsigned int frame = 0;
  while (read_ok == se::ReaderStatus::ok) {
    se::perfstats.setIter(frame++);

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

    // Preprocess depth
    se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
    se::preprocessor::downsample_rgba(input_rgba_img,  processed_rgba_img);

    if (frame > 1)
    {
      std::cout << "TRACKED = " << tracker.track(processed_depth_img, T_MS) << std::endl;
    }

    TICK("integration")
    integrator.integrateDepth(processed_depth_img, sensor, T_MS);
    TOCK("integration")

    // Render volume
    se::Image<Eigen::Vector3f> surface_point_cloud_M(processed_img_res.x(), processed_img_res.y());
    se::Image<Eigen::Vector3f> surface_normals_M(processed_img_res.x(), processed_img_res.y());
    se::raycaster::raycastVolume(map_tsdf, surface_point_cloud_M, surface_normals_M, T_MS, sensor);
    const Eigen::Vector3f ambient{ 0.1, 0.1, 0.1};
    se::raycaster::renderVolumeKernel(output_volume_img_data, processed_img_res, se::math::to_translation(T_MS), ambient, surface_point_cloud_M, surface_normals_M);

    // Visualise imgs
    convert_to_output_rgba_img(processed_rgba_img, output_rgba_img_data);
    convert_to_output_depth_img(processed_depth_img, output_depth_img_data);
    tracker.renderTrackingResult(output_tracking_img_data);

    TICK("draw")
    drawthem(output_rgba_img_data,     processed_img_res,
             output_depth_img_data,    processed_img_res,
             output_tracking_img_data, processed_img_res,
             output_volume_img_data,   processed_img_res);
    TOCK("draw")


    if (frame == 1 || frame % 100 == 0)
    {
      map_tsdf.saveMesh(std::to_string(frame));
      map_tsdf.saveSlice(se::math::to_translation(T_MS), std::to_string(frame));
      map_tsdf.saveStrucutre(std::to_string(frame));
    }

    unsigned int final_frame = 800;
    if (frame == final_frame)
    {
      std::cout << "Reached FRAME = " << final_frame << std::endl;
      break;
    }
    se::perfstats.writeToFilestream();
  }

  se::perfstats.writeToFilestream();

  delete[] output_rgba_img_data;
  delete[] output_depth_img_data;

  return 0;
}