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

static std::ostream* log_stream;
static std::ofstream log_file_stream;

int main() {
  log_file_stream.open("./log.txt");
  log_stream = &log_file_stream;
  se::perfstats.setFilestream(&log_file_stream);


  // MILESTONE 01
  // Creating a single-res TSDF map with colour and 4 voxel block size
  const Eigen::Vector3f map_dim(10.f, 10.f, 20.f);
  const float map_res(0.02f);
  se::TSDFMap<se::Res::Single> map_tsdf(map_dim, map_res);

  // Setup sensor
  se::SensorConfig sensor_config;
  int ds = 2;
  sensor_config.width  =  640   / ds;
  sensor_config.height =  480   / ds;
  sensor_config.fx     =  481.2 / ds;
  sensor_config.fy     = -480.0 / ds;
  sensor_config.cx     =  (319.5 + 0.5f) / ds - 0.5f;
  sensor_config.cy     =  (239.5 + 0.5f) / ds - 0.5f;
  sensor_config.left_hand_frame = sensor_config.fy < 0;
  sensor_config.near_plane      = 0.4f;
  sensor_config.far_plane       = 6.f;

  const se::PinholeCamera sensor(sensor_config);

  // Setup reader
  static se::Reader* reader = nullptr;

  se::Configuration config;
  config.fps                = 24;
  config.drop_frames        = false;
  config.enable_benchmark   = false;
  config.sequence_path      = "/home/nils/workspace_/projects/supereight_2/se_2/datasets/icl_nuim/traj_2/scene.raw";
  config.ground_truth_file  = "/home/nils/workspace_/projects/supereight_2/se_2/datasets/icl_nuim/traj_2/scene.raw.txt";

  // ========= READER INITIALIZATION  =========
  reader = se::create_reader(config);

  if (reader == nullptr) {
    exit(EXIT_FAILURE);
  }

  // Setup input, processed and output imgs
  se::ReaderStatus read_ok = se::ReaderStatus::ok;
  Eigen::Matrix4f T_MS;
  Eigen::Vector2i input_img_res(640, 480);
  static se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  static se::Image<uint32_t> input_rgba_img(input_img_res.x(), input_img_res.y());

  Eigen::Vector2i processed_img_res = input_img_res / ds;
  static se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());
  static se::Image<uint32_t>    processed_rgba_img(processed_img_res.x(), processed_img_res.y());

  static uint32_t* output_rgba_img_data     =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  static uint32_t* output_depth_img_data    =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  static uint32_t* output_tracking_img_data =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  static uint32_t* output_volume_img_data   =  new uint32_t[processed_img_res.x() * processed_img_res.y()];

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