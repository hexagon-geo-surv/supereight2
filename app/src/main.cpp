#include <se/supereight.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "se/common/filesystem.hpp"
#include "reader.hpp"

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
    exit(2);
  }

  // Read the configuration
  const std::string config_filename = argv[1];
  const se::Config<se::TSDFDataConfig, se::PinholeCameraConfig> config(config_filename);
  std::cout << config;

  // Create the mesh output directory
  if (config.app.enable_meshing)
  {
    if (!config.app.mesh_output_dir.empty())
    {
      stdfs::create_directories(config.app.mesh_output_dir);
    }
  }

  // Setup log stream
  std::ofstream log_file_stream;
  log_file_stream.open(config.app.log_file);
  se::perfstats.setFilestream(&log_file_stream);

  // Setup the map
  se::TSDFMap<se::Res::Single> map(config.map, config.data);

  // Setup input images
  const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
  se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  se::Image<uint32_t>    input_rgba_img(input_img_res.x(), input_img_res.y());

  // Setup processed images
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());
  se::Image<uint32_t>    processed_rgba_img(processed_img_res.x(), processed_img_res.y());

  // Setup output images / renders
  uint32_t* output_rgba_img_data     =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  uint32_t* output_depth_img_data    =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  uint32_t* output_tracking_img_data =  new uint32_t[processed_img_res.x() * processed_img_res.y()];
  uint32_t* output_volume_img_data   =  new uint32_t[processed_img_res.x() * processed_img_res.y()];

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  // ========= READER INITIALIZATION  =========
  se::Reader* reader = nullptr;
  reader = se::create_reader(config.reader);

  if (reader == nullptr)
  {
    exit(EXIT_FAILURE);
  }

  // Setup input, processed and output imgs
  se::ReaderStatus read_ok = se::ReaderStatus::ok;
  Eigen::Matrix4f T_MS;

  se::Tracker tracker(map, sensor, config.tracker);

  // Integrated depth at given pose
  se::MapIntegrator integrator(map);
  int frame = 0;

  se::Image<Eigen::Vector3f> surface_point_cloud_M(processed_img_res.x(), processed_img_res.y());
  se::Image<Eigen::Vector3f> surface_normals_M(processed_img_res.x(), processed_img_res.y());
  se::Image<int8_t>          surface_scale(processed_img_res.x(), processed_img_res.y());

  while (read_ok == se::ReaderStatus::ok)
  {
    se::perfstats.setIter(frame++);

    TICK("total")

    TICK("read")
    if (config.app.enable_ground_truth)
    {
      read_ok = reader->nextData(input_depth_img, input_rgba_img, T_MS);
    } else
    {
      if (frame == 1)
      {
        read_ok = reader->nextData(input_depth_img, input_rgba_img, T_MS);
      } else
      {
        read_ok = reader->nextData(input_depth_img, input_rgba_img);
      }
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
    if (!config.app.enable_ground_truth && frame > 1 && (frame % config.app.tracking_rate == 0))
    {
      tracker.track(processed_depth_img, T_MS, surface_point_cloud_M, surface_normals_M);
    }
    TOCK("tracking")

    TICK("integration")
    if (frame % config.app.integration_rate == 0)
    {
      integrator.integrateDepth(sensor, processed_depth_img, T_MS, frame);
    }
    TOCK("integration")

    TICK("raycast")
    se::raycaster::raycastVolume(map, surface_point_cloud_M, surface_normals_M, surface_scale, T_MS, sensor);
    TOCK("raycast")

    TICK("render")
    if (config.app.enable_rendering)
    {
      const Eigen::Vector3f ambient{0.1, 0.1, 0.1};
      convert_to_output_rgba_img(processed_rgba_img, output_rgba_img_data);
      convert_to_output_depth_img(processed_depth_img, sensor.near_plane, sensor.far_plane, output_depth_img_data);
      tracker.renderTrackingResult(output_tracking_img_data);
      if (frame % config.app.rendering_rate == 0)
      {
        se::raycaster::renderVolumeKernel(output_volume_img_data, processed_img_res, se::math::to_translation(T_MS), ambient, surface_point_cloud_M, surface_normals_M, surface_scale);
      }
    }
    TOCK("render")

    TICK("draw")
    if (config.app.enable_gui)
    {
      drawthem(output_rgba_img_data,     processed_img_res,
               output_depth_img_data,    processed_img_res,
               output_tracking_img_data, processed_img_res,
               output_volume_img_data,   processed_img_res);
    }
    TOCK("draw")

    TOCK("total")
    const bool last_frame = frame == config.app.max_frames || static_cast<size_t>(frame) == reader->numFrames();
    if (config.app.enable_meshing && ((config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0) || last_frame))
    {
      map.saveMesh(config.app.mesh_output_dir + "/mesh", std::to_string(frame));
      if (config.app.enable_slice_meshing)
      {
        map.saveFieldSlice(config.app.mesh_output_dir + "/slice", se::math::to_translation(T_MS), std::to_string(frame));
      }
      if (config.app.enable_structure_meshing)
      {
        map.saveStrucutre(config.app.mesh_output_dir + "/struct", std::to_string(frame));
      }
    }

    se::perfstats.writeToFilestream();

    if (frame == config.app.max_frames)
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
