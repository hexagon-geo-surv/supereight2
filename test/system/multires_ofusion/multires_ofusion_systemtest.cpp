#include <gtest/gtest.h>

#include "config.hpp"
#include "lodepng.h"
#include "se/map/map.hpp"
#include "se/integrator/map_integrator.hpp"

extern int my_argc;
extern char** my_argv;

int my_argc;
char** my_argv;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  my_argc = argc;
  my_argv = argv;
  return RUN_ALL_TESTS();
}



TEST(MultiResOFusionSystemTest, GetFieldInterpolation)
{
  const std::string config_filename = "/home/nils/workspace_/projects/supereight-2-srl-test/se_app/test/multires_ofusion/config.yaml";
  const se::Config<se::OccDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccMap<se::Res::Multi> map(config.map, config.data);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i  input_img_res(config.sensor.width, config.sensor.height);
  se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());

  // Set pose to identity
  Eigen::Matrix4f T_MS = Eigen::Matrix4f::Identity();

  // Set depth image
  for (int y = 0; y < input_img_res.y(); y++)
  {
    for (int x = 0; x < input_img_res.x(); x++)
    {
      input_depth_img(x, y) = 4.f;
    }
  }

  se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);

  se::MapIntegrator integrator(map);

  const int max_frame = 1;
  for (int frame = 0; frame < max_frame; frame++)
  {
    integrator.integrateDepth(sensor, processed_depth_img, T_MS, frame);
  }

  map.saveFieldSlice(config.app.mesh_output_dir + "/test-field-interp-slice",
                     se::math::to_translation(T_MS),
                     std::to_string(max_frame));
  map.saveStrucutre(config.app.mesh_output_dir + "/test-field-interp-structure",
                    std::to_string(max_frame));

  Eigen::Vector3f       point_M;
  std::optional<se::field_t> field_value;

  // Node neighbours different size
  const Eigen::Vector3i voxel_coord_free_1(639.5f, 511.5f, 799.5f);
  map.voxelToPoint(voxel_coord_free_1, point_M);
  field_value = map.getFieldInterp(point_M);
  EXPECT_FLOAT_EQ(-5.015, *field_value);

  // All inside node
  const Eigen::Vector3i voxel_coord_free_2(600.5f, 520.5f, 811.5f);
  map.voxelToPoint(voxel_coord_free_2, point_M);
  field_value = map.getFieldInterp(point_M);
  EXPECT_FLOAT_EQ(-5.015, *field_value);

  // Node and block neighbours
  const Eigen::Vector3i voxel_coord_free_3(575.5f, 511.5f, 615.5f);
  map.voxelToPoint(voxel_coord_free_3, point_M);
  field_value = map.getFieldInterp(point_M);
  EXPECT_FLOAT_EQ(-5.015, *field_value);
}



TEST(MultiResOFusionSystemTest, GetField)
{
  const std::string config_filename = "/home/nils/workspace_/projects/supereight-2-srl-test/se_app/test/multires_ofusion/config.yaml";
  const se::Config<se::OccDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccMap<se::Res::Multi> map(config.map, config.data);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i  input_img_res(config.sensor.width, config.sensor.height);
  se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());

  // Set pose to identity
  Eigen::Matrix4f T_MS = Eigen::Matrix4f::Identity();

  // Set depth image
  for (int y = 0; y < input_img_res.y(); y++)
  {
    for (int x = 0; x < input_img_res.x(); x++)
    {
      input_depth_img(x, y) = (x < input_img_res.x() / 2) ? 2.f : 4.f;
    }
  }

  se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);

  se::MapIntegrator integrator(map);

  const int max_frame = 1;
  for (int frame = 0; frame < max_frame; frame++)
  {
    integrator.integrateDepth(sensor, processed_depth_img, T_MS, frame);
  }

  const Eigen::Vector3i voxel_coord_unknown_1(688, 500, 933);
  const Eigen::Vector3i voxel_coord_unknown_2(256, 166, 338);
  const Eigen::Vector3i voxel_coord_free_1(578, 500, 737);
  const Eigen::Vector3i voxel_coord_free_2(477, 500, 618);
  Eigen::Vector3f       point_M;

  se::OccData data;

  map.saveFieldSlice(config.app.mesh_output_dir + "/test-field-slice",
                     se::math::to_translation(T_MS),
                     std::to_string(max_frame));

  map.saveStrucutre(config.app.mesh_output_dir + "/test-field-structure",
                    std::to_string(max_frame));

  map.voxelToPoint(voxel_coord_unknown_1, point_M);
  data = map.getData(point_M);
  EXPECT_EQ(se::OccData().occupancy, data.occupancy);

  map.voxelToPoint(voxel_coord_unknown_2, point_M);
  data = map.getData(point_M);
  EXPECT_EQ(se::OccData().occupancy, data.occupancy);

  map.voxelToPoint(voxel_coord_free_1, point_M);
  data = map.getData(point_M);
  EXPECT_FLOAT_EQ(-5.015, data.occupancy);

  map.voxelToPoint(voxel_coord_free_2, point_M);
  data = map.getData(point_M);
  EXPECT_FLOAT_EQ(-5.015, data.occupancy);
}



TEST(MultiResOFusionSystemTest, GetMaxField)
{
  const std::string config_filename = "/home/nils/workspace_/projects/supereight-2-srl-test/se_app/test/multires_ofusion/config.yaml";
  const se::Config<se::OccDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccMap<se::Res::Multi> map(config.map, config.data);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i  input_img_res(config.sensor.width, config.sensor.height);
  se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());

  // Set pose to identity
  Eigen::Matrix4f T_MS = Eigen::Matrix4f::Identity();

  // Set depth image
  for (int y = 0; y < input_img_res.y(); y++)
  {
    for (int x = 0; x < input_img_res.x(); x++)
    {
      input_depth_img(x, y) = (x < input_img_res.x() / 2) ? 2.f : 4.f;
    }
  }

  se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);

  se::MapIntegrator integrator(map);

  const int max_frame = 1;
  for (int frame = 0; frame < max_frame; frame++)
  {
    integrator.integrateDepth(sensor, processed_depth_img, T_MS, frame);
  }

  const Eigen::Vector3i voxel_coord(640, 512, 896);
  const int             scale_5 = 5;
  Eigen::Vector3f       point_M;

  se::OccData data;

  map.voxelToPoint(voxel_coord, point_M);
  data = map.getMaxData(point_M, scale_5);

  map.saveFieldSlice(config.app.mesh_output_dir + "/test-max-field-slice-field",
                     se::math::to_translation(T_MS),
                     std::to_string(max_frame));

  map.saveScaleSlice(config.app.mesh_output_dir + "/test-max-field-slice-scale",
                     se::math::to_translation(T_MS),
                     std::to_string(max_frame));




  for (int scale = 0; scale < map.getOctree()->getMaxScale(); scale++)
  {
    map.saveMaxFieldSlice(config.app.mesh_output_dir + "/test-max-field-slice-max-field-scale-" + std::to_string(scale),
                          se::math::to_translation(T_MS),
                          scale,
                          std::to_string(max_frame));
  }

  map.saveStrucutre(config.app.mesh_output_dir + "/test-max-field-structure",
                    std::to_string(max_frame));
}



TEST(MultiResOFusionSystemTest, DeleteChildren)
{
  const std::string config_filename = "/home/nils/workspace_/projects/supereight-2-srl-test/se_app/test/multires_ofusion/config.yaml";
  const se::Config<se::OccDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccMap<se::Res::Multi> map(config.map, config.data);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i  input_img_res(config.sensor.width, config.sensor.height);
  se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());
  se::Image<se::depth_t> input_noise_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<se::depth_t> processed_depth_img(processed_img_res.x(), processed_img_res.y());

  // Set pose to identity
  Eigen::Matrix4f T_MS = Eigen::Matrix4f::Identity();

  // Set depth image
  for (int y = 0; y < input_img_res.y(); y++)
  {
    for (int x = 0; x < input_img_res.x(); x++)
    {
      input_noise_depth_img(x, y) = (x < input_img_res.x() * 3 / 8 || x > input_img_res.x() * 5 / 8) ? 2.f : 4.f;
    }
  }

  for (int y = 0; y < input_img_res.y(); y++)
  {
    for (int x = 0; x < input_img_res.x(); x++)
    {
      input_depth_img(x, y) = 4.f;
    }
  }

  se::MapIntegrator integrator(map);

  const int max_frame = 15;
  for (int frame = 0; frame < max_frame; frame++)
  {
    std::cout << "FRAME = " << frame << std::endl;
    se::preprocessor::downsample_depth((frame == 0) ? input_noise_depth_img : input_depth_img, processed_depth_img);
    integrator.integrateDepth(sensor, processed_depth_img, T_MS, frame);

    const Eigen::Vector3i voxel_coord(471, 512, 807);
    Eigen::Vector3f       point_M;

    se::OccData data;

    map.voxelToPoint(voxel_coord, point_M);
    data = map.getData(point_M);

    map.saveFieldSlice(config.app.mesh_output_dir + "/test-delete-child-slice",
                       se::math::to_translation(T_MS),
                       std::to_string(frame));

    map.saveStrucutre(config.app.mesh_output_dir + "/test-delete-child-structure",
                      std::to_string(frame));
  }
}



TEST(MultiResOFusionSystemTest, Raycasting)
{
  // Read the configuration
  const std::string config_filename = "/home/nils/workspace_/projects/supereight-2-srl-test/se_app/test/multires_ofusion/config.yaml";
  const se::Config<se::OccDataConfig, se::PinholeCameraConfig> config (config_filename);
  std::cout << config;

  // Setup log stream
  std::ofstream log_file_stream;
  log_file_stream.open(config.app.log_file);
  se::perfstats.setFilestream(&log_file_stream);

  // Setup the map
  se::OccMap<se::Res::Multi> map(config.map, config.data);

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

  // Integrated depth at given pose

  int frame = 0;

  se::Image<Eigen::Vector3f> surface_point_cloud_M(processed_img_res.x(), processed_img_res.y());
  se::Image<Eigen::Vector3f> surface_normals_M(processed_img_res.x(), processed_img_res.y());
  se::Image<int8_t>          surface_scale(processed_img_res.x(), processed_img_res.y());

  Eigen::Matrix4f T_MS;
  reader->nextData(input_depth_img, input_rgba_img, T_MS);

  // Preprocess depth
  se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
  se::preprocessor::downsample_rgba(input_rgba_img,  processed_rgba_img);

  se::MapIntegrator integrator(map);
  integrator.integrateDepth(sensor, processed_depth_img, T_MS, frame);

  se::raycaster::raycastVolume(map, surface_point_cloud_M, surface_normals_M, surface_scale, T_MS, sensor);

  const Eigen::Vector3f ambient{0.1, 0.1, 0.1};
  convert_to_output_rgba_img(processed_rgba_img, output_rgba_img_data);
  convert_to_output_depth_img(processed_depth_img, output_depth_img_data);
  se::raycaster::renderVolumeKernel(output_volume_img_data, processed_img_res, se::math::to_translation(T_MS), ambient, surface_point_cloud_M, surface_normals_M, surface_scale);

  map.saveStrucutre(config.app.mesh_output_dir + "/test-raycasting-structure", std::to_string(frame));
  map.saveFieldSlice(config.app.mesh_output_dir + "/test-raycasting-slice-field", se::math::to_translation(T_MS), std::to_string(frame));
  map.saveMesh(config.app.mesh_output_dir + "/test-raycasting-mesh", std::to_string(frame));

  lodepng_encode32_file((config.app.mesh_output_dir + "/test-raycasting-img.png").c_str(), reinterpret_cast<const unsigned char*>(output_volume_img_data), processed_img_res.x(), processed_img_res.y());

  delete[] output_rgba_img_data;
  delete[] output_depth_img_data;
  delete[] output_tracking_img_data;
  delete[] output_volume_img_data;
}
