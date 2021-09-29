#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "config.hpp"
#include "se/map/map.hpp"
#include "se/integrator/map_integrator.hpp"
#include <se/common/filesystem.hpp>

extern int my_argc;
extern char** my_argv;

int my_argc;
char** my_argv;
std::string tmp;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  my_argc = argc;
  my_argv = argv;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
    exit(2);
  }

  // Create a temporary directory for all the tests.
  tmp = stdfs::temp_directory_path() / stdfs::path("supereight_test_results");
  stdfs::create_directories(tmp);

  return RUN_ALL_TESTS();
}



TEST(MultiResOFusionSystemTest, GetFieldInterpolation)
{
  const std::string config_filename (my_argv[1]);
  se::Config<se::OccupancyDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

  // Output files in a temporary directory.
  config.app.mesh_output_dir = std::string(tmp + "/meshes");
  config.app.log_file = std::string(tmp + "/log.tsv");
  stdfs::create_directories(config.app.mesh_output_dir);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
  se::Image<float>      input_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<float>      processed_depth_img(processed_img_res.x(), processed_img_res.y());

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
  map.saveStructure(config.app.mesh_output_dir + "/test-field-interp-structure_"
                    + std::to_string(max_frame) + ".ply");

  Eigen::Vector3f       point_M;
  std::optional<se::field_t> field_value;

  // Node neighbours different size
  const Eigen::Vector3i voxel_coord_free_1(639, 511, 799);
  map.voxelToPoint(voxel_coord_free_1, point_M);
  field_value = map.getFieldInterp(point_M);
  EXPECT_FLOAT_EQ(-5.015, *field_value);

  // All inside node
  const Eigen::Vector3i voxel_coord_free_2(600, 520, 811);
  map.voxelToPoint(voxel_coord_free_2, point_M);
  field_value = map.getFieldInterp(point_M);
  EXPECT_FLOAT_EQ(-5.015, *field_value);

  // Node and block neighbours
  const Eigen::Vector3i voxel_coord_free_3(575, 511, 615);
  map.voxelToPoint(voxel_coord_free_3, point_M);
  field_value = map.getFieldInterp(point_M);
  EXPECT_FLOAT_EQ(-5.015, *field_value);
}



TEST(MultiResOFusionSystemTest, GetField)
{
  const std::string config_filename (my_argv[1]);
  se::Config<se::OccupancyDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

  // Output files in a temporary directory.
  config.app.mesh_output_dir = std::string(tmp + "/meshes");
  config.app.log_file = std::string(tmp + "/log.tsv");
  stdfs::create_directories(config.app.mesh_output_dir);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
  se::Image<float>      input_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<float>      processed_depth_img(processed_img_res.x(), processed_img_res.y());

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

  se::OccupancyData data;

  map.saveFieldSlice(config.app.mesh_output_dir + "/test-field-slice",
                     se::math::to_translation(T_MS),
                     std::to_string(max_frame));

  map.saveStructure(config.app.mesh_output_dir + "/test-field-structure_"
                    + std::to_string(max_frame) + ".ply");

  map.voxelToPoint(voxel_coord_unknown_1, point_M);
  data = map.getData(point_M);
  EXPECT_EQ(se::OccupancyData().occupancy, data.occupancy);

  map.voxelToPoint(voxel_coord_unknown_2, point_M);
  data = map.getData(point_M);
  EXPECT_EQ(se::OccupancyData().occupancy, data.occupancy);

  map.voxelToPoint(voxel_coord_free_1, point_M);
  data = map.getData(point_M);
  EXPECT_FLOAT_EQ(-5.015, data.occupancy);

  map.voxelToPoint(voxel_coord_free_2, point_M);
  data = map.getData(point_M);
  EXPECT_FLOAT_EQ(-5.015, data.occupancy);
}



TEST(MultiResOFusionSystemTest, GetMaxField)
{
  const std::string config_filename (my_argv[1]);
  se::Config<se::OccupancyDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

  // Output files in a temporary directory.
  config.app.mesh_output_dir = std::string(tmp + "/meshes");
  config.app.log_file = std::string(tmp + "/log.tsv");
  stdfs::create_directories(config.app.mesh_output_dir);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
  se::Image<float>      input_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<float>      processed_depth_img(processed_img_res.x(), processed_img_res.y());

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

  se::OccupancyData data;

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

  map.saveStructure(config.app.mesh_output_dir + "/test-max-field-structure_"
                    + std::to_string(max_frame));
}



TEST(MultiResOFusionSystemTest, DeleteChildren)
{
  const std::string config_filename (my_argv[1]);
  se::Config<se::OccupancyDataConfig, se::PinholeCameraConfig> config (config_filename);
  se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

  // Output files in a temporary directory.
  config.app.mesh_output_dir = std::string(tmp + "/meshes");
  config.app.log_file = std::string(tmp + "/log.tsv");
  stdfs::create_directories(config.app.mesh_output_dir);

  // Create a pinhole camera and downsample the intrinsics
  const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

  const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
  se::Image<float>      input_depth_img(input_img_res.x(), input_img_res.y());
  se::Image<float>      input_noise_depth_img(input_img_res.x(), input_img_res.y());
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<float>      processed_depth_img(processed_img_res.x(), processed_img_res.y());

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

    se::OccupancyData data;

    map.voxelToPoint(voxel_coord, point_M);
    data = map.getData(point_M);

    map.saveFieldSlice(config.app.mesh_output_dir + "/test-delete-child-slice",
                       se::math::to_translation(T_MS),
                       std::to_string(frame));

    map.saveStructure(config.app.mesh_output_dir + "/test-delete-child-structure_"
                      + std::to_string(frame) + ".ply");
  }
}



TEST(MultiResOFusionSystemTest, Raycasting)
{
  // Read the configuration
  const std::string config_filename (my_argv[1]);
  se::Config<se::OccupancyDataConfig, se::PinholeCameraConfig> config (config_filename);
  std::cout << config;

  // Output files in a temporary directory.
  config.app.mesh_output_dir = std::string(tmp + "/meshes");
  config.app.log_file = std::string(tmp + "/log.tsv");
  stdfs::create_directories(config.app.mesh_output_dir);

  // Setup log stream
  std::ofstream log_file_stream;
  log_file_stream.open(config.app.log_file);
  se::perfstats.setFilestream(&log_file_stream);

  // Setup the map
  se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

  // Setup input images
  const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
  se::Image<float>      input_depth_img(input_img_res.x(), input_img_res.y());
  se::Image<uint32_t>   input_rgba_img(input_img_res.x(), input_img_res.y());

  // Setup processed images
  const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
  se::Image<float>      processed_depth_img(processed_img_res.x(), processed_img_res.y());
  se::Image<uint32_t>   processed_rgba_img(processed_img_res.x(), processed_img_res.y());

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

  map.saveStructure(config.app.mesh_output_dir + "/test-raycasting-structure_" + std::to_string(frame) + ".ply");
  map.saveStructure(config.app.mesh_output_dir + "/test-raycasting-structure_" + std::to_string(frame) + ".vtk");
  map.saveStructure(config.app.mesh_output_dir + "/test-raycasting-structure_" + std::to_string(frame) + ".obj");
  map.saveFieldSlice(config.app.mesh_output_dir + "/test-raycasting-slice-field", se::math::to_translation(T_MS), std::to_string(frame));
  map.saveMesh(config.app.mesh_output_dir + "/test-raycasting-mesh_" + std::to_string(frame) + ".ply");
  map.saveMesh(config.app.mesh_output_dir + "/test-raycasting-mesh_" + std::to_string(frame) + ".vtk");
  map.saveMesh(config.app.mesh_output_dir + "/test-raycasting-mesh_" + std::to_string(frame) + ".obj");

  cv::Mat depth_cv_image(processed_img_res.y(), processed_img_res.x(), CV_8UC4, output_volume_img_data);
  cv::cvtColor(depth_cv_image, depth_cv_image, cv::COLOR_RGBA2BGRA);
  cv::imwrite((config.app.mesh_output_dir + "/test-raycasting-img.png").c_str(), depth_cv_image);

  delete[] output_rgba_img_data;
  delete[] output_depth_img_data;
  delete[] output_tracking_img_data;
  delete[] output_volume_img_data;
}
