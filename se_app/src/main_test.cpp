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
#include "se/io/point_cloud_io.hpp"

#include "draw.h"
#include "reader.hpp"

static std::ostream* log_stream;
static std::ofstream log_file_stream;

int main()
{
  const Eigen::Vector3f map_dim(10.24f, 10.24f, 10.24f);
  const float map_res(0.04f);
  se::TSDFMap<se::Res::Single> map_tsdf(map_dim, map_res);

  se::SensorConfig sensor_config;
  Eigen::Vector2i img_res(640, 480);
  int ds = 8;

  sensor_config.width  =  img_res.x() / ds;
  sensor_config.height =  img_res.y() / ds;

  sensor_config.fx     =  480.0 / ds;
  sensor_config.fy     =  480.0 / ds;
  sensor_config.cx     =  (320.0 + 0.5f) / ds - 0.5f;
  sensor_config.cy     =  (240.0 + 0.5f) / ds - 0.5f;
  sensor_config.left_hand_frame = sensor_config.fy < 0;

  sensor_config.near_plane      = 0.4f;
  sensor_config.far_plane       = 6.f;

  const se::PinholeCamera sensor(sensor_config);

  Eigen::Matrix4f T_BS;
  T_BS << 0,                                        0, 1, 0,
         -1,                                        0, 0, 0,
          0, (sensor_config.left_hand_frame) ? 1 : -1, 0, 0,
          0,                                        0, 0, 1;

  std::cout << T_BS << std::endl;

  Eigen::Matrix4f T_MB = Eigen::Matrix4f::Identity();

//  Eigen::Matrix3f R_MB;
//  R_MB << 0.7071068, -0.7071068,  0.0000000,
//          0.7071068,  0.7071068,  0.0000000,
//          0.0000000,  0.0000000,  1.0000000;
//  T_MB.topLeftCorner<3,3>()  = R_MB;

  T_MB.topRightCorner<3,1>() = Eigen::Matrix<float,3,1>(0,0,0);

  Eigen::Matrix4f T_MS = T_MB * T_BS;

  Eigen::Vector2i input_img_res(img_res.x() / ds, img_res.y() / ds);
  static se::Image<se::depth_t> input_depth_img(input_img_res.x(), input_img_res.y());

  for (int x = 0; x < input_img_res.x(); x++)
  {
    for (int y = 0; y < input_img_res.y(); y++)
    {
      input_depth_img[x + input_img_res.x() * y] = 2.f;
    }
  }

  for (int x = input_img_res.x() / 2; x < input_img_res.x(); x++)
  {
    for (int y = input_img_res.y() / 3; y < input_img_res.y(); y++)
    {
      input_depth_img[x + input_img_res.x() * y] = 1.5f;
    }
  }

  // Integrated depth at given pose
  struct IntegrConfig {};
  IntegrConfig integr_config;
  se::MapIntegrator<se::TSDFMap<se::Res::Single>, IntegrConfig> integrator(map_tsdf, integr_config);

  TICK("integration")
  integrator.integrateDepth(input_depth_img, sensor, T_MS);
  TOCK("integration")

  static uint32_t* output_volume_img_data = new uint32_t[input_img_res.x() * input_img_res.y()];
  static uint32_t* output_dummy_img_data  = new uint32_t[input_img_res.x() * input_img_res.y()];

  map_tsdf.saveMesh(std::to_string(0));
  map_tsdf.saveStrucutre(std::to_string(0));
  map_tsdf.saveSlice(se::math::to_translation(T_MS), std::to_string(0));

  unsigned int iteration = 0;
  auto octree_ptr = map_tsdf.getOctree();
  typename se::TSDFMap<se::Res::Single>::DataType data;
  se::visitor::getData(octree_ptr, Eigen::Vector3i(85,75,122), data);

  // Render volume
  se::Image<Eigen::Vector3f> surface_point_cloud_M(input_img_res.x(), input_img_res.y());
  se::Image<Eigen::Vector3f> surface_normals_M(input_img_res.x(), input_img_res.y());
  se::raycaster::raycastVolume(map_tsdf, surface_point_cloud_M, surface_normals_M, T_MS, sensor);
  const Eigen::Vector3f ambient{ 0.1, 0.1, 0.1};
  se::raycaster::renderVolumeKernel(output_volume_img_data, input_img_res, se::math::to_translation(T_MS), ambient, surface_point_cloud_M, surface_normals_M);
  drawthem(output_volume_img_data, input_img_res,
           output_dummy_img_data, input_img_res,
           output_dummy_img_data, input_img_res,
           output_dummy_img_data, input_img_res);

  static std::ofstream surface_steam;
  static std::ofstream normal_steam;
  if (sensor_config.left_hand_frame)
  {
    save_point_cloud_vtk(surface_point_cloud_M, "/home/nils/workspace_/projects/supereight-2-srl/se_map/test/out/point-cloud-left.vtk", Eigen::Matrix4f::Identity());
    surface_steam.open("/home/nils/workspace_/projects/supereight-2-srl/se_map/test/out/surface-left.txt");
    normal_steam.open("/home/nils/workspace_/projects/supereight-2-srl/se_map/test/out/normal-left.txt");
  } else
  {
    save_point_cloud_vtk(surface_point_cloud_M, "/home/nils/workspace_/projects/supereight-2-srl/se_map/test/out/point-cloud-right.vtk", Eigen::Matrix4f::Identity());
    surface_steam.open("/home/nils/workspace_/projects/supereight-2-srl/se_map/test/out/surface-right.txt");
    normal_steam.open("/home/nils/workspace_/projects/supereight-2-srl/se_map/test/out/normal-right.txt");
  }

  for (int y = 0; y < surface_point_cloud_M.height(); y++)
  {
    for (int x = 0; x < surface_point_cloud_M.width(); x++)
    {
      surface_steam << surface_point_cloud_M[x + y * surface_point_cloud_M.width()].x() + 5.12 << ", " << surface_point_cloud_M[x + y * surface_point_cloud_M.width()].y() + 5.12 << ", " << surface_point_cloud_M[x + y * surface_point_cloud_M.width()].z() + 5.12 << std::endl;
      normal_steam << surface_normals_M[x + y * surface_point_cloud_M.width()].x() << ", " << surface_normals_M[x + y * surface_point_cloud_M.width()].y() << ", " << surface_normals_M[x + y * surface_point_cloud_M.width()].z() << std::endl;
    }
  }
  surface_steam.close();
  normal_steam.close();


  while (true)
  {
    // Render volume
    se::raycaster::raycastVolume(map_tsdf, surface_point_cloud_M, surface_normals_M, T_MS, sensor);
    se::raycaster::renderVolumeKernel(output_volume_img_data, input_img_res, se::math::to_translation(T_MS), ambient, surface_point_cloud_M, surface_normals_M);
    drawthem(output_volume_img_data, input_img_res,
             output_dummy_img_data, input_img_res,
             output_dummy_img_data, input_img_res,
             output_dummy_img_data, input_img_res);
  }
}
