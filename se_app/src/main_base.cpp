#include "se/sensor.hpp"
#include "se/octree.hpp"
#include "se/map.hpp"
#include "se/utils/type_util.hpp"

#include "draw.h"
#include "reader.hpp"


int main() {
  // Creating a single-res TSDF map with colour and 4 voxel block size
  const Eigen::Vector3f map_dim(10.f, 10.f, 3.f);
  const float map_res(0.1f);

  /// CASE 1 Custom

  typedef se::Data<se::Field::TSDF, se::Colour::On, se::Semantics::On> DataT;    // Create custom data
  se::Map<DataT, se::Res::Single, 4> map_tsdf_col_sem(map_dim, map_res); // Custom Map (same as TSDFColSemMap)

  // Set custom data
  DataT data_c;
  data_c.tsdf   = 3.f;
  data_c.weight = 2;
  data_c.rgba   = 0xFF0000FF;
  data_c.sem    = 3;

  /// CASE 2 Default

  se::Map<> map_tsdf(map_dim, map_res);                             // Default Map (TSDF, Colour::Off, Semantics::Off, SingleRes)
  se::Data data_t;
  data_t.tsdf   = 4.f;
  data_t.weight = 5;

  /// CASE 3 Occupancy

  // Setup map
  se::MapConfig map_config;                                         // Sets default values
                                                                // (10x10x3 m^3, 0.1 m/voxel, centred origin)
  map_config.origin = Eigen::Vector3f(5.f, 5.f, 0.5f);          // Set custom origin
  se::OccMap<se::Res::Multi> map_occ(map_config);                       // Initialise Occupancy Map

  const Eigen::Vector3f point_M = Eigen::Vector3f(2, 2, 2);     // Create point at x = 2m, y = 2m, z = 2m
  se::SimpleIntegrator simple_integrator;                           // Create a simple map integrator

  // Set data
  se::OccData data_s;                                               // Data<Field::Occupancy> would create the same type
  data_s.occupancy = 0.5f;
  data_s.timestamp = 0.3;

  simple_integrator.setData(map_occ, data_s, point_M);           // Set data (allocates data if not allocated yet)

  se::OccData data_g;
  if (map_occ.getData(Eigen::Vector3f(0,0,0), data_g)) {
    std::cout << "Data is available" << std::endl;
  } else {
    std::cout << "Data is NOT available" << std::endl;
  }

  if (map_occ.getData(point_M, data_g)) {
    std::cout << "Data is available" << std::endl;
    if (data_s.occupancy == data_g.occupancy &&
        data_s.timestamp == data_s.timestamp) {
      std::cout << "Get and set data are equivalent" << std::endl;
    } else {
      std::cout << "Get and set data are NOT equivalent" << std::endl;
    }
  } else {
    std::cout << "Data is NOT available" << std::endl;
  }

//  se::SensorIntegrator integrator(sensor);
//  Eigen::Matrix4f T_WS = Eigen::Matrix4f::Identity();
//  integrator.integrateDepth(tsdf_map_m1, depth_image, T_WS);

//  se::SimpleIntegrator simple

  return 0;
}
