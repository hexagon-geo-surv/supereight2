/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/supereight.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "montage.hpp"
#include "reader.hpp"
#include "se/common/filesystem.hpp"

int main(int argc, char** argv)
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
            return 2;
        }

        // ========= Config & I/O INITIALIZATION  =========
        const std::string config_filename = argv[1];
        const se::Config<se::OccupancyDataConfig, se::LeicaLidarConfig> config(config_filename);
        std::cout << config;

        // Create the mesh output directory
        if (!config.app.mesh_path.empty()) {
            stdfs::create_directories(config.app.mesh_path);
        }
        if (!config.app.slice_path.empty()) {
            stdfs::create_directories(config.app.slice_path);
        }
        if (!config.app.structure_path.empty()) {
            stdfs::create_directories(config.app.structure_path);
        }

        // Setup log stream
        std::ofstream log_file_stream;
        log_file_stream.open(config.app.log_file);
        se::perfstats.setFilestream(&log_file_stream);

        // Setup input ray / pose batch
        std::vector<std::pair<Eigen::Matrix4f, Eigen::Vector3f>,
                    Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, Eigen::Vector3f>>>
            ray_pose_batch;

        // ========= Map INITIALIZATION  =========
        // Setup the single-res TSDF map w/ default block size of 8 voxels
        // Custom way of setting up the same map:
        // se::Map<se::Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off>, se::Res::Single, 8>
        // See end of map.hpp and data.hpp for more details
        se::OccupancyMap<se::Res::Multi> map(config.map, config.data);

        // ========= Sensor INITIALIZATION  =========
        // Create a pinhole camera and downsample the intrinsics
        // Supported sensor models {se::PinholeCamera, se::OusterLidar}
        const se::LeicaLidar sensor(config.sensor, config.app.sensor_downsampling_factor);

        // ========= READER INITIALIZATION  =========
        std::unique_ptr<se::Reader> reader(se::create_reader(config.reader));
        if (!reader) {
            return EXIT_FAILURE;
        }

        // Setup input, processed and output imgs
        Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity(); //< Body to world transformation
        Eigen::Matrix4f T_BS = sensor.T_BS;                 //< Sensor to body transformation
        Eigen::Matrix4f T_WS = T_WB * T_BS;                 //< Sensor to world transformation

        // ========= Tracker & Pose INITIALIZATION  =========
        se::Tracker tracker(map, sensor, config.tracker);

        // ========= Integrator INITIALIZATION  =========
        // The integrator uses a field dependent allocation (TSDF: ray-casting; occupancy: volume-carving)
        // and updating method
        se::MapIntegrator integrator(map);

        int frame = 0;
        while (frame != config.app.max_frames) { // ToDo: check does not make sense
            se::perfstats.setIter(frame++);

            TICK("total")

            TICK("read")
            se::ReaderStatus read_ok = se::ReaderStatus::ok;
            ray_pose_batch.clear();
            read_ok = reader->nextData(config.reader.scan_time_interval, ray_pose_batch);
            if (read_ok != se::ReaderStatus::ok) {
                break;
            }
            for (size_t i = 0; i < ray_pose_batch.size(); i++) {
                ray_pose_batch[i].first = ray_pose_batch[i].first * T_BS;
            }
            TOCK("read")

            // Integrate depth for a given sensor, depth image, pose and frame number
            TICK("integration")
            integrator.integrateRayBatch(sensor, ray_pose_batch, frame);
            TOCK("integration")
            if (config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0) {
                if (!config.app.structure_path.empty()) {
                    map.saveStructure(config.app.structure_path + "/struct_" + std::to_string(frame)
                                      + ".ply");
                }
                if (!config.app.slice_path.empty()) {
                    map.saveFieldSlices(
                        config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                        config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                        config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                        Eigen::Vector3f(-4.65, -3.2, -0.7));
                }
                if (!config.app.mesh_path.empty()) {
                    map.saveMesh(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
                    map.saveMeshVoxel(config.app.mesh_path + "/voxel_mesh_" + std::to_string(frame)
                                      + ".ply");
                }
            }

            TOCK("total")

            se::perfstats.writeToFilestream();
        }

        // Final Meshes / Slices
        if (!config.app.structure_path.empty()) {
            map.saveStructure(config.app.structure_path + "/struct_" + std::to_string(frame)
                              + ".ply");
        }
        if (!config.app.mesh_path.empty()) {
            map.saveMeshVoxel(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
        }
        if (!config.app.slice_path.empty()) {
            map.saveFieldSlices(
                config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                Eigen::Vector3f(-4.65, -3.2, -0.7));
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
