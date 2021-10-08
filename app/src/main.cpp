/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/supereight.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "reader.hpp"
#include "se/common/filesystem.hpp"

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
        exit(2);
    }

    // ========= Config & I/O INITIALIZATION  =========
    const std::string config_filename = argv[1];
    const se::Config<se::TSDFDataConfig, se::PinholeCameraConfig> config(config_filename);
    std::cout << config;

    // Create the mesh output directory
    if (config.app.enable_meshing) {
        if (!config.app.mesh_output_dir.empty()) {
            stdfs::create_directories(config.app.mesh_output_dir);
        }
    }

    // Setup log stream
    std::ofstream log_file_stream;
    log_file_stream.open(config.app.log_file);
    se::perfstats.setFilestream(&log_file_stream);

    // Setup input images
    const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
    se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
    se::Image<uint32_t> input_rgba_img(input_img_res.x(), input_img_res.y());

    // Setup processed images
    const Eigen::Vector2i processed_img_res = input_img_res / config.app.sensor_downsampling_factor;
    se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());
    se::Image<uint32_t> processed_rgba_img(processed_img_res.x(), processed_img_res.y());

    // Setup output images / renders
    uint32_t* output_rgba_img_data = new uint32_t[processed_img_res.x() * processed_img_res.y()];
    uint32_t* output_depth_img_data = new uint32_t[processed_img_res.x() * processed_img_res.y()];
    uint32_t* output_tracking_img_data =
        new uint32_t[processed_img_res.x() * processed_img_res.y()];
    uint32_t* output_volume_img_data = new uint32_t[processed_img_res.x() * processed_img_res.y()];

    // ========= Map INITIALIZATION  =========
    // Setup the single-res TSDF map w/ default block size of 8 voxels
    // Custom way of setting up the same map:
    // se::Map<se::Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off>, se::Res::Single, 8>
    // See end of map.hpp and data.hpp for more details
    se::TSDFMap<se::Res::Single> map(config.map, config.data);

    // ========= Sensor INITIALIZATION  =========
    // Create a pinhole camera and downsample the intrinsics
    // Supported sensor models {se::PinholeCamera, se::OusterLidar}
    const se::PinholeCamera sensor(config.sensor, config.app.sensor_downsampling_factor);

    // ========= READER INITIALIZATION  =========
    se::Reader* reader = nullptr;
    reader = se::create_reader(config.reader);

    if (reader == nullptr) {
        exit(EXIT_FAILURE);
    }

    // Setup input, processed and output imgs
    se::ReaderStatus read_ok = se::ReaderStatus::ok;
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

    // Setup surface pointcloud, normals and scale
    se::Image<Eigen::Vector3f> surface_point_cloud_W(processed_img_res.x(), processed_img_res.y());
    se::Image<Eigen::Vector3f> surface_normals_W(processed_img_res.x(), processed_img_res.y());
    se::Image<int8_t> surface_scale(processed_img_res.x(), processed_img_res.y());

    while (read_ok == se::ReaderStatus::ok) {
        se::perfstats.setIter(frame++);

        TICK("total")

        TICK("read")
        if (config.app.enable_ground_truth || frame == 1) {
            read_ok = reader->nextData(input_depth_img, input_rgba_img, T_WB);
            T_WS = T_WB * T_BS;
        }
        else {
            read_ok = reader->nextData(input_depth_img, input_rgba_img);
        }
        TOCK("read")

        // Preprocess depth
        TICK("ds-depth")
        se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
        TOCK("ds-depth")
        TICK("ds-rgba")
        se::preprocessor::downsample_rgba(input_rgba_img, processed_rgba_img);
        TOCK("ds-rgba")

        // Track pose (if enabled)
        // Initial pose (frame == 0) is initialised with the identity matrix
        TICK("tracking")
        if (!config.app.enable_ground_truth && frame > 1
            && (frame % config.app.tracking_rate == 0)) {
            tracker.track(processed_depth_img, T_WS, surface_point_cloud_W, surface_normals_W);
        }
        se::perfstats.sampleT_WB(T_WB);
        TOCK("tracking")

        // Integrate depth for a given sensor, depth image, pose and frame number
        TICK("integration")
        if (frame % config.app.integration_rate == 0) {
            integrator.integrateDepth(sensor, processed_depth_img, T_WS, frame);
        }
        TOCK("integration")

        // Raycast from T_MS
        TICK("raycast")
        if (config.app.enable_rendering || !config.app.enable_ground_truth) {
          se::raycaster::raycastVolume(
              map, surface_point_cloud_W, surface_normals_W, surface_scale, T_WS, sensor);
        }
        TOCK("raycast")

        // Convert rgba, depth and render the volume (if enabled)
        // The volume is only rendered at the set rendering rate
        TICK("render")
        if (config.app.enable_rendering) {
            const Eigen::Vector3f ambient{0.1, 0.1, 0.1};
            convert_to_output_rgba_img(processed_rgba_img, output_rgba_img_data);
            convert_to_output_depth_img(
                processed_depth_img, sensor.near_plane, sensor.far_plane, output_depth_img_data);
            tracker.renderTrackingResult(output_tracking_img_data);
            if (frame % config.app.rendering_rate == 0) {
                se::raycaster::renderVolumeKernel(output_volume_img_data,
                                                  processed_img_res,
                                                  se::math::to_translation(T_WS),
                                                  ambient,
                                                  surface_point_cloud_W,
                                                  surface_normals_W,
                                                  surface_scale);
            }
        }
        TOCK("render")

        // Visualise rgba, depth, tracking data and the volume render (if enabled)
        TICK("draw")
        if (config.app.enable_gui) {
            drawthem(output_rgba_img_data,
                     processed_img_res,
                     output_depth_img_data,
                     processed_img_res,
                     output_tracking_img_data,
                     processed_img_res,
                     output_volume_img_data,
                     processed_img_res);
        }
        TOCK("draw")

        // Save logs, mesh, slices and struct (if enabled)
        TOCK("total")
        const bool last_frame =
            frame == config.app.max_frames || static_cast<size_t>(frame) == reader->numFrames();
        if (config.app.enable_meshing
            && ((config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0)
                || last_frame)) {
            map.saveMesh(config.app.mesh_output_dir + "/mesh_" + std::to_string(frame) + ".ply");
            if (config.app.enable_slice_meshing) {
                map.saveFieldSlice(config.app.mesh_output_dir + "/slice",
                                   se::math::to_translation(T_WS),
                                   std::to_string(frame));
            }
            if (config.app.enable_structure_meshing) {
                map.saveStructure(config.app.mesh_output_dir + "/struct_" + std::to_string(frame)
                                  + ".ply");
            }
        }

        se::perfstats.writeToFilestream();

        if (frame == config.app.max_frames) {
            break;
        }
    }

    // Free memory
    delete[] output_rgba_img_data;
    delete[] output_depth_img_data;
    delete[] output_tracking_img_data;
    delete[] output_volume_img_data;

    return 0;
}
