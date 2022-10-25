/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/supereight.hpp>

#include "config.hpp"
#include "draw.hpp"
#include "montage.hpp"
#include "reader.hpp"
#include "se/common/filesystem.hpp"
#include "se/common/system_utils.hpp"

int main(int argc, char** argv)
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
            return 2;
        }

        // ========= Config & I/O INITIALIZATION  =========
        const std::string config_filename = argv[1];
        const se::Config<se::TSDFDataConfig, se::PinholeCameraConfig> config(config_filename);
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

        // Setup input images
        const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
        se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
        se::Image<uint32_t> input_colour_img(input_img_res.x(), input_img_res.y());

        // Setup processed images
        const Eigen::Vector2i processed_img_res =
            input_img_res / config.app.sensor_downsampling_factor;
        se::Image<float> processed_depth_img(processed_img_res.x(), processed_img_res.y());
        se::Image<uint32_t> processed_colour_img(processed_img_res.x(), processed_img_res.y());

        // Setup output images / renders
        std::unique_ptr<uint32_t[]> output_rgba_img_data(
            new uint32_t[processed_img_res.x() * processed_img_res.y()]);
        std::unique_ptr<uint32_t[]> output_depth_img_data(
            new uint32_t[processed_img_res.x() * processed_img_res.y()]);
        std::unique_ptr<uint32_t[]> output_tracking_img_data(
            new uint32_t[processed_img_res.x() * processed_img_res.y()]);
        std::unique_ptr<uint32_t[]> output_volume_img_data(
            new uint32_t[processed_img_res.x() * processed_img_res.y()]);

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

        // Setup surface pointcloud, normals and scale
        se::Image<Eigen::Vector3f> surface_point_cloud_W(processed_img_res.x(),
                                                         processed_img_res.y());
        se::Image<Eigen::Vector3f> surface_normals_W(processed_img_res.x(), processed_img_res.y());
        se::Image<int8_t> surface_scale(processed_img_res.x(), processed_img_res.y());

        int frame = 0;
        while (frame != config.app.max_frames) {
            se::perfstats.setIter(frame++);

            TICK("total")

            TICK("read")
            se::ReaderStatus read_ok = se::ReaderStatus::ok;
            if (config.app.enable_ground_truth || frame == 1) {
                read_ok = reader->nextData(input_depth_img, input_colour_img, T_WB);
                T_WS = T_WB * T_BS;
            }
            else {
                read_ok = reader->nextData(input_depth_img, input_colour_img);
            }
            if (read_ok != se::ReaderStatus::ok) {
                break;
            }
            TOCK("read")

            // Preprocess depth
            TICK("ds-depth")
            se::preprocessor::downsample_depth(input_depth_img, processed_depth_img);
            TOCK("ds-depth")
            TICK("ds-colour")
            se::preprocessor::downsample_colour(input_colour_img, processed_colour_img);
            TOCK("ds-colour")

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
                se::raycaster::raycast_volume(
                    map, surface_point_cloud_W, surface_normals_W, surface_scale, T_WS, sensor);
            }
            TOCK("raycast")

            // Convert rgba, depth and render the volume (if enabled)
            // The volume is only rendered at the set rendering rate
            TICK("render")
            if (config.app.enable_rendering) {
                const Eigen::Vector3f ambient{0.1, 0.1, 0.1};
                convert_to_output_colour_img(processed_colour_img, output_rgba_img_data.get());
                convert_to_output_depth_img(processed_depth_img,
                                            sensor.near_plane,
                                            sensor.far_plane,
                                            output_depth_img_data.get());
                tracker.renderTrackingResult(output_tracking_img_data.get());
                if (frame % config.app.rendering_rate == 0) {
                    se::raycaster::render_volume_kernel(output_volume_img_data.get(),
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
                // Create vectors of images and labels.
                cv::Size res(processed_img_res.x(), processed_img_res.y());
                std::vector<cv::Mat> images;
                std::vector<std::string> labels;
                labels.emplace_back("INPUT RGB");
                images.emplace_back(res, CV_8UC4, output_rgba_img_data.get());
                labels.emplace_back("INPUT DEPTH");
                images.emplace_back(res, CV_8UC4, output_depth_img_data.get());
                labels.emplace_back(config.app.enable_ground_truth ? "TRACKING OFF" : "TRACKING");
                images.emplace_back(res, CV_8UC4, output_tracking_img_data.get());
                labels.emplace_back("RENDER");
                images.emplace_back(res, CV_8UC4, output_volume_img_data.get());
                // Combine all the images into one, overlay the labels and show it.
                cv::Mat render = se::montage(2, 2, images, labels);
                drawit(reinterpret_cast<uint32_t*>(render.data),
                       Eigen::Vector2i(render.cols, render.rows));
            }
            TOCK("draw")

            // Save logs, mesh, slices and struct (if enabled)
            TOCK("total")
            const bool last_frame =
                frame == config.app.max_frames || static_cast<size_t>(frame) == reader->numFrames();
            if ((config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0)
                || last_frame) {
                if (!config.app.mesh_path.empty()) {
                    map.saveMesh(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
                }
                if (!config.app.slice_path.empty()) {
                    map.saveFieldSlices(
                        config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                        config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                        config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                        se::math::to_translation(T_WS));
                }
                if (!config.app.structure_path.empty()) {
                    map.saveStructure(config.app.structure_path + "/struct_" + std::to_string(frame)
                                      + ".ply");
                }
            }

            se::perfstats.sample("memory usage",
                                 se::system::memory_usage_self() / 1024.0 / 1024.0,
                                 PerfStats::MEMORY);
            se::perfstats.writeToFilestream();
        }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
