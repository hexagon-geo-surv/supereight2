/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_CONFIG_HPP
#define SE_CONFIG_HPP



#include "reader.hpp"
#include "se/map/map.hpp"
#include "se/sensor/sensor.hpp"
#include "se/tracker/tracker.hpp"



namespace se {



struct AppConfig {
    /** Whether to use the available ground truth camera pose.
     *
     * <br>\em Default: false
     */
    bool enable_ground_truth;

    /** Whether to show the supereight renders. Hiding them results in faster operation.
     *
     * <br>\em Default: true
     */
    bool enable_rendering;

    /** Whether to show the GUI. Hiding the GUI results in faster operation.
     *
     * <br>\em Default: true
     */
    bool enable_gui;

    /** Whether to mesh the octree.
     *
     * <br>\em Default: false
     */
    bool enable_meshing;

    /** Whether to create meshes of a slice from the octree.
     *
     * <br>\em Default: false
     */
    bool enable_slice_meshing;

    /** Whether to created meshes of the octree's structure.
     *
     * <br>\em Default: false
     */
    bool enable_structure_meshing;

    /** The directory where meshes are saved.
     *
     * <br>\em Default: ""
     */
    std::string mesh_output_dir;

    /** The ratio of the input frame size over the frame size used internally.
     * Values greater than 1 result in the input frames being downsampled
     * before processing. Valid values are 1, 2, 4 and 8.
     *
     * <br>\em Default: 1
     */
    int sensor_downsampling_factor;

    /** Perform tracking on a frame every tracking_rate frames.
     *
     * <br>\em Default: 1
     */
    int tracking_rate;

    /** Integrate a 3D reconstruction every integration_rate frames. Should not
     * be less than tracking_rate.
     *
     * <br>\em Default: 1
     */
    int integration_rate;

    /** Render the 3D reconstruction every rendering_rate frames.
     *
     * \note AppConfig::enable_render == true (default) required.
     *
     * Special cases:
     * If rendering_rate == 0 the volume is only rendered for configuration::max_frame.
     * If rendering_rate < 0  the volume is only rendered for frame abs(rendering_rate).
     *
     * <br>\em Default: 4
     */
    int rendering_rate;

    /** Mesh the 3D reconstruction every meshing_rate frames.
     *
     * Special cases:
     * If meshing_rate == 0 the volume is only meshed for configuration::max_frame.
     * If meshing_rate < 0  the volume is only meshed for frame abs(meshing_rate).
     *
     * <br>\em Default: 100
     */
    int meshing_rate;

    /** The maximum number of frames to read. Set to -1 to read the whole dataset.
     *
     * <br>\em Default: -1 (full dataset)
     */
    int max_frames;

    /** The file where the timing results will be written to. The timing results will be written to
     * standard output if log_file is empty.
     *
     * \todo Show on stdout on empty string. Will have to update the perfstats.
     *
     * <br>\em Default: ""
     */
    std::string log_file;



    /** Initializes the app config to some sensible defaults.
     */
    AppConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in AppConfig::AppConfig().
     */
    AppConfig(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



std::ostream& operator<<(std::ostream& os, const AppConfig& c);



template<typename DataConfigT, typename SensorConfigT>
struct Config {
    MapConfig map;
    DataConfigT data;
    SensorConfigT sensor;
    TrackerConfig tracker;
    ReaderConfig reader;
    AppConfig app;

    /** Default initializes all configs.
     */
    Config();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in Config::Config().
     */
    Config(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



template<typename DataConfigT, typename SensorConfigT>
std::ostream& operator<<(std::ostream& os, const Config<DataConfigT, SensorConfigT>& c);



} // namespace se

#include "impl/config_impl.hpp"

#endif // SE_CONFIG_HPP
