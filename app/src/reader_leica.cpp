/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_leica.hpp"

#include <cassert>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <regex>

#include "se/common/filesystem.hpp"
#include "se/common/image_utils.hpp"
#include "se/common/yaml.hpp"

/** A timestamped ground truth pose (VIO estimate) .
 */
struct LeicaPoseEntry {
    uint64_t timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;

    /** Initialize an invalid LeicaPoseEntry.
     */
    LeicaPoseEntry() = default;

    LeicaPoseEntry(const double t, const Eigen::Vector3f& p, const Eigen::Quaternionf& o) :
            timestamp(t), position(p), orientation(o)
    {
    }

    /**
     * Initialize using a single-line string from a Leica trajectory.csv
     */
    LeicaPoseEntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ',', true);
        timestamp = std::stol(columns[1].c_str());
        position =
            Eigen::Vector3f(std::stof(columns[2]), std::stof(columns[3]), std::stof(columns[4]));
        orientation = Eigen::Quaternionf(std::stof(columns[5]),
                                         std::stof(columns[6]),
                                         std::stof(columns[7]),
                                         std::stof(columns[8]));
    }

    /** Return a single-line string representation of the ground truth (VIO) pose.
     * It can be used to write it to a ground truth file that is understood by
     * supereight.
     */
    std::string string() const
    {
        const std::string s = std::to_string(timestamp) + "," + std::to_string(position.x()) + ","
            + std::to_string(position.y()) + "," + std::to_string(position.z()) + ","
            + std::to_string(orientation.x()) + "," + std::to_string(orientation.y()) + ","
            + std::to_string(orientation.z()) + "," + std::to_string(orientation.w());
        return s;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** A timestamped ground truth pose (VIO estimate) .
 */
struct LeicaLiDAREntry {
    uint64_t timestamp;
    Eigen::Vector3f position;
    int intensity;

    /** Initialize an invalid LeicaPoseEntry.
     */
    LeicaLiDAREntry() = default;

    LeicaLiDAREntry(const double t, const Eigen::Vector3f& p, const int intensity) :
            timestamp(t), position(p), intensity(intensity)
    {
    }

    /**
     * Initialize using a single-line string from a Leica trajectory.csv
     */
    LeicaLiDAREntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ',', true);
        timestamp = std::stol(columns[0].c_str());
        position =
            Eigen::Vector3f(std::stof(columns[1]), std::stof(columns[2]), std::stof(columns[3]));
        intensity = std::stoi(columns[4]);
    }

    /** Return a single-line string representation of the ground truth (VIO) pose.
     * It can be used to write it to a ground truth file that is understood by
     * supereight.
     */
    std::string string() const
    {
        const std::string s = std::to_string(timestamp) + "," + std::to_string(position.x()) + ","
            + std::to_string(position.y()) + "," + std::to_string(position.z()) + ","
            + std::to_string(intensity);
        return s;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool readLidarAndPoses(
    const std::string& path,
    float scan_time_interval,
    std::vector<std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>>& clouds,
    std::vector<LeicaPoseEntry>& interpolatedPoses)
{
    // Ensure a valid directory was provided
    if (!stdfs::is_directory(path)) {
        return false;
    }

    std::vector<uint64_t> cloudTimestamps;

    // Setup stream of pose data
    std::ifstream poseStream;
    poseStream.open(path + "/trajectory.csv");
    std::string poseLine;
    // set reading position to 2nd line
    std::getline(poseStream, poseLine);
    std::getline(poseStream, poseLine);
    // read first pose
    LeicaPoseEntry pose(poseLine);

    // Setup stream of lidar data
    std::ifstream lidarStream;
    lidarStream.open(path + "/lidar.csv");
    if (!lidarStream.good()) {
        std::cerr << "Error: " << path + "/lidar.csv could not be found \n";
        return false;
    }
    std::string line;
    int numberOfLines = 0;
    while (std::getline(lidarStream, line))
        numberOfLines++;
    if (numberOfLines - 1 <= 0) {
        std::cerr << "Error: No LiDAR Measurements present in: " << path + "/lidar.csv\n";
        return false;
    }
    else {
        std::clog << "Found " << numberOfLines << " measurement points\n";
    }

    // set reading position to second line
    lidarStream.clear();
    lidarStream.seekg(0, std::ios::beg);
    std::getline(lidarStream, line);
    std::getline(lidarStream, line);
    // Skip LiDAR measurements that are older than trajectory
    LeicaLiDAREntry lidarMeas(line);
    while (lidarMeas.timestamp < pose.timestamp) {
        std::getline(lidarStream, line);
        lidarMeas = LeicaLiDAREntry(line);
    }
    std::clog << "First Pose at t = " << pose.timestamp << std::endl;
    std::clog << " Starting to read LiDAR from t = " << lidarMeas.timestamp << std::endl;

    // initialise last timestamp_
    uint64_t last_timestamp = 0;


    bool endOfFile = false;

    while (true) {
        // Initialize Point Cloud
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> cloud;

        if (last_timestamp == 0) { //first pose

            // add to point cloud
            cloud.push_back(Eigen::Vector3f(
                lidarMeas.position[0], lidarMeas.position[1], lidarMeas.position[2]));
            last_timestamp = lidarMeas.timestamp;
        }

        while ((lidarMeas.timestamp - last_timestamp) * 1e-09 <= scan_time_interval) {
            // take two seconds of scan
            // get first line
            std::getline(lidarStream, line);
            if (!lidarStream.good()) {
                std::cout << "reached end of lidar file" << std::endl;
                endOfFile = true;
                break;
            }
            lidarMeas = LeicaLiDAREntry(line);
            cloud.push_back(Eigen::Vector3f(
                lidarMeas.position[0], lidarMeas.position[1], lidarMeas.position[2]));
        }

        LeicaPoseEntry prevPose;
        while (pose.timestamp < (lidarMeas.timestamp + last_timestamp) / 2) {
            prevPose = pose;
            std::getline(poseStream, poseLine);
            pose = LeicaPoseEntry(poseLine);
        }

        // do interpolation
        double r =
            (static_cast<double>((lidarMeas.timestamp + last_timestamp) / 2 - prevPose.timestamp))
            / static_cast<double>(pose.timestamp - prevPose.timestamp);
        uint64_t tsInterp = (lidarMeas.timestamp + last_timestamp) / 2;
        Eigen::Vector3f posInterp = r * pose.position + (1. - r) * prevPose.position;
        Eigen::Quaternionf oriInterp = prevPose.orientation.slerp(r, pose.orientation);
        LeicaPoseEntry poseInterp(tsInterp, posInterp, oriInterp);
        interpolatedPoses.push_back(poseInterp);

        cloudTimestamps.push_back((lidarMeas.timestamp + last_timestamp) / 2);
        last_timestamp = lidarMeas.timestamp;
        clouds.push_back(cloud);
        if (endOfFile)
            break;
    }
    std::clog << "Created " << clouds.size() << " point clouds from overall LiDAR scan data"
              << std::endl;
    return true;
}

bool readLidarAndPosesWithMotionCompensation(
    const std::string& path,
    const float scan_time_interval,
    const Eigen::Matrix4f T_BL,
    std::vector<std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>>& clouds,
    std::vector<LeicaPoseEntry>& poses)
{
    // Ensure a valid directory was provided
    if (!stdfs::is_directory(path)) {
        std::cerr << "Error: " << path + " is not a valid path \n";
        return false;
    }

    // Setup stream of pose data
    std::ifstream poseStream;
    poseStream.open(path + "/trajectory.csv");
    if (!poseStream.good()) {
        std::cerr << "Error: " << path + "/trajectory.csv could not be found \n";
        return false;
    }

    // Setup stream of lidar data
    std::ifstream lidarStream;
    lidarStream.open(path + "/lidar.csv");
    if (!lidarStream.good()) {
        std::cerr << "Error: " << path + "/lidar.csv could not be found \n";
        return false;
    }

    // Initialize line strings and set reading position to 2n
    std::string poseLine, lidarLine;

    // Read first lidar Measurement
    std::getline(lidarStream, lidarLine);
    std::getline(lidarStream, lidarLine);
    LeicaLiDAREntry lidarMeas(lidarLine);

    // Read first pose
    std::getline(poseStream, poseLine);
    std::getline(poseStream, poseLine);
    LeicaPoseEntry pose(poseLine);

    // Skip LiDAR measurements that are older than trajectory
    while (lidarMeas.timestamp < pose.timestamp) {
        std::getline(lidarStream, lidarLine);
        lidarMeas = LeicaLiDAREntry(lidarLine);
    }
    std::clog << "First Pose at t = " << pose.timestamp << std::endl;
    std::clog << " Starting to read LiDAR from t = " << lidarMeas.timestamp << std::endl;

    // -----------------------------------------------------------
    // initialise varialbles holding previous values for iteration
    uint64_t prevTimestamp = 0;

    LeicaPoseEntry prevPose, basePose, interpPose;
    prevPose = pose; // pose right before measurement
    basePose = pose; // pose that one scan interval is referenced to

    LeicaLiDAREntry prevLidarMeas;
    prevLidarMeas = lidarMeas;

    bool endOfLidarFile = false;
    bool endOfTrajectoryFile = false;

    while (true) {
        // Initialize Point Cloud
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> cloud;

        // Variable holding motion compensated measurement
        Eigen::Vector3f posMotionCompensated;

        if (prevTimestamp == 0) { // first measurement

            // Get next (second) pose
            std::getline(poseStream, poseLine);
            pose = LeicaPoseEntry(poseLine);

            // interpolate pose
            double r = static_cast<double>((lidarMeas.timestamp - prevPose.timestamp))
                / static_cast<double>(pose.timestamp - prevPose.timestamp);
            Eigen::Vector3f posInterp = r * pose.position + (1. - r) * prevPose.position;
            Eigen::Quaternionf oriInterp = prevPose.orientation.slerp(r, pose.orientation);
            interpPose = LeicaPoseEntry(lidarMeas.timestamp, posInterp, oriInterp);
            Eigen::Matrix4f T_WB_base;
            T_WB_base.setIdentity();
            T_WB_base.topLeftCorner<3, 3>() = basePose.orientation.toRotationMatrix();
            T_WB_base.topRightCorner<3, 1>() = basePose.position;
            Eigen::Matrix4f T_WB_interp;
            T_WB_interp.setIdentity();
            T_WB_interp.topLeftCorner<3, 3>() = interpPose.orientation.toRotationMatrix();
            T_WB_interp.topRightCorner<3, 1>() = interpPose.position;

            posMotionCompensated =
                (T_BL.inverse() * T_WB_base.inverse() * T_WB_interp * T_BL
                 * Eigen::Vector4f(
                     lidarMeas.position[0], lidarMeas.position[1], lidarMeas.position[2], 1.0))
                    .head<3>();

            cloud.push_back(Eigen::Vector3f(
                posMotionCompensated[0], posMotionCompensated[1], posMotionCompensated[2]));
            prevTimestamp = lidarMeas.timestamp;

            // set base pose
            basePose = prevPose;
        }
        else {
            // Get one scan interval
            while ((lidarMeas.timestamp - prevTimestamp) * 1e-09 <= scan_time_interval) {
                // check if timestamp still between prevPose and pose
                while (lidarMeas.timestamp > pose.timestamp) {
                    prevPose = pose;
                    std::getline(poseStream, poseLine);
                    if (!poseStream.good()) {
                        std::cout << "reached end of trajectory file" << std::endl;
                        endOfTrajectoryFile = true;
                        break;
                    }
                    pose = LeicaPoseEntry(poseLine);
                }

                if (endOfTrajectoryFile)
                    break;

                // interpolate pose
                double r = static_cast<double>((lidarMeas.timestamp - prevPose.timestamp))
                    / static_cast<double>(pose.timestamp - prevPose.timestamp);
                Eigen::Vector3f posInterp = r * pose.position + (1. - r) * prevPose.position;
                Eigen::Quaternionf oriInterp = prevPose.orientation.slerp(r, pose.orientation);
                interpPose = LeicaPoseEntry(lidarMeas.timestamp, posInterp, oriInterp);
                Eigen::Matrix4f T_WB_base;
                T_WB_base.setIdentity();
                T_WB_base.topLeftCorner<3, 3>() = basePose.orientation.toRotationMatrix();
                T_WB_base.topRightCorner<3, 1>() = basePose.position;
                Eigen::Matrix4f T_WB_interp;
                T_WB_interp.setIdentity();
                T_WB_interp.topLeftCorner<3, 3>() = interpPose.orientation.toRotationMatrix();
                T_WB_interp.topRightCorner<3, 1>() = interpPose.position;

                posMotionCompensated =
                    (T_BL.inverse() * T_WB_base.inverse() * T_WB_interp * T_BL
                     * Eigen::Vector4f(
                         lidarMeas.position[0], lidarMeas.position[1], lidarMeas.position[2], 1.0))
                        .head<3>();

                cloud.push_back(Eigen::Vector3f(
                    posMotionCompensated[0], posMotionCompensated[1], posMotionCompensated[2]));


                // Get next measurement
                std::getline(lidarStream, lidarLine);
                if (!lidarStream.good()) {
                    std::cout << "reached end of lidar file" << std::endl;
                    endOfLidarFile = true;
                    break;
                }
                lidarMeas = LeicaLiDAREntry(lidarLine);
            }
            prevTimestamp = lidarMeas.timestamp;

            // Save cloud and pose
            clouds.push_back(cloud);
            poses.push_back(basePose);

            // set basePose for next scan interval
            if (lidarMeas.timestamp > pose.timestamp)
                basePose = pose;
            else
                basePose = prevPose;
        }

        if (endOfLidarFile || endOfTrajectoryFile)
            break;
    }

    std::clog << "Created " << clouds.size() << " point clouds from overall LiDAR scan data"
              << std::endl;
    return true;
}

/** Generate a ground truth file from poses and write it in a temporary file.
 */
std::string write_ground_truth_tmp(const std::string& path,
                                   const std::vector<LeicaPoseEntry>& poses)
{
    // Open a temporary file
    const std::string tmp_filename = path + "/trajectory_se.csv";
    std::ofstream fs(tmp_filename, std::ios::out);
    if (!fs.good()) {
        std::cerr << "Error: Could not write trajectory file " << tmp_filename << "\n";
        return "";
    }
    // Write the header
    fs << "# timestamp, tx, ty, tz, qx, qy, qz, qw\n";
    // Write each of the associated poses
    for (size_t i = 0; i < poses.size(); ++i) {
        fs << poses[i].string() << "\n";
    }
    return tmp_filename;
}



se::LeicaReader::LeicaReader(const se::ReaderConfig& c) : se::Reader(c)
{
    // Ensure a valid directory was provided
    if (!stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
        return;
    }
    std::cout << "created leica reader" << std::endl;

    if (c.leica_reader_type == "rangeImage") {
        // Set the depth and RGBA image resolutions.
        depth_image_res_ = Eigen::Vector2i(c.width, c.height);
        rgba_image_res_ = Eigen::Vector2i(c.width, c.height);
        azimuth_angular_resolution_ = 360.0f / c.width;
        elevation_angular_resolution_ = 180.0f / c.height;

        // Read Lidar and Pose data and store as vector of Point clouds respectively write interpolated poses
        std::vector<LeicaPoseEntry> interpolatedPoses;
        interpolatedPoses.clear();
        clouds_.clear();
        if (c.use_motion_comp) {
            readLidarAndPosesWithMotionCompensation(
                sequence_path_, c.scan_time_interval, c.T_BL, clouds_, interpolatedPoses);
        }
        else {
            readLidarAndPoses(sequence_path_, c.scan_time_interval, clouds_, interpolatedPoses);
        }

        num_frames_ = clouds_.size();

        // write csv with interpolated poses
        std::string tmpTrajFile = write_ground_truth_tmp(sequence_path_, interpolatedPoses);
        // Open interpolated Poses
        ground_truth_fs_.close();
        ground_truth_fs_.open(tmpTrajFile, std::ios::in);
        if (!ground_truth_fs_.good()) {
            std::cerr << "Error: Could not read generated ground truth file " << tmpTrajFile
                      << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
    }
    else if (c.leica_reader_type == "ray") {
        // Setup stream of lidar data
        lidar_stream_.open(sequence_path_ + "/lidar.csv");
        if (!lidar_stream_.good()) {
            status_ = se::ReaderStatus::error;
            std::cerr << "Error: " << sequence_path_ + "/lidar.csv could not be found \n";
            return;
        }
        std::string line;
        int numberOfLines = 0;
        while (std::getline(lidar_stream_, line))
            numberOfLines++;
        if (numberOfLines - 1 <= 0) {
            status_ = se::ReaderStatus::error;
            std::cerr << "Error: No LiDAR Measurements present in: "
                      << sequence_path_ + "/lidar.csv\n";
            return;
        }
        else if (verbose_ >= 1) {
            std::clog << "Found " << numberOfLines << " measurement points\n";
        }

        // set reading position to second line
        lidar_stream_.clear();
        lidar_stream_.seekg(0, std::ios::beg);
        std::getline(lidar_stream_, line);

        // initialise current LiDAR timestamp_
        ray_timestamp_ = 0;


        // Setup stream of trajectory data
        trajectory_stream_.open(sequence_path_ + "/trajectory.csv");
        if (!trajectory_stream_.good()) {
            status_ = se::ReaderStatus::error;
            std::cerr << "Error: " << sequence_path_ + "/trajectory.csv could not be found \n";
            return;
        }
        std::string line2;
        int numberOfLines2 = 0;
        while (std::getline(trajectory_stream_, line2))
            numberOfLines2++;
        if (numberOfLines2 - 1 <= 0) {
            status_ = se::ReaderStatus::error;
            std::cerr << "Error: No Pose Data present in: " << sequence_path_ + "/trajectory.csv\n";
            return;
        }
        else if (verbose_ >= 1) {
            std::clog << "Found " << numberOfLines << " VIO poses\n";
        }

        // set reading position to second line
        trajectory_stream_.clear();
        trajectory_stream_.seekg(0, std::ios::beg);
        std::getline(trajectory_stream_, line2);

        // Set reading positions for first measurements (t_ray > t_pose; required for interpolation)
        std::getline(lidar_stream_, line);
        LeicaLiDAREntry lidarMeas(line);

        std::getline(trajectory_stream_, line2);
        LeicaPoseEntry pose(line2);

        while (lidarMeas.timestamp < pose.timestamp) {
            std::getline(lidar_stream_, line);
            lidarMeas = LeicaLiDAREntry(line);
        }
        ray_timestamp_ = lidarMeas.timestamp;

        // Save first previous pose
        ts_prev_ = pose.timestamp;
        pos_prev_ = pose.position;
        ori_prev_ = pose.orientation;

        // Save first current pose
        std::getline(trajectory_stream_, line2);
        pose = LeicaPoseEntry(line2);
        ts_curr_ = pose.timestamp;
        pos_curr_ = pose.position;
        ori_curr_ = pose.orientation;

        while (ts_curr_ < ray_timestamp_) {
            std::getline(trajectory_stream_, line2);
            ts_prev_ = ts_curr_;
            pos_prev_ = pos_curr_;
            ori_prev_ = ori_curr_;
            pose = LeicaPoseEntry(line2);
            ts_curr_ = pose.timestamp;
            pos_curr_ = pose.position;
            ori_curr_ = pose.orientation;
        }
    }
}



void se::LeicaReader::restart()
{
    // ToDo: need to reset Reader Stream here?
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
    }
}



std::string se::LeicaReader::name() const
{
    return std::string("LeicaReader");
}



se::ReaderStatus se::LeicaReader::nextDepth(se::Image<float>& depth_image)
{
    std::clog << "frame_ " << frame_ << " with " << clouds_.size() << "pointclouds\n";
    if (frame_ == clouds_.size()) {
        std::clog << " Reached EOF" << std::endl;
        return se::ReaderStatus::eof;
    }

    // Resize the output image if needed.
    if ((depth_image.width() != depth_image_res_.x())
        || (depth_image.height() != depth_image_res_.y())) {
        depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
    }
    // Initialize the image to zeros.
    std::memset(depth_image.data(), 0, depth_image_res_.prod() * sizeof(float));

    // --------------------------
    // -----Create depth img-----
    // --------------------------
    // Read the point cloud and convert each point to a distance.
    uint64_t count = 0;
    for (const auto& point : clouds_.at(frame_)) {
        const float_t R = Eigen::Vector3f(point.x(), point.y(), point.z()).norm();
        // project
        if (R < 1.0e-12) {
            continue;
        }

        // Compute azimuth and elevation angles (projection) [deg]
        float rad2deg = 180.0 / M_PI;
        const float_t azimuth = std::atan2(point.x(), point.z()); // ToDo: 2.0 * M_PI - ... ???
        const float_t elevation = std::asin(point.y() / R);
        /*** note that z is in forward direction for BLK2Fly LiDAR sensor
         * This differs from commonly used conventions!
         */

        // Determine row and column in image
        float_t u = (azimuth + M_PI) * rad2deg / azimuth_angular_resolution_;
        float_t v = (elevation + 0.5 * M_PI) * rad2deg / elevation_angular_resolution_;


        // azimuthal wrap-around
        if (u < -0.5) {
            u += depth_image_res_.x();
        }
        if (u > depth_image_res_.x() - 0.5) {
            u -= depth_image_res_.x();
        }
        if (v < -0.5) {
            v += depth_image_res_.y();
        }
        if (v > depth_image_res_.y() - 0.5) {
            v -= depth_image_res_.y();
        }

        int row = std::round(v);
        int col = std::round(u);

        if (depth_image(col, row) > 0) {
            if (R < depth_image(col, row)) {
                depth_image(col, row) = R;
            }
        }
        else {
            ++count;
            depth_image(col, row) = R;
        }
    }

    return se::ReaderStatus::ok;
}

se::ReaderStatus se::LeicaReader::nextRay(Eigen::Vector3f& ray_measurement)
{
    std::string line;
    // Get 1 Line of LiDAR measurements = 1 ray
    std::getline(lidar_stream_, line);
    if (!lidar_stream_.good()) // Reached end of file
        return se::ReaderStatus::eof;

    LeicaLiDAREntry ray;
    ray = LeicaLiDAREntry(line);

    ray_measurement = ray.position;
    ray_timestamp_ = ray.timestamp;

    return se::ReaderStatus::ok;
}

se::ReaderStatus se::LeicaReader::nextPose(Eigen::Matrix4f& T_WB)
{
    std::string line;
    LeicaPoseEntry pose;
    T_WB.setIdentity();

    while (ray_timestamp_ > ts_curr_) {
        // Get next {pse
        std::getline(trajectory_stream_, line);
        if (!trajectory_stream_.good())
            return se::ReaderStatus::eof;
        ts_prev_ = ts_curr_;
        pos_prev_ = pos_curr_;
        ori_prev_ = ori_curr_;
        pose = LeicaPoseEntry(line);
        ts_curr_ = pose.timestamp;
        pos_curr_ = pose.position;
        ori_curr_ = pose.orientation;
    }

    // interpolate between poses
    double r =
        (static_cast<double>(ray_timestamp_ - ts_prev_)) / static_cast<double>(ts_curr_ - ts_prev_);
    Eigen::Vector3f posInterp = r * pos_curr_ + (1. - r) * pos_prev_;
    Eigen::Quaternionf oriInterp = ori_prev_.slerp(r, ori_curr_);
    T_WB.topLeftCorner<3, 3>() = oriInterp.toRotationMatrix();
    T_WB.topRightCorner<3, 1>() = posInterp;

    return se::ReaderStatus::ok;
}

se::ReaderStatus se::LeicaReader::nextRayBatch(
    const float batch_interval,
    std::vector<std::pair<Eigen::Matrix4f, Eigen::Vector3f>,
                Eigen::aligned_allocator<std::pair<Eigen::Matrix4f, Eigen::Vector3f>>>&
        rayPoseBatch)
{
    std::string lidarLine;
    // Get 1 Line of LiDAR measurements = 1 ray
    std::getline(lidar_stream_, lidarLine);
    if (!lidar_stream_.good()) // Reached end of file
        return se::ReaderStatus::eof;

    // Get 1st entry in current interval
    LeicaLiDAREntry ray;
    ray = LeicaLiDAREntry(lidarLine);
    uint64_t t0 = ray.timestamp;
    ray_timestamp_ = ray.timestamp;

    std::string trajectoryLine;
    LeicaPoseEntry pose;
    Eigen::Matrix4f T_WB;
    T_WB.setIdentity();

    while (ray_timestamp_ > ts_curr_) {
        // Get next pose
        std::getline(trajectory_stream_, trajectoryLine);
        if (!trajectory_stream_.good())
            return se::ReaderStatus::eof;
        ts_prev_ = ts_curr_;
        pos_prev_ = pos_curr_;
        ori_prev_ = ori_curr_;
        pose = LeicaPoseEntry(trajectoryLine);
        ts_curr_ = pose.timestamp;
        pos_curr_ = pose.position;
        ori_curr_ = pose.orientation;
    }

    // interpolate between poses
    double r =
        (static_cast<double>(ray_timestamp_ - ts_prev_)) / static_cast<double>(ts_curr_ - ts_prev_);
    Eigen::Vector3f posInterp = r * pos_curr_ + (1. - r) * pos_prev_;
    Eigen::Quaternionf oriInterp = ori_prev_.slerp(r, ori_curr_);
    T_WB.topLeftCorner<3, 3>() = oriInterp.toRotationMatrix();
    T_WB.topRightCorner<3, 1>() = posInterp;
    rayPoseBatch.push_back(std::pair<Eigen::Matrix4f, Eigen::Vector3f>(T_WB, ray.position));


    // now get interval
    while ((ray_timestamp_ - t0) * 1e-09 <= batch_interval) {
        std::getline(lidar_stream_, lidarLine);
        if (!lidar_stream_.good()) // Reached end of file
            return se::ReaderStatus::eof;

        // Lidar measurement
        LeicaLiDAREntry ray;
        ray = LeicaLiDAREntry(lidarLine);
        ray_timestamp_ = ray.timestamp;

        // Pose interpolated
        T_WB.setIdentity();

        while (ray_timestamp_ > ts_curr_) {
            // Get next pose
            std::getline(trajectory_stream_, trajectoryLine);
            if (!trajectory_stream_.good())
                return se::ReaderStatus::eof;
            ts_prev_ = ts_curr_;
            pos_prev_ = pos_curr_;
            ori_prev_ = ori_curr_;
            pose = LeicaPoseEntry(trajectoryLine);
            ts_curr_ = pose.timestamp;
            pos_curr_ = pose.position;
            ori_curr_ = pose.orientation;
        }

        // interpolate between poses
        double r = (static_cast<double>(ray_timestamp_ - ts_prev_))
            / static_cast<double>(ts_curr_ - ts_prev_);
        Eigen::Vector3f posInterp = r * pos_curr_ + (1. - r) * pos_prev_;
        Eigen::Quaternionf oriInterp = ori_prev_.slerp(r, ori_curr_);
        T_WB.topLeftCorner<3, 3>() = oriInterp.toRotationMatrix();
        T_WB.topRightCorner<3, 1>() = posInterp;
        rayPoseBatch.push_back(std::pair<Eigen::Matrix4f, Eigen::Vector3f>(T_WB, ray.position));
    }

    return se::ReaderStatus::ok;
}
se::ReaderStatus se::LeicaReader::nextRGBA(se::Image<uint32_t>& rgba_image)
{
    // Resize the output image if needed.
    if ((rgba_image.width() != rgba_image_res_.x())
        || (rgba_image.height() != rgba_image_res_.y())) {
        rgba_image = se::Image<uint32_t>(rgba_image_res_.x(), rgba_image_res_.y());
    }
    // Create a blank image
    std::memset(rgba_image.data(), 0, rgba_image_res_.prod() * sizeof(uint32_t));
    return se::ReaderStatus::ok;
}
