/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#ifndef __READER_BASE_HPP
#define __READER_BASE_HPP

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "se/image/image.hpp"
#include "se/str_utils.hpp"



namespace se {

  struct ReaderConfig {
    float fps;
    bool drop_frames;
    bool enable_print;
    std::string sequence_path;
    std::string ground_truth_file;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };



  /** The result of trying to read a depth/RGB image or a pose.
   */
  enum class ReaderStatus : int {
    /** Data read successfully. */
    ok = 0,
    /** Temporary data read error.
     * Further reads might succeed. Typically used to indicate an invalid
     * image or pose. */
    skip,
    /** End of dataset reached. */
    eof,
    /** Fatal data read error.
     * No further read should be attempted. Typically used to indicate that
     * the dataset could not be read at all or no camera was found. */
    error,
  };



  /** Base abstract class for dataset readers.
   *
   * The overloaded nextData() functions all increment the frame counter, so
   * only one of them should be called within a single pipeline iteration.
   *
   * \note Derived classes should adhere to the following rules:
   *     - The se::Reader constructor should be called at the very beginning of
   *       all derived class constructors.
   *     - se::Reader::restart() should be called at the very beginning of all
   *       derived class implementations of restart().
   *     - The value of se::Reader::is_live_reader_ should be set appropriately
   *       at the derived class constructor.
   *     - The value of se::Reader::status_ should only be set in derived class
   *       constructors and se::Reader::restart() if applicable.
   */
  class Reader {
    public:
      /** Construct a Reader from a ReaderConfig.
       *
       * \note This constructor should be called in all derived class
       *       constructors.
       *
       * \param[in] c The configuration struct to use.
       */
      Reader(const ReaderConfig& c);

      virtual ~Reader() {};

      /** Read the next depth image.
       *
       * \note The frame number is incremented when calling this function.
       *
       * \param[out] depth_image The next depth image.
       * \return An appropriate status code.
       */
      ReaderStatus nextData(Image<float>& depth_image);

      /** Read the next depth and RGBA images.
       *
       * \note The frame number is incremented when calling this function.
       *
       * \param[out] depth_image The next depth image.
       * \param[out] rgba_image  The next RGBA image.
       * \return An appropriate status code.
       */
      ReaderStatus nextData(Image<float>&    depth_image,
                            Image<uint32_t>& rgba_image);

      /** Read the next depth and RGBA images and ground truth pose.
       *
       * \note The frame number is incremented when calling this function.
       *
       * \param[out] depth_image The next depth image.
       * \param[out] rgba_image  The next RGBA image.
       * \param[out] T_WB        The next ground truth pose.
       * \return An appropriate status code.
       */
      ReaderStatus nextData(Image<float>&    depth_image,
                            Image<uint32_t>& rgba_image,
                            Eigen::Matrix4f& T_WB);

     /** Read the ground truth pose at the provided frame number.
       * Each line in the ground truth file should correspond to a single
       * depth/RGBA image pair and have a format<br>
       * `... tx ty tz qx qy qz qw`,<br>
       * that is the pose is encoded in the last 7 columns of the line.
       *
       * \param[in]  frame The frame number of the requested ground truth pose.
       * \param[out] T_WB  The ground truth pose.
       * \return An appropriate status code.
       */
      ReaderStatus getPose(Eigen::Matrix4f& T_WB, const size_t frame);

      /** Restart reading from the beginning.
       *
       * \note Although this is a virtual function, it does have a default
       *       implementation that resets the Reader protected variables.
       *       Reader::restart() (this default implementation) should be called
       *       at the very beginning of any implementation of restart().
       */
      virtual void restart() = 0;

      /** The name of the reader.
       *
       * \return The name of the reader as a string.
       */
      virtual std::string name() const = 0;

      /** The state of the reader.
       * If good() returns false, there is no need to keep reading frames.
       *
       * \return True if the last call to one of the Reader::next*() functions
       *         completed successfully.
       */
      bool good() const;

      /** The current frame number.
       * Frame numbering starts from 0, so the number of frames read so far is
       * `frame() + 1`.
       *
       * \return The number of frames read. Returns SIZE_MAX if no frames
       *         have been read yet.
       */
      size_t frame() const;

      /** The total number of frames in the current dataset.
       *
       * \return The total number of frames. Returns 0 if the number of frames
       *         is unknown (e.g. for camera input).
       */
      size_t numFrames() const;

      /** The dimensions of the depth images.
       *
       * \return A 2D vector containing the width and height of the images.
       */
      Eigen::Vector2i depthImageRes() const;

      /** The dimensions of the RGBA images.
       *
       * \return A 2D vector containing the width and height of the images.
       */
      Eigen::Vector2i RGBAImageRes() const;

      /** Whether the reader uses a live camera as input.
       *
       * \return True if the input is from a live camera, false otherwise.
       */
      bool isLiveReader() const;

      /** Don't use this.
       * \deprecated Ugly hack only for use with the Qt GUI until we fix it.
       */
      bool camera_active_;

      /** Don't use this.
       * \deprecated Ugly hack only for use with the Qt GUI until we fix it.
       */
      bool camera_open_;

      /** Merge se::ReaderStatus values keeping the worst one.
       *
       * \param[in] status_1 The first se::ReaderStatus to merge.
       * \param[in] status_2 The second se::ReaderStatus to merge.
       * \return The worst of the 2 statuses.
       */
      static ReaderStatus mergeStatus(ReaderStatus status_1,
                                      ReaderStatus status_2);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
      std::string sequence_path_;
      std::string ground_truth_file_;
      std::ifstream ground_truth_fs_;
      Eigen::Vector2i depth_image_res_;
      Eigen::Vector2i rgba_image_res_;
      float fps_;
      double spf_;
      bool drop_frames_;
      bool enable_print_;
      bool is_live_reader_;
      ReaderStatus status_;
      /** The frame_ is initialized to SIZE_MAX, so that when first incremented
       * it becomes 0. Unsigned integer overflow is defined behaviour in C/C++
       * so this is safe to do.
       */
      size_t frame_;
      size_t num_frames_;


      /** Read the ground truth pose at the provided frame number.
       * Each line in the ground truth file should correspond to a single
       * depth/RGBA image pair and have a format<br>
       * `... tx ty tz qx qy qz qw`,<br>
       * that is the pose is encoded in the last 7 columns of the line.
       *
       * \note Use getPose(...) to request a pose. It keeps track of the ground_truth_frame_
       *       and ifstream state.
       *
       * \param[in]  frame The frame number of the requested ground truth pose.
       * \param[out] T_WB  The ground truth pose.
       * \return An appropriate status code.
       */
      ReaderStatus readPose(Eigen::Matrix4f& T_WB, const size_t frame);


      /** Read the next ground truth pose.
       * Each line in the ground truth file should correspond to a single
       * depth/RGBA image pair and have a format<br>
       * `... tx ty tz qx qy qz qw`,<br>
       * that is the pose is encoded in the last 7 columns of the line.
       *
       * \note The frame number is NOT incremented inside this function.
       *
       * \param[out] T_WB The next ground truth pose.
       * \return An appropriate status code.
       */
      ReaderStatus nextPose(Eigen::Matrix4f& T_WB);

    private:
      size_t ground_truth_frame_;
      std::chrono::steady_clock::time_point prev_frame_timestamp_;

      /** Prepare for reading the next frame.
       * If Reader::fps_ is 0, this function just increments the frame number
       * by 1. Otherwise it sleeps for the required time so that the desired
       * framerate is achieved. It then increments the frame number by 1 or
       * more, so that the value of Reader::drop_frames_ is respected.
       */
      void nextFrame();

      /** Read the next depth image.
       *
       * \param[out] depth_image The next depth image.
       * \return An appropriate status code.
       */
      virtual ReaderStatus nextDepth(Image<float>& depth_image) = 0;

      /** Read the next RGBA image.
       *
       * \param[out] rgba_image The next RGBA image.
       * \return An appropriate status code.
       */
      virtual ReaderStatus nextRGBA(Image<uint32_t>& rgba_image) = 0;
  };

} // namespace se

/** The overview of the reader configuration in configuration format.
 *
 * \return The configuration of the reader to the ostream.
 */
static std::ostream& operator<<(std::ostream& out, se::Reader* reader) {
  out << str_utils::header_to_pretty_str("READER") << "\n";
  out << str_utils::str_to_pretty_str(reader->name(),  "Reader type") << "\n";
  out << str_utils::str_to_pretty_str(((reader->numFrames() == 0) ? "Unknown" : std::to_string(reader->numFrames())),
                                           "Number frames") << "\n";
  out << "\n";
  return out;
}


#endif

