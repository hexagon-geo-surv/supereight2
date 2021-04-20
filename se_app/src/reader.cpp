/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#include "reader.hpp"

#include "filesystem.hpp"
#include "se/str_utils.hpp"
#include "reader_iclnuim.hpp"
#include "reader_newercollege.hpp"
#include "reader_openni.hpp"
#include "reader_raw.hpp"



se::Reader* se::create_reader(const Configuration& config) {

  se::Reader* reader = nullptr;

  // Create the reader configuration from the general configuration
  const se::ReaderConfig reader_config = {
      config.fps,
      config.drop_frames,
      !config.enable_benchmark,
      config.sequence_path,
      config.ground_truth_file
  };

  // OpenNI from a camera or a file
  if (config.sequence_type == "openni" && (reader_config.sequence_path.empty()
        || (stdfs::path(reader_config.sequence_path).extension() == ".oni"))) {
    reader = new se::OpenNIReader(reader_config);

  // ICL-NUIM reader
  } else if (config.sequence_type == "iclnuim"
        && stdfs::is_directory(reader_config.sequence_path)) {
    reader = new se::ICLNUIMReader(reader_config);

  // Slambench 1.0 .raw reader
  } else if (config.sequence_type == "raw"
        && stdfs::path(reader_config.sequence_path).extension() == ".raw") {
    reader = new se::RAWReader(reader_config);

  // NewerCollege reader
  } else if (config.sequence_type == "newercollege"
        && stdfs::is_directory(reader_config.sequence_path)) {
    reader = new se::NewerCollegeReader(reader_config);

  } else {
    std::cerr << "Error: Unrecognised file format, file not loaded\n";
    reader = nullptr;
  }

  // Handle failed initialization
  if ((reader != nullptr) && !reader->good()) {
    delete reader;
    reader = nullptr;
  }
  return reader;
}

