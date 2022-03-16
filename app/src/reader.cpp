/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "reader.hpp"

#include "reader_interiornet.hpp"
#include "reader_newercollege.hpp"
#include "reader_openni.hpp"
#include "reader_raw.hpp"
#include "reader_tum.hpp"
#include "se/common/filesystem.hpp"
#include "se/common/str_utils.hpp"



se::Reader* se::create_reader(const se::ReaderConfig& config)
{
    se::Reader* reader = nullptr;
    // OpenNI from a camera or a file
    if (config.reader_type == se::ReaderType::OPENNI
        && (config.sequence_path.empty()
            || (stdfs::path(config.sequence_path).extension() == ".oni"))) {
        reader = new se::OpenNIReader(config);
    } // Slambench 1.0 .raw reader
    else if (config.reader_type == se::ReaderType::RAW
             && stdfs::path(config.sequence_path).extension() == ".raw") {
        reader = new se::RAWReader(config);
    } // NewerCollege reader
    else if (config.reader_type == se::ReaderType::NEWERCOLLEGE
             && stdfs::is_directory(config.sequence_path)) {
        reader = new se::NewerCollegeReader(config);
    } // TUM reader
    else if (config.reader_type == se::ReaderType::TUM
             && stdfs::is_directory(config.sequence_path)) {
        reader = new se::TUMReader(config);
    } // InteriorNet reader
    else if (config.reader_type == se::ReaderType::INTERIORNET
             && stdfs::is_directory(config.sequence_path)) {
        reader = new se::InteriorNetReader(config);
    }
    else {
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
