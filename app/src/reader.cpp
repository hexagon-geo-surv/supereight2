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
    if (config.reader_type == se::ReaderType::OPENNI) {
        reader = new se::OpenNIReader(config);
    }
    else if (config.reader_type == se::ReaderType::RAW) {
        reader = new se::RAWReader(config);
    }
    else if (config.reader_type == se::ReaderType::NEWERCOLLEGE) {
        reader = new se::NewerCollegeReader(config);
    }
    else if (config.reader_type == se::ReaderType::TUM) {
        reader = new se::TUMReader(config);
    }
    else if (config.reader_type == se::ReaderType::INTERIORNET) {
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
