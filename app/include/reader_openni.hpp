/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef __READER_OPENNI_HPP
#define __READER_OPENNI_HPP

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include "reader_base.hpp"
#include "se/image/image.hpp"

#ifdef SE_OPENNI2
#    include <openni2/OpenNI.h>
#endif



namespace se {

/** Reader for the Microsoft Kinect and Asus Xtion using the OpenNI2 driver.
 * https://structure.io/openni
 */
class OpenNIReader : public Reader {
    public:
    /** Construct an OpenNIReader from a ReaderConfig.
     *
     * \param[in] c The configuration struct to use.
     */
    OpenNIReader(const ReaderConfig& c);

    ~OpenNIReader();

    /** Restart reading from the beginning. */
    void restart();

    /** The name of the reader.
     *
     * \return The string `"OpenNIReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
#ifdef SE_OPENNI2
    template<typename T>
    class MyFrameAllocator : public openni::VideoStream::FrameAllocator {
        public:
        MyFrameAllocator(T* buffer) : buffer_(buffer)
        {
        }
        void* allocateFrameBuffer(int)
        {
            return buffer_;
        }
        void freeFrameBuffer(void*)
        {
        }

        private:
        T* buffer_;
    };

    std::unique_ptr<MyFrameAllocator<uint16_t>> depth_allocator_;
    std::unique_ptr<MyFrameAllocator<uint8_t>> colour_allocator_;

    openni::Status rc_;
    openni::Device device_;
    openni::VideoStream depth_stream_;
    openni::VideoStream colour_stream_;
    openni::VideoFrameRef depth_frame_;
    openni::VideoFrameRef colour_frame_;
    std::unique_ptr<uint16_t> depth_image_;
    std::unique_ptr<uint8_t> colour_image_;
#endif

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextColour(Image<uint32_t>& colour_image);
};

} // namespace se

#endif
