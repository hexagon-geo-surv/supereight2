/*
 * SPDX-FileCopyrightText: 2011-2013 Gerhard Reitmayr, TU Graz
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_DRAW_HPP
#define SE_DRAW_HPP

#include <Eigen/Dense>

#ifdef SE_GLUT
#    include <algorithm>
#    include <cstdint>

#    ifdef __APPLE__
#        include <GLUT/glut.h>
#    else
#        include <GL/glut.h>
#    endif


template<typename T>
struct gl;

template<>
struct gl<float> {
    static const int format = GL_LUMINANCE;
    static const int type = GL_FLOAT;
};

template<>
struct gl<uint8_t> {
    static const int format = GL_LUMINANCE;
    static const int type = GL_UNSIGNED_BYTE;
};

template<>
struct gl<uint16_t> {
    static const int format = GL_LUMINANCE;
    static const int type = GL_UNSIGNED_SHORT;
};

template<>
struct gl<uint32_t> {
    static const int format = GL_RGBA;
    static const int type = GL_UNSIGNED_BYTE;
};



template<typename T>
void drawit(const T* scene, const Eigen::Vector2i& res)
{
    const Eigen::Vector2i content_res(res);

    // Create a GLUT window if one does not already exist.
    if (glutGetWindow() == 0) {
        int argc = 1;
        char* argv = (char*) "supereight";
        glutInit(&argc, &argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
        glutInitWindowSize(content_res.x(), content_res.y());
        glutCreateWindow("supereight display");

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, content_res.x());

        // Change raster coordinates from [-1, 1] to [0, 1].
        glMatrixMode(GL_PROJECTION);
        gluOrtho2D(0.0, 1.0, 0.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
    }

    // Get the window resolution and the scaling factor to scale the content to the
    // window.
    const Eigen::Vector2i window_res =
        Eigen::Vector2i(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
    const float width_factor = (float) window_res.x() / content_res.x();
    const float height_factor = (float) window_res.y() / content_res.y();
    const float factor = std::min(width_factor, height_factor);

    glViewport(0, 0, window_res.x(), window_res.y());

    glClear(GL_COLOR_BUFFER_BIT);

    if (scene != nullptr) {
        // Draw the image at the top left.
        glRasterPos2i(0, 1);
        // Scale the image and flip it up-down.
        glPixelZoom(factor, -factor);
        glDrawPixels(res.x(), res.y(), gl<T>::format, gl<T>::type, scene);
    }

    glutSwapBuffers();
}

#else  // SE_GLUT

template<typename T>
void drawit(const T*, const Eigen::Vector2i&)
{
    // Don't draw anything if GLUT isn't available.
}

#endif // SE_GLUT

#endif // SE_DRAW_HPP
