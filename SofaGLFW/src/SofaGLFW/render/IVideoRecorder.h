/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <SofaGLFW/config.h>

#include <cstdint>
#include <memory>
#include <string>

namespace sofaglfw::render
{

/// Backend-agnostic video recorder abstraction. Only the OpenGL backend
/// provides a concrete implementation (wrapping sofa::gl::VideoRecorderFFMPEG),
/// because synchronous framebuffer read-back is required. When no implementation
/// is registered (e.g. bgfx-only build), createVideoRecorder() returns nullptr
/// and video recording is silently unavailable.
class SOFAGLFW_API IVideoRecorder
{
public:
    virtual ~IVideoRecorder() = default;

    virtual bool init(const std::string& ffmpegExecPath, int width, int height,
                      unsigned int framerate, unsigned int bitrate,
                      const std::string& codecExtension,
                      const std::string& codecName) = 0;

    virtual void addFrame(const uint8_t* pixels, int width, int height) = 0;
    virtual void finish() = 0;
};

/// @return a video recorder if a backend provides one, else nullptr.
SOFAGLFW_API std::unique_ptr<IVideoRecorder> createVideoRecorder();

/// Registration hook used by the concrete (OpenGL) implementation's TU.
using VideoRecorderCreator = std::unique_ptr<IVideoRecorder> (*)();
SOFAGLFW_API void registerVideoRecorder(VideoRecorderCreator creator);

struct SOFAGLFW_API VideoRecorderRegistrar
{
    explicit VideoRecorderRegistrar(VideoRecorderCreator creator)
    {
        registerVideoRecorder(creator);
    }
};

} // namespace sofaglfw::render
