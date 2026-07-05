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
#include <SofaGLFW/render/IVideoRecorder.h>

#include <sofa/gl/VideoRecorderFFMPEG.h>

namespace sofaglfw::render
{

/// OpenGL-backed video recorder wrapping sofa::gl::VideoRecorderFFMPEG.
class VideoRecorderGL : public IVideoRecorder
{
public:
    bool init(const std::string& ffmpegExecPath, int width, int height,
              unsigned int framerate, unsigned int bitrate,
              const std::string& codecExtension,
              const std::string& codecName) override
    {
        const std::string videoFilename =
            m_recorder.findFilename(framerate, bitrate / 1024, codecExtension);
        return m_recorder.init(ffmpegExecPath, videoFilename, width, height,
                               framerate, bitrate, codecName);
    }

    void addFrame(const uint8_t* pixels, int width, int height) override
    {
        m_recorder.addFrame(pixels, width, height);
    }

    void finish() override
    {
        m_recorder.finishVideo();
    }

private:
    sofa::gl::VideoRecorderFFMPEG m_recorder;
};

// Register the OpenGL video recorder so SofaGLFWBaseGUI can create one when the
// OpenGL backend is compiled in.
static const VideoRecorderRegistrar s_videoRecorderRegistrar(
    [] { return std::unique_ptr<IVideoRecorder>(new VideoRecorderGL()); });

} // namespace sofaglfw::render
