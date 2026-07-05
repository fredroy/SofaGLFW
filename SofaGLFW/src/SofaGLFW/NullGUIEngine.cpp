/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaGLFW/config.h>
#include <SofaGLFW/NullGUIEngine.h>
#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <SofaGLFW/render/IRenderBackend.h>
#include <sofa/core/visual/VisualParams.h>
#include <GLFW/glfw3.h>

#include <sofa/helper/io/File.h>
#include <sofa/helper/io/STBImage.h>

namespace sofaglfw
{

void NullGUIEngine::init()
{
    m_lastTime = glfwGetTime();
    m_lastDisplayTime = m_lastTime;
    m_avgFrameTime = 0.0;
}
void NullGUIEngine::initBackend(GLFWwindow* window)
{
    m_window = window;
}
void NullGUIEngine::startFrame(SofaGLFWBaseGUI* baseGUI)
{
    if (baseGUI && baseGUI->getRenderBackend())
        baseGUI->getRenderBackend()->present(m_window);
}
void NullGUIEngine::endFrame()
{
    constexpr double displayRefreshInterval = 0.1;
    constexpr double smoothingFactor = 0.05;

    const double now = glfwGetTime();
    const double dt = now - m_lastTime;
    m_lastTime = now;

    if (dt > 0.0)
    {
        if (m_avgFrameTime <= 0.0)
            m_avgFrameTime = dt;
        else
            m_avgFrameTime += smoothingFactor * (dt - m_avgFrameTime);
    }

    if (now - m_lastDisplayTime >= displayRefreshInterval)
    {
        const double fps = (m_avgFrameTime > 0.0) ? 1.0 / m_avgFrameTime : 0.0;
        char title_string[32];
        std::snprintf(title_string, sizeof(title_string), "FPS: %.1f", fps);
        glfwSetWindowTitle(m_window, title_string);
        m_lastDisplayTime = now;
    }
}

void NullGUIEngine::beforeDraw(GLFWwindow* window)
{
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    sofa::core::visual::VisualParams::defaultInstance()->viewport() = {0, 0, width, height};
}

void NullGUIEngine::terminate()
{

}

bool NullGUIEngine::dispatchMouseEvents()
{
    return true;
}

void NullGUIEngine::resetCounter()
{

}

sofa::type::Vec2i NullGUIEngine::getFrameBufferPixels(std::vector<uint8_t>& pixels)
{
    int width, height;
    glfwGetFramebufferSize(m_window, &width, &height);
    pixels.resize(width * height * 4, 0);
    return {width, height};
}

void NullGUIEngine::saveNamedScreenshot(SofaGLFWBaseGUI* baseGUI, std::string filename, int compression_level)
{
    SOFA_UNUSED(compression_level);
    if (baseGUI && baseGUI->getRenderBackend())
        baseGUI->getRenderBackend()->requestBackbufferScreenshot(m_window, filename);
}

} // namespace sofaglfw
