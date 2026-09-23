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
#include <sofa/helper/logging/Messaging.h>

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
    // The window's user pointer is the GUI (set before the engine is initialized).
    if (auto* gui = static_cast<SofaGLFWBaseGUI*>(glfwGetWindowUserPointer(window)))
        m_backend = gui->getRenderBackend();
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
    // The scene is drawn straight into the backbuffer, in the units its renderer expects.
    sofa::type::Vec2i size;
    if (m_backend)
        size = m_backend->backbufferViewportSize(window);
    else
        glfwGetFramebufferSize(window, &size[0], &size[1]);
    sofa::core::visual::VisualParams::defaultInstance()->viewport() = {0, 0, size[0], size[1]};
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
    // {0, 0} when the backend has no synchronous read-back: no frame is recorded.
    if (!m_backend)
    {
        pixels.clear();
        return {0, 0};
    }
    return m_backend->readBackbufferPixels(m_window, pixels);
}

void NullGUIEngine::saveNamedScreenshot(SofaGLFWBaseGUI* baseGUI, std::string filename, int compression_level)
{
    render::IRenderBackend* backend = baseGUI ? baseGUI->getRenderBackend() : m_backend;
    if (!backend || !backend->requestBackbufferScreenshot(m_window, filename, compression_level))
        msg_error("NullGUIEngine") << "Could not save the screenshot " << filename;
}

} // namespace sofaglfw
