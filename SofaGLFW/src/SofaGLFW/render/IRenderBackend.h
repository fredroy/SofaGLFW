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

struct GLFWwindow;

namespace sofa::helper::visual
{
    class DrawTool;
}

namespace sofaglfw::render
{

/// Backend-agnostic interface owning the rendering engine lifetime and frame
/// presentation. Concrete implementations (OpenGL, bgfx) live in their own
/// translation units and are the *only* place their respective graphics API
/// headers are included. Shared GUI code talks exclusively to this interface,
/// which is how the plugins avoid #ifdef in their logic.
class SOFAGLFW_API IRenderBackend
{
public:
    virtual ~IRenderBackend() = default;

    /// Initialize the graphics engine on the given window. @return success.
    virtual bool initEngine(GLFWwindow* window, uint32_t width, uint32_t height) = 0;

    /// React to a framebuffer/window resize (in framebuffer pixels).
    virtual void resize(uint32_t width, uint32_t height) = 0;

    /// Present the current frame (bgfx_frame / glfwSwapBuffers).
    /// @return a monotonically increasing frame number (bgfx frame counter). GL
    /// returns its own counter; the value is only meaningful for matching an
    /// asynchronous readback request against a completed frame.
    virtual uint32_t present(GLFWwindow* window) = 0;

    /// Shut the engine down. Safe to call multiple times.
    virtual void terminate() = 0;

    /// Request a screenshot of the default backbuffer, saved to @p path. Used by
    /// the headless/Null engine which has no offscreen viewport target. May be
    /// asynchronous; the backend completes it on the next present().
    /// @return true if the request was accepted.
    virtual bool requestBackbufferScreenshot(GLFWwindow* window, const std::string& path) = 0;

    /// Create the DrawTool matching this backend (DrawToolGL / DrawToolBGFX).
    virtual std::unique_ptr<sofa::helper::visual::DrawTool> makeDrawTool() = 0;

    /// Configure VisualParams for this backend (e.g. setSupported(API_OpenGL)).
    virtual void configureVisualParams() = 0;

    /// Register the ObjectFactory aliases mapping VisualModel/OglModel to the
    /// concrete visual model of this backend.
    virtual void registerVisualModelAliases() = 0;

    // --- capabilities -------------------------------------------------------

    /// True if ImGui multi-viewport (ImGuiConfigFlags_ViewportsEnable) is usable.
    virtual bool supportsMultiViewport() const = 0;

    /// GLFW client-API hint: true if the window needs an OpenGL context created
    /// by GLFW; false if the backend manages the surface itself (bgfx).
    virtual bool needsGlfwContext() const = 0;

    // --- runtime knobs ------------------------------------------------------

    virtual void setVsync(bool enabled) = 0;
    virtual bool isVsync() const = 0;
    virtual void setMsaa(int level) = 0;
    virtual int  getMsaa() const = 0;
};

} // namespace sofaglfw::render
