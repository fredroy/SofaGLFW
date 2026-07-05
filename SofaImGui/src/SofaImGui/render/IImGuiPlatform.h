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

#include <SofaImGui/config.h>

#include <imgui.h> // ImTextureID, ImGuiDockNodeFlags, ImDrawData

#include <sofa/type/Vec.h>

#include <cstdint>
#include <string>
#include <vector>

struct GLFWwindow;

namespace sofaimgui::render
{

/// Backend-agnostic ImGui platform/renderer layer. Concrete implementations
/// (OpenGL2/3, bgfx) wrap the matching ImGui backend, the offscreen framebuffer
/// used to host the scene inside the viewport window, and screenshot readback.
/// This is what lets ImGuiGUIEngine.cpp drop every #if SOFAIMGUI_USE_BGFX.
class SOFAIMGUI_API IImGuiPlatform
{
public:
    virtual ~IImGuiPlatform() = default;

    // --- ImGui backend lifetime --------------------------------------------

    /// Initialize the ImGui platform (GLFW) and renderer backend.
    virtual void initBackend(GLFWwindow* window) = 0;

    /// Begin a new ImGui frame for the renderer backend (ImGui_Impl*_NewFrame).
    virtual void newFrame() = 0;

    /// Render the collected ImGui draw data to the backbuffer.
    virtual void renderDrawData(ImDrawData* drawData) = 0;

    /// Shut the renderer backend down.
    virtual void shutdown() = 0;

    /// (Re)build the ImGui fonts texture on the GPU after the atlas changed.
    virtual void recreateFontsTexture() = 0;

    // --- offscreen scene target --------------------------------------------

    /// Bind/prepare the offscreen render target sized to (width,height) in
    /// framebuffer pixels. The scene is drawn into it, then shown via
    /// sceneTexture() inside the viewport ImGui window.
    virtual void beginSceneTarget(int width, int height) = 0;

    /// Unbind the offscreen render target.
    virtual void endSceneTarget() = 0;

    /// ImGui texture id referencing the scene target's color attachment, for
    /// ImGui::Image(). Returns 0 when no valid target exists yet.
    virtual ImTextureID sceneTexture() const = 0;

    /// True if the scene texture is stored bottom-up and must be shown with a
    /// vertical UV flip (OpenGL FBO). bgfx render targets are top-down.
    virtual bool sceneTextureFlippedV() const = 0;

    // --- capabilities -------------------------------------------------------

    /// Dockspace flags to use for the central node (backends differ: the GL
    /// path uses a pass-through central node, bgfx draws into it).
    virtual ImGuiDockNodeFlags dockspaceFlags() const = 0;

    /// True if ImGui multi-viewport (ImGuiConfigFlags_ViewportsEnable) is usable
    /// with this platform backend.
    virtual bool supportsMultiViewport() const = 0;

    // --- screenshots --------------------------------------------------------

    /// Request that the current scene target be saved to @p path. May complete
    /// asynchronously; call pumpScreenshot() each frame to finish pending saves.
    virtual void requestScreenshot(const std::string& path) = 0;

    /// Advance any pending asynchronous screenshot readback. Call once per frame
    /// after present, passing the frame number returned by present() so the
    /// backend can tell when the read-back has completed. GL ignores it.
    virtual void pumpScreenshot(uint32_t presentedFrame) = 0;

    /// Read back the scene target pixels synchronously (used by video recording).
    /// @return {width,height}; empty (0,0) if unsupported by this backend.
    virtual sofa::type::Vec2i readSceneTargetPixels(std::vector<uint8_t>& pixels) = 0;
};

} // namespace sofaimgui::render
