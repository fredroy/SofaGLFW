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

#include <SofaImGui/render/IImGuiPlatform.h>

#include <sofa/gl/gl.h> // GLuint, GLEW
#include <sofa/gl/FrameBufferObject.h>

#include <memory>
#include <string>
#include <utility>

namespace sofaimgui::render
{

/// Legacy OpenGL implementation of IImGuiPlatform. The scene is rendered into a
/// sofa::gl::FrameBufferObject whose color texture is shown in the viewport;
/// screenshots read the FBO texture back synchronously, and video capture uses
/// double-buffered PBOs (ported from ImGuiGUIEngine's GL path).
class ImGuiPlatformGL : public IImGuiPlatform
{
public:
    ImGuiPlatformGL() = default;
    ~ImGuiPlatformGL() override;

    void initBackend(GLFWwindow* window) override;
    void newFrame() override;
    void renderDrawData(ImDrawData* drawData) override;
    void shutdown() override;
    void recreateFontsTexture() override;

    void beginSceneTarget(int width, int height) override;
    void endSceneTarget() override;
    ImTextureID sceneTexture() const override;
    bool sceneTextureFlippedV() const override { return true; }

    ImGuiDockNodeFlags dockspaceFlags() const override;
    bool supportsMultiViewport() const override { return true; }

    void requestScreenshot(const std::string& path) override;
    void pumpScreenshot(uint32_t presentedFrame) override;
    sofa::type::Vec2i readSceneTargetPixels(std::vector<uint8_t>& pixels) override;

private:
    std::unique_ptr<sofa::gl::FrameBufferObject> m_fbo;
    std::pair<unsigned int, unsigned int> m_currentFBOSize{0, 0};

    static inline constexpr int s_NB_PBOS = 2;
    GLuint m_pbos[s_NB_PBOS]{0, 0};
    sofa::type::Vec2i m_pboSize;
    std::size_t m_frameCount{0};
    bool m_pbosInitialized{false};
};

} // namespace sofaimgui::render
