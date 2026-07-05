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

#include <bgfx/c99/bgfx.h>

namespace sofaimgui::render
{

/// bgfx implementation of IImGuiPlatform. The scene is rendered into an
/// offscreen framebuffer (m_sceneFB) whose color texture is shown inside the
/// viewport window; screenshots are captured via asynchronous read-back.
class ImGuiPlatformBGFX : public IImGuiPlatform
{
public:
    ImGuiPlatformBGFX() = default;
    ~ImGuiPlatformBGFX() override;

    void initBackend(GLFWwindow* window) override;
    void newFrame() override;
    void renderDrawData(ImDrawData* drawData) override;
    void shutdown() override;
    void recreateFontsTexture() override;

    void beginSceneTarget(int width, int height) override;
    void endSceneTarget() override;
    ImTextureID sceneTexture() const override;
    bool sceneTextureFlippedV() const override { return false; }

    ImGuiDockNodeFlags dockspaceFlags() const override;
    bool supportsMultiViewport() const override { return false; }

    void requestScreenshot(const std::string& path) override;
    void pumpScreenshot(uint32_t presentedFrame) override;
    sofa::type::Vec2i readSceneTargetPixels(std::vector<uint8_t>& pixels) override;

private:
    void recreateSceneFB(uint16_t width, uint16_t height);
    void processScreenshotReadback();

    bgfx_frame_buffer_handle_t m_sceneFB{UINT16_MAX};
    bgfx_texture_handle_t m_sceneFBTexture{UINT16_MAX};
    uint16_t m_sceneFBWidth{0};
    uint16_t m_sceneFBHeight{0};

    bgfx_texture_handle_t m_readbackTexture{UINT16_MAX};
    uint16_t m_readbackWidth{0};
    uint16_t m_readbackHeight{0};
    std::vector<uint8_t> m_readbackData;
    uint32_t m_readbackFrame{0};
    bool m_readbackPending{false};

    std::string m_pendingScreenshotPath;
    uint32_t m_lastPresentedFrame{0};
    GLFWwindow* m_window{nullptr};
};

} // namespace sofaimgui::render
