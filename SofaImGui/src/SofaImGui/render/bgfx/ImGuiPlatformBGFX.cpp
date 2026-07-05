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
#include <SofaImGui/render/bgfx/ImGuiPlatformBGFX.h>
#include <SofaImGui/render/ImGuiPlatformFactory.h>
#include <SofaImGui/imgui_impl_bgfx.h>

#include <backends/imgui_impl_glfw.h>
#include <bgfx/bgfx.h>

#include <sofa/helper/io/STBImage.h>

#include <algorithm>
#include <cstring>

namespace sofaimgui::render
{

// View ids used on the backbuffer (must not collide with the scene views 0/1
// used by the scene renderer, nor with the offscreen framebuffer).
namespace
{
    constexpr uint16_t kViewImGuiClear = 254;
    constexpr uint16_t kViewImGui = 255;
    constexpr uint16_t kViewScreenshotBlit = 253;
}

ImGuiPlatformBGFX::~ImGuiPlatformBGFX()
{
    if (m_sceneFB.idx != UINT16_MAX)
    {
        bgfx_destroy_frame_buffer(m_sceneFB);
        m_sceneFB.idx = UINT16_MAX;
        m_sceneFBTexture.idx = UINT16_MAX;
    }
    if (m_readbackTexture.idx != UINT16_MAX)
    {
        bgfx_destroy_texture(m_readbackTexture);
        m_readbackTexture.idx = UINT16_MAX;
    }
}

void ImGuiPlatformBGFX::initBackend(GLFWwindow* window)
{
    ImGui_ImplGlfw_InitForOther(window, true);
    ImGui_Implbgfx_Init(kViewImGui);
    bgfx_set_view_clear(kViewImGui, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx_set_view_mode(kViewImGui, BGFX_VIEW_MODE_SEQUENTIAL);
}

void ImGuiPlatformBGFX::newFrame()
{
    ImGui_Implbgfx_NewFrame();
}

void ImGuiPlatformBGFX::renderDrawData(ImDrawData* drawData)
{
    const uint16_t w = static_cast<uint16_t>(drawData->DisplaySize.x * drawData->FramebufferScale.x);
    const uint16_t h = static_cast<uint16_t>(drawData->DisplaySize.y * drawData->FramebufferScale.y);

    // Clear backbuffer before ImGui (scene lives in the offscreen FB)
    bgfx_set_view_rect(kViewImGuiClear, 0, 0, w, h);
    bgfx_set_view_clear(kViewImGuiClear, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);
    bgfx_touch(kViewImGuiClear);

    bgfx_set_view_rect(kViewImGui, 0, 0, w, h);
    bgfx_touch(kViewImGui);
    ImGui_Implbgfx_RenderDrawLists(drawData);
}

void ImGuiPlatformBGFX::shutdown()
{
    if (m_sceneFB.idx != UINT16_MAX)
    {
        bgfx_destroy_frame_buffer(m_sceneFB);
        m_sceneFB.idx = UINT16_MAX;
        m_sceneFBTexture.idx = UINT16_MAX;
    }
    if (m_readbackTexture.idx != UINT16_MAX)
    {
        bgfx_destroy_texture(m_readbackTexture);
        m_readbackTexture.idx = UINT16_MAX;
    }
    ImGui_Implbgfx_Shutdown();
}

void ImGuiPlatformBGFX::recreateFontsTexture()
{
    ImGui_Implbgfx_DestroyFontsTexture();
    ImGui_Implbgfx_CreateFontsTexture();
}

void ImGuiPlatformBGFX::recreateSceneFB(uint16_t width, uint16_t height)
{
    if (m_sceneFB.idx != UINT16_MAX)
    {
        bgfx_destroy_frame_buffer(m_sceneFB);
        m_sceneFB.idx = UINT16_MAX;
        m_sceneFBTexture.idx = UINT16_MAX;
    }

    if (width == 0 || height == 0)
        return;

    bgfx_texture_handle_t textures[2];
    textures[0] = bgfx_create_texture_2d(width, height, false, 1,
        BGFX_TEXTURE_FORMAT_RGBA8, BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP, NULL, 0);
    textures[1] = bgfx_create_texture_2d(width, height, false, 1,
        BGFX_TEXTURE_FORMAT_D24S8, BGFX_TEXTURE_RT_WRITE_ONLY, NULL, 0);

    m_sceneFB = bgfx_create_frame_buffer_from_handles(2, textures, true);
    m_sceneFBTexture = textures[0];
    m_sceneFBWidth = width;
    m_sceneFBHeight = height;
}

void ImGuiPlatformBGFX::beginSceneTarget(int width, int height)
{
    const uint16_t desiredW = static_cast<uint16_t>(std::max(1, width));
    const uint16_t desiredH = static_cast<uint16_t>(std::max(1, height));

    if (desiredW != m_sceneFBWidth || desiredH != m_sceneFBHeight)
        recreateSceneFB(desiredW, desiredH);

    if (m_sceneFB.idx != UINT16_MAX)
    {
        // Scene views 0 (background) and 1 (scene) render into the offscreen FB.
        bgfx_set_view_frame_buffer(0, m_sceneFB);
        bgfx_set_view_frame_buffer(1, m_sceneFB);
    }
}

void ImGuiPlatformBGFX::endSceneTarget()
{
    // bgfx submits are deferred to present(); nothing to unbind explicitly.
}

ImTextureID ImGuiPlatformBGFX::sceneTexture() const
{
    if (m_sceneFBTexture.idx == UINT16_MAX)
        return static_cast<ImTextureID>(0);
    return static_cast<ImTextureID>(m_sceneFBTexture.idx);
}

ImGuiDockNodeFlags ImGuiPlatformBGFX::dockspaceFlags() const
{
    // bgfx draws the scene into the central node via ImGui::Image, so it must
    // not be a pass-through node.
    return ImGuiDockNodeFlags_None;
}

void ImGuiPlatformBGFX::requestScreenshot(const std::string& path)
{
    m_pendingScreenshotPath = path;
}

void ImGuiPlatformBGFX::pumpScreenshot(uint32_t presentedFrame)
{
    m_lastPresentedFrame = presentedFrame;

    // Kick off a read-back once a screenshot is requested and none is in flight.
    if (!m_pendingScreenshotPath.empty() && m_sceneFBTexture.idx != UINT16_MAX && !m_readbackPending)
    {
        if (m_readbackTexture.idx == UINT16_MAX
            || m_readbackWidth != m_sceneFBWidth
            || m_readbackHeight != m_sceneFBHeight)
        {
            if (m_readbackTexture.idx != UINT16_MAX)
                bgfx_destroy_texture(m_readbackTexture);

            m_readbackTexture = bgfx_create_texture_2d(
                m_sceneFBWidth, m_sceneFBHeight, false, 1,
                BGFX_TEXTURE_FORMAT_RGBA8,
                BGFX_TEXTURE_BLIT_DST | BGFX_TEXTURE_READ_BACK, NULL, 0);
            m_readbackWidth = m_sceneFBWidth;
            m_readbackHeight = m_sceneFBHeight;
        }

        bgfx_blit(kViewScreenshotBlit, m_readbackTexture, 0, 0, 0, 0,
            m_sceneFBTexture, 0, 0, 0, 0,
            m_sceneFBWidth, m_sceneFBHeight, 0);

        m_readbackData.resize(static_cast<size_t>(m_readbackWidth) * m_readbackHeight * 4);
        m_readbackFrame = bgfx_read_texture(m_readbackTexture, m_readbackData.data(), 0, 0);
        m_readbackPending = true;
    }

    if (m_readbackPending && presentedFrame >= m_readbackFrame)
        processScreenshotReadback();
}

void ImGuiPlatformBGFX::processScreenshotReadback()
{
    m_readbackPending = false;

    sofa::helper::io::STBImage image;
    image.init(m_readbackWidth, m_readbackHeight, 1, 1,
        sofa::helper::io::Image::DataType::UINT32,
        sofa::helper::io::Image::ChannelFormat::RGBA);

    const uint8_t* src = m_readbackData.data();
    uint8_t* dst = image.getPixels();
    const uint32_t pitch = m_readbackWidth * 4;

    for (uint32_t row = 0; row < m_readbackHeight; ++row)
    {
        uint32_t srcRow = m_readbackHeight - 1 - row;
        memcpy(dst + row * pitch, src + srcRow * pitch, pitch);
    }

    image.save(m_pendingScreenshotPath.c_str(), 90);
    m_pendingScreenshotPath.clear();
    m_readbackData.clear();
}

sofa::type::Vec2i ImGuiPlatformBGFX::readSceneTargetPixels(std::vector<uint8_t>& pixels)
{
    // bgfx read-back is asynchronous; synchronous per-frame capture (for video
    // recording) is not supported on this backend.
    SOFA_UNUSED(pixels);
    return {0, 0};
}

// Self-register at static-init time.
static const ImGuiPlatformRegistrar s_registrar(
    sofaglfw::render::RenderAPI::BGFX,
    [] { return std::unique_ptr<IImGuiPlatform>(new ImGuiPlatformBGFX()); });

} // namespace sofaimgui::render
