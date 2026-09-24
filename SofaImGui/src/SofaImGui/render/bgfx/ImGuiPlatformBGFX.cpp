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
#include <BGFXPlugin/Context.h>
#include <SofaGLFW/render/bgfx/SceneRendererBGFX.h>

#include <backends/imgui_impl_glfw.h>
#include <GLFW/glfw3.h>

#include <SofaGLFW/render/bgfx/BgfxCallback.h> // saveRgba8Screenshot
#include <sofa/helper/logging/Messaging.h>

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
    destroySceneFB();
    destroyReadbackTexture();
}

void ImGuiPlatformBGFX::destroySceneFB()
{
    if (m_sceneFB.idx != UINT16_MAX && bgfxplugin::context::isAlive())
        bgfx_destroy_frame_buffer(m_sceneFB);
    m_sceneFB.idx = UINT16_MAX;
    m_sceneFBTexture.idx = UINT16_MAX;
}

void ImGuiPlatformBGFX::destroyReadbackTexture()
{
    if (m_readbackTexture.idx != UINT16_MAX && bgfxplugin::context::isAlive())
        bgfx_destroy_texture(m_readbackTexture);
    m_readbackTexture.idx = UINT16_MAX;
}

void ImGuiPlatformBGFX::initBackend(GLFWwindow* window)
{
    m_window = window;
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
    bgfx_set_view_rect(kViewImGuiClear, 0, 0, w, h, 0.0f, 1.0f);
    bgfx_set_view_clear(kViewImGuiClear, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);
    bgfx_touch(kViewImGuiClear);

    bgfx_set_view_rect(kViewImGui, 0, 0, w, h, 0.0f, 1.0f);
    bgfx_touch(kViewImGui);
    ImGui_Implbgfx_RenderDrawLists(drawData);
}

void ImGuiPlatformBGFX::shutdown()
{
    // Complete the screenshots requested just before closing ("save, then quit"): a
    // read-back writes into m_readbackData when bgfx next renders, at the latest in
    // bgfx_shutdown, after this platform is destroyed. It takes a couple of frames.
    if (bgfxplugin::context::isAlive())
    {
        for (int frame = 0; frame < 8 && (m_readbackPending || !m_screenshotQueue.empty()); ++frame)
            pumpScreenshot(bgfx_frame(false));
        if (!m_screenshotQueue.empty() || m_readbackPending)
            msg_warning("ImGuiPlatformBGFX") << "Screenshots requested before closing were not saved.";
    }
    if (m_readbackPending)
    {
        // Still in flight: keep its buffer alive for bgfx to write into.
        static std::vector<std::vector<uint8_t>> s_inFlightReadbacks;
        s_inFlightReadbacks.push_back(std::move(m_readbackData));
        m_readbackPending = false;
    }

    // Normally runs before the backend shuts bgfx down; after that, the handles are gone.
    destroySceneFB();
    destroyReadbackTexture();
    if (bgfxplugin::context::isAlive())
        ImGui_Implbgfx_Shutdown();
}

void ImGuiPlatformBGFX::recreateFontsTexture()
{
    ImGui_Implbgfx_DestroyFontsTexture();
    ImGui_Implbgfx_CreateFontsTexture();
}

void ImGuiPlatformBGFX::recreateSceneFB(uint16_t width, uint16_t height, int msaa)
{
    destroySceneFB();

    if (width == 0 || height == 0)
        return;

    // Depth format: D24S8 is not a render target everywhere (Apple GPUs through
    // MoltenVK, AMD on Vulkan); take the first one the renderer can render into.
    const bgfx_caps_t* caps = bgfx_get_caps();
    bgfx_texture_format_t depthFormat = BGFX_TEXTURE_FORMAT_D24S8;
    for (const bgfx_texture_format_t candidate : { BGFX_TEXTURE_FORMAT_D24S8, BGFX_TEXTURE_FORMAT_D32F,
                                                   BGFX_TEXTURE_FORMAT_D24, BGFX_TEXTURE_FORMAT_D16 })
    {
        if (caps->formats[candidate] & BGFX_CAPS_FORMAT_TEXTURE_FRAMEBUFFER)
        {
            depthFormat = candidate;
            break;
        }
    }

    // Multisampling (the Settings' MSAA): the color target is resolved by bgfx before
    // the viewport samples it. Only when both formats can be multisampled targets.
    uint64_t rtFlag = BGFX_TEXTURE_RT;
    const bool canMultisample = (caps->formats[BGFX_TEXTURE_FORMAT_RGBA8] & BGFX_CAPS_FORMAT_TEXTURE_FRAMEBUFFER_MSAA)
                             && (caps->formats[depthFormat] & BGFX_CAPS_FORMAT_TEXTURE_FRAMEBUFFER_MSAA);
    if (canMultisample)
    {
        switch (msaa)
        {
        case 2:  rtFlag = BGFX_TEXTURE_RT_MSAA_X2;  break;
        case 4:  rtFlag = BGFX_TEXTURE_RT_MSAA_X4;  break;
        case 8:  rtFlag = BGFX_TEXTURE_RT_MSAA_X8;  break;
        case 16: rtFlag = BGFX_TEXTURE_RT_MSAA_X16; break;
        default: break;
        }
    }

    bgfx_texture_handle_t textures[2];
    textures[0] = bgfx_create_texture_2d(width, height, false, 1,
        BGFX_TEXTURE_FORMAT_RGBA8, rtFlag | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP, NULL, 0);
    textures[1] = bgfx_create_texture_2d(width, height, false, 1,
        depthFormat, rtFlag | BGFX_TEXTURE_RT_WRITE_ONLY, NULL, 0);

    m_sceneFB = bgfx_create_frame_buffer_from_handles(2, textures, true);
    if (m_sceneFB.idx == UINT16_MAX)
        msg_error("ImGuiPlatformBGFX") << "Could not create the " << width << "x" << height
                                       << " scene frame buffer: the viewport stays empty.";
    m_sceneFBTexture = textures[0];
    m_sceneFBWidth = width;
    m_sceneFBHeight = height;
    m_sceneFBMsaa = msaa;
}

void ImGuiPlatformBGFX::beginSceneTarget(int width, int height, int msaa)
{
    // width/height are window units; SceneRendererBGFX sets the scene view rect in
    // framebuffer pixels (times the framebuffer scale), so the target must be that
    // large too, or a HiDPI window only shows the top-left part of the scene.
    float xscale = 1.0f, yscale = 1.0f;
    sofaglfw::render::SceneRendererBGFX::framebufferScale(m_window, xscale, yscale);
    const uint16_t desiredW = static_cast<uint16_t>(std::max(1, static_cast<int>(std::max(1, width) * xscale)));
    const uint16_t desiredH = static_cast<uint16_t>(std::max(1, static_cast<int>(std::max(1, height) * yscale)));

    if (desiredW != m_sceneFBWidth || desiredH != m_sceneFBHeight || msaa != m_sceneFBMsaa)
        recreateSceneFB(desiredW, desiredH, msaa);

    if (m_sceneFB.idx != UINT16_MAX)
    {
        // Every scene view (background, scene, transparent models, overlays) renders
        // into the offscreen FB.
        for (uint16_t view = 0; view < sofaglfw::render::SceneRendererBGFX::kSceneViewCount; ++view)
            bgfx_set_view_frame_buffer(view, m_sceneFB);
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
    m_screenshotQueue.push_back(path);
}

void ImGuiPlatformBGFX::pumpScreenshot(uint32_t presentedFrame)
{
    // Kick off a read-back once a screenshot is requested and none is in flight.
    if (!m_screenshotQueue.empty() && m_sceneFBTexture.idx != UINT16_MAX && !m_readbackPending)
    {
        m_readbackPath = m_screenshotQueue.front();
        m_screenshotQueue.pop_front();

        if (m_readbackTexture.idx == UINT16_MAX
            || m_readbackWidth != m_sceneFBWidth
            || m_readbackHeight != m_sceneFBHeight)
        {
            destroyReadbackTexture();

            m_readbackTexture = bgfx_create_texture_2d(
                m_sceneFBWidth, m_sceneFBHeight, false, 1,
                BGFX_TEXTURE_FORMAT_RGBA8,
                BGFX_TEXTURE_BLIT_DST | BGFX_TEXTURE_READ_BACK, NULL, 0);
            m_readbackWidth = m_sceneFBWidth;
            m_readbackHeight = m_sceneFBHeight;
        }

        // bgfx API >= 157: blit/readback work on texture regions (0 extents = whole mip).
        bgfx_texture_region_t dst{};
        dst.handle = m_readbackTexture;
        dst.width = m_sceneFBWidth;
        dst.height = m_sceneFBHeight;
        bgfx_texture_region_t src{};
        src.handle = m_sceneFBTexture;
        src.width = m_sceneFBWidth;
        src.height = m_sceneFBHeight;
        bgfx_blit(kViewScreenshotBlit, &dst, &src);

        m_readbackData.resize(static_cast<size_t>(m_readbackWidth) * m_readbackHeight * 4);
        m_readbackFrame = bgfx_read_texture(&dst, m_readbackData.data());
        m_readbackPending = true;
    }

    if (m_readbackPending && presentedFrame >= m_readbackFrame)
        processScreenshotReadback();
}

bool ImGuiPlatformBGFX::sceneTextureFlippedV() const
{
    return bgfxplugin::context::isAlive() && bgfx_get_caps()->originBottomLeft;
}

void ImGuiPlatformBGFX::processScreenshotReadback()
{
    m_readbackPending = false;

    // The texture is top-down, except on renderers whose targets start at the bottom left.
    sofaglfw::render::saveRgba8Screenshot(m_readbackPath.c_str(), m_readbackWidth, m_readbackHeight,
                                          m_readbackWidth * 4u, m_readbackData.data(), /*bgra*/ false,
                                          /*bottomUp*/ sceneTextureFlippedV());
    m_readbackPath.clear();
    m_readbackData.clear();
}

sofa::type::Vec2i ImGuiPlatformBGFX::readSceneTargetPixels(std::vector<uint8_t>& pixels)
{
    // bgfx read-back is asynchronous; synchronous per-frame capture (for video
    // recording) is not supported on this backend.
    pixels.clear();
    return {0, 0};
}

// Self-register at static-init time.
static const ImGuiPlatformRegistrar s_registrar(
    sofaglfw::render::RenderAPI::BGFX,
    [] { return std::unique_ptr<IImGuiPlatform>(new ImGuiPlatformBGFX()); });

} // namespace sofaimgui::render
