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
#include <SofaGLFW/render/bgfx/RenderBackendBGFX.h>
#include <SofaGLFW/render/bgfx/BgfxScreenshotCallback.h>
#include <SofaGLFW/render/bgfx/BgfxNativeWindow.h>
#include <SofaGLFW/render/RenderBackendFactory.h>
#include <SofaGLFW/render/bgfx/SceneRendererBGFX.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/helper/logging/Messaging.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

#include <bgfx/bgfx.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>

#include <BGFXPlugin/Context.h>
#include <BGFXPlugin/DrawToolBGFX.h>
#include <BGFXPlugin/init.h>

namespace sofaglfw::render
{

namespace
{
// Since bgfx API 160 these options describe the swap chain instead of being reset flags
// (their bit values are shared, so the same mask can be split both ways).
constexpr uint32_t kSwapChainFlags = 0
    | BGFX_SWAP_CHAIN_FULLSCREEN_MASK
    | BGFX_SWAP_CHAIN_MSAA_MASK
    | BGFX_SWAP_CHAIN_SRGB_BACKBUFFER
    | BGFX_SWAP_CHAIN_HDR10
    | BGFX_SWAP_CHAIN_HIDPI
    | BGFX_SWAP_CHAIN_TRANSPARENT_BACKBUFFER
    ;
} // namespace

RenderBackendBGFX::RenderBackendBGFX() = default;

RenderBackendBGFX::~RenderBackendBGFX()
{
    terminate();
}

namespace
{

/// Renderer requested with SOFA_BGFX_RENDERER (e.g. vulkan to run the Linux/Windows
/// path on macOS through MoltenVK), or @p fallback when unset or unavailable.
bgfx_renderer_type rendererFromEnvironment(bgfx_renderer_type fallback)
{
    const char* env = std::getenv("SOFA_BGFX_RENDERER");
    if (!env || !*env)
        return fallback;

    std::string name(env);
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static const std::pair<const char*, bgfx_renderer_type> kNames[] = {
        { "noop", BGFX_RENDERER_TYPE_NOOP },
        { "d3d11", BGFX_RENDERER_TYPE_DIRECT3D11 },
        { "d3d12", BGFX_RENDERER_TYPE_DIRECT3D12 },
        { "metal", BGFX_RENDERER_TYPE_METAL },
        { "opengl", BGFX_RENDERER_TYPE_OPENGL },
        { "gl", BGFX_RENDERER_TYPE_OPENGL },
        { "opengles", BGFX_RENDERER_TYPE_OPENGLES },
        { "vulkan", BGFX_RENDERER_TYPE_VULKAN },
    };
    for (const auto& [key, type] : kNames)
    {
        if (name != key)
            continue;
        bgfx_renderer_type supported[BGFX_RENDERER_TYPE_COUNT];
        const uint8_t count = bgfx_get_supported_renderers(BGFX_RENDERER_TYPE_COUNT, supported);
        if (std::find(supported, supported + count, type) != supported + count)
            return type;
        msg_warning("RenderBackendBGFX") << "SOFA_BGFX_RENDERER=" << env
                                         << ": this renderer is not compiled in bgfx. Using the default.";
        return fallback;
    }
    msg_warning("RenderBackendBGFX") << "SOFA_BGFX_RENDERER=" << env
        << " is not a known renderer (noop, d3d11, d3d12, metal, opengl, opengles, vulkan). Using the default.";
    return fallback;
}

} // namespace

bool RenderBackendBGFX::initEngine(GLFWwindow* window, uint32_t width, uint32_t height)
{
    SOFA_UNUSED(width);
    SOFA_UNUSED(height);

    m_window = window;

    m_debug = BGFX_DEBUG_TEXT;
    // Keep the vsync/MSAA flags set before init (the persisted preferences).
    m_reset |= BGFX_RESET_HIDPI;

    bgfx_init_t init;
    bgfx_init_ctor(&init);

    m_type = rendererFromEnvironment(m_type);
    init.type = m_type;
    init.platformData.type = bgfxNativeWindowHandleType();
    init.debug = true;

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    // The window and its back buffer are described by a swap chain (bgfx API >= 160).
    // Sizes are in framebuffer pixels: bgfx no longer applies the content scale itself.
    init.swapChain.nwh = bgfxNativeWindowHandle(window);
    init.swapChain.ndt = bgfxNativeDisplayHandle();
    init.swapChain.width = static_cast<uint32_t>(fbWidth);
    init.swapChain.height = static_cast<uint32_t>(fbHeight);
    init.swapChain.flags = m_reset & kSwapChainFlags;
    init.reset = m_reset & ~kSwapChainFlags;
    init.callback = bgfxScreenshotCallback();

    // Per-frame transient geometry pool. bgfx's defaults (6 MB vertices, 2 MB indices)
    // are sized for small UIs; DrawToolBGFX streams all debug geometry through it
    // (collision models, force fields, many spheres...) and the GUI shares the same
    // pool. The pool is allocated once, for each of the two frames in flight.
    init.limits.maxTransientVbSize = 32u << 20;
    init.limits.maxTransientIbSize = 16u << 20;
    m_swapChain = init.swapChain;

    m_initialized = bgfx_init(&init);
    if (!m_initialized)
    {
        // bgfx has no context now: no bgfx call is valid.
        msg_error("RenderBackendBGFX") << "Could not initialize bgfx"
            << (m_type == BGFX_RENDERER_TYPE_COUNT ? std::string() : std::string(" with the ") + bgfx_get_renderer_name(m_type) + " renderer")
            << " (native window handle " << init.swapChain.nwh << ").";
        return false;
    }

    bgfxplugin::context::markInitialized();
    msg_info("RenderBackendBGFX") << "bgfx renderer: " << bgfx_get_renderer_name(bgfx_get_renderer_type());

    bgfx_set_debug(m_debug, BGFX_INVALID_HANDLE, 0);
    bgfx_set_view_clear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);
    return true;
}

void RenderBackendBGFX::resize(uint32_t width, uint32_t height)
{
    if (!m_initialized)
        return;
    m_swapChain.width = width;
    m_swapChain.height = height;
    m_swapChain.flags = m_reset & kSwapChainFlags;
    bgfx_reset(m_reset & ~kSwapChainFlags, &m_swapChain);
}

uint32_t RenderBackendBGFX::present(GLFWwindow* window)
{
    SOFA_UNUSED(window);
    return bgfx_frame(false);
}

void RenderBackendBGFX::terminate()
{
    if (!m_initialized)
        return;

    // The scene graph still owns BGFXModel components (and their textures), which
    // are destroyed after the GUI: shutdown() makes every such owner free its bgfx
    // handles first, so they are not destroyed later against a dead context.
    bgfxplugin::context::shutdown();
    m_initialized = false;
}

bool RenderBackendBGFX::requestBackbufferScreenshot(GLFWwindow* window, const std::string& path, int compressionLevel)
{
    SOFA_UNUSED(compressionLevel); // the bgfx callback writes PNG
    SOFA_UNUSED(window);
    // Screenshot of the default backbuffer; serviced by the bgfx callback on the
    // next frame.
    bgfx_frame_buffer_handle_t handle = BGFX_INVALID_HANDLE;
    bgfx_request_screen_shot(handle, path.c_str());
    return true;
}

sofa::type::Vec2i RenderBackendBGFX::backbufferViewportSize(GLFWwindow* window) const
{
    // Logical pixels: SceneRendererBGFX applies the window content scale.
    int width = 0, height = 0;
    glfwGetWindowSize(window, &width, &height);
    return { width, height };
}

sofa::type::Vec2i RenderBackendBGFX::readBackbufferPixels(GLFWwindow* window, std::vector<uint8_t>& pixels)
{
    // bgfx reads back asynchronously (screenshots): no synchronous read for video.
    SOFA_UNUSED(window);
    pixels.clear();
    return { 0, 0 };
}

std::unique_ptr<sofa::helper::visual::DrawTool> RenderBackendBGFX::makeDrawTool()
{
    return std::make_unique<bgfxplugin::DrawToolBGFX>();
}

void RenderBackendBGFX::configureVisualParams()
{
    // bgfx manages its own render state; nothing to advertise via API_OpenGL.
}

void RenderBackendBGFX::registerVisualModelAliases()
{
    bgfxplugin::registerComponents(sofa::core::ObjectFactory::getInstance());

    sofa::core::ObjectFactory::ClassEntry::SPtr classVisualModel;
    sofa::core::ObjectFactory::AddAlias("VisualModel", "BGFXModel", true, &classVisualModel);
    sofa::core::ObjectFactory::AddAlias("OglModel", "BGFXModel", true, &classVisualModel);

    // The bgfx counterparts of Sofa.GL components, under their GL names.
    for (const auto& [glName, bgfxName] : { std::pair{ "OglLabel", "BGFXLabel" },
                                            std::pair{ "OglSceneFrame", "BGFXSceneFrame" },
                                            std::pair{ "OglColorMap", "BGFXColorMap" },
                                            std::pair{ "DataDisplay", "BGFXDataDisplay" } })
    {
        sofa::core::ObjectFactory::ClassEntry::SPtr entry;
        sofa::core::ObjectFactory::AddAlias(glName, bgfxName, true, &entry);
    }
}

void RenderBackendBGFX::applyReset()
{
    if (!m_initialized || !m_window)
        return;
    int w, h;
    glfwGetFramebufferSize(m_window, &w, &h); // pixels, as initEngine
    m_swapChain.width = static_cast<uint32_t>(w);
    m_swapChain.height = static_cast<uint32_t>(h);
    m_swapChain.flags = m_reset & kSwapChainFlags;
    bgfx_reset(m_reset & ~kSwapChainFlags, &m_swapChain);
}

void RenderBackendBGFX::setVsync(bool enabled)
{
    if (enabled)
        m_reset |= BGFX_RESET_VSYNC;
    else
        m_reset &= ~BGFX_RESET_VSYNC;
    applyReset();
}

void RenderBackendBGFX::setMsaa(int level)
{
    m_reset &= ~BGFX_RESET_MSAA_MASK;
    switch (level)
    {
    case 2:  m_reset |= BGFX_RESET_MSAA_X2;  break;
    case 4:  m_reset |= BGFX_RESET_MSAA_X4;  break;
    case 8:  m_reset |= BGFX_RESET_MSAA_X8;  break;
    case 16: m_reset |= BGFX_RESET_MSAA_X16; break;
    default: break;
    }
    applyReset();
}

int RenderBackendBGFX::getMsaa() const
{
    const uint32_t msaa = m_reset & BGFX_RESET_MSAA_MASK;
    if (msaa == BGFX_RESET_MSAA_X16) return 16;
    if (msaa == BGFX_RESET_MSAA_X8)  return 8;
    if (msaa == BGFX_RESET_MSAA_X4)  return 4;
    if (msaa == BGFX_RESET_MSAA_X2)  return 2;
    return 0;
}

// Self-register this backend at static-init time.
static const RenderBackendRegistrar s_registrar(
    RenderAPI::BGFX,
    [] { return std::unique_ptr<IRenderBackend>(new RenderBackendBGFX()); },
    [] { return std::unique_ptr<ISceneRenderer>(new SceneRendererBGFX()); });

} // namespace sofaglfw::render
