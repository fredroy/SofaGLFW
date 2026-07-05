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

#include <bgfx/bgfx.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>

#include <BGFXPlugin/DrawToolBGFX.h>

namespace sofaglfw::render
{

RenderBackendBGFX::RenderBackendBGFX() = default;

RenderBackendBGFX::~RenderBackendBGFX()
{
    terminate();
}

bool RenderBackendBGFX::initEngine(GLFWwindow* window, uint32_t width, uint32_t height)
{
    SOFA_UNUSED(width);
    SOFA_UNUSED(height);

    m_window = window;

    m_debug = BGFX_DEBUG_TEXT;
    m_reset = BGFX_RESET_VSYNC | BGFX_RESET_HIDPI;

    bgfx_init_t init;
    bgfx_init_ctor(&init);

    init.type = m_type;
    init.platformData.nwh = bgfxNativeWindowHandle(window);
    init.platformData.ndt = bgfxNativeDisplayHandle();
    init.platformData.type = bgfxNativeWindowHandleType();
    init.debug = true;

    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    init.resolution.width = static_cast<uint32_t>(fbWidth);
    init.resolution.height = static_cast<uint32_t>(fbHeight);
    init.resolution.reset = m_reset;
    init.callback = bgfxScreenshotCallback();

    const bool res = bgfx_init(&init);

    bgfx_reset(static_cast<uint32_t>(fbWidth), static_cast<uint32_t>(fbHeight), m_reset,
               init.resolution.formatColor);

    bgfx_set_debug(m_debug);

    bgfx_set_view_clear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);

    m_initialized = res;
    return res;
}

void RenderBackendBGFX::resize(uint32_t width, uint32_t height)
{
    if (!m_initialized)
        return;
    bgfx_reset(width, height, m_reset, BGFX_TEXTURE_FORMAT_COUNT);
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
    bgfx_shutdown();
    m_initialized = false;
}

bool RenderBackendBGFX::requestBackbufferScreenshot(GLFWwindow* window, const std::string& path)
{
    SOFA_UNUSED(window);
    // Screenshot of the default backbuffer; serviced by the bgfx callback on the
    // next frame.
    bgfx_frame_buffer_handle_t handle = BGFX_INVALID_HANDLE;
    bgfx_request_screen_shot(handle, path.c_str());
    return true;
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
    sofa::core::ObjectFactory::ClassEntry::SPtr classVisualModel;
    sofa::core::ObjectFactory::AddAlias("VisualModel", "BGFXModel", true, &classVisualModel);
    sofa::core::ObjectFactory::AddAlias("OglModel", "BGFXModel", true, &classVisualModel);
}

void RenderBackendBGFX::applyReset()
{
    if (!m_initialized || !m_window)
        return;
    int w, h;
    glfwGetWindowSize(m_window, &w, &h);
    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(m_window, &xscale, &yscale);
    bgfx_reset(static_cast<uint32_t>(w * xscale), static_cast<uint32_t>(h * yscale),
               m_reset, BGFX_TEXTURE_FORMAT_COUNT);
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
