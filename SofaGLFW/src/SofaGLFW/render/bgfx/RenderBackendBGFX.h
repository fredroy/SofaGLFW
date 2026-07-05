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

#include <SofaGLFW/render/IRenderBackend.h>

#include <bgfx/c99/bgfx.h>

namespace sofaglfw::render
{

/// bgfx implementation of IRenderBackend. This is the only place, together with
/// the other files under render/bgfx/, where <bgfx/*> is included on the
/// SofaGLFW side.
class RenderBackendBGFX : public IRenderBackend
{
public:
    RenderBackendBGFX();
    ~RenderBackendBGFX() override;

    bool initEngine(GLFWwindow* window, uint32_t width, uint32_t height) override;
    void resize(uint32_t width, uint32_t height) override;
    uint32_t present(GLFWwindow* window) override;
    void terminate() override;
    bool requestBackbufferScreenshot(GLFWwindow* window, const std::string& path) override;

    std::unique_ptr<sofa::helper::visual::DrawTool> makeDrawTool() override;
    void configureVisualParams() override;
    void registerVisualModelAliases() override;

    bool supportsMultiViewport() const override { return false; }
    bool needsGlfwContext() const override { return false; }

    void setVsync(bool enabled) override;
    bool isVsync() const override { return (m_reset & BGFX_RESET_VSYNC) != 0; }
    void setMsaa(int level) override;
    int  getMsaa() const override;

private:
    void applyReset();

    GLFWwindow* m_window{nullptr};

#if WIN32
    bgfx_renderer_type m_type = bgfx_renderer_type::BGFX_RENDERER_TYPE_DIRECT3D11;
#elif __APPLE__
    bgfx_renderer_type m_type = bgfx_renderer_type::BGFX_RENDERER_TYPE_METAL;
#else
    bgfx_renderer_type m_type = bgfx_renderer_type::BGFX_RENDERER_TYPE_COUNT; // auto
#endif

    uint32_t m_debug = BGFX_DEBUG_TEXT;
    uint32_t m_reset = BGFX_RESET_VSYNC | BGFX_RESET_HIDPI;
    bool m_initialized{false};
};

} // namespace sofaglfw::render
