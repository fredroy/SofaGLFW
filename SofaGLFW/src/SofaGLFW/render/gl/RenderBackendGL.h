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

namespace sofaglfw::render
{

/// Legacy OpenGL implementation of IRenderBackend. This is the only place,
/// together with the other files under render/gl/, where OpenGL/GLEW headers
/// are included on the SofaGLFW side.
class RenderBackendGL : public IRenderBackend
{
public:
    RenderBackendGL() = default;
    ~RenderBackendGL() override = default;

    bool initEngine(GLFWwindow* window, uint32_t width, uint32_t height) override;
    void resize(uint32_t width, uint32_t height) override;
    uint32_t present(GLFWwindow* window) override;
    void terminate() override;
    bool requestBackbufferScreenshot(GLFWwindow* window, const std::string& path) override;

    std::unique_ptr<sofa::helper::visual::DrawTool> makeDrawTool() override;
    void configureVisualParams() override;
    void registerVisualModelAliases() override;

    bool supportsMultiViewport() const override { return true; }
    bool needsGlfwContext() const override { return true; }

    void setVsync(bool enabled) override;
    bool isVsync() const override { return m_vsync; }
    void setMsaa(int level) override;
    int  getMsaa() const override { return m_msaa; }

private:
    GLFWwindow* m_window{nullptr};
    bool m_glewInitialized{false};
    bool m_vsync{false};
    int m_msaa{0};
    uint32_t m_frameCounter{0};
};

} // namespace sofaglfw::render
