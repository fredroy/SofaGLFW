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

#include <SofaGLFW/render/ISceneRenderer.h>

#include <bgfx/c99/bgfx.h>
#include <BGFXPlugin/Texture.h>

#include <map>
#include <memory>
#include <string>

namespace sofaglfw::render
{

/// bgfx implementation of ISceneRenderer: two-view (background + scene) draw
/// with camera matrix setup, depth-range remap for non-GL backends, and a
/// textured background quad rendered from a transient buffer.
class SceneRendererBGFX : public ISceneRenderer
{
public:
    SceneRendererBGFX() = default;
    ~SceneRendererBGFX() override;

    void drawScene(sofa::simulation::Node* groot,
                   sofa::core::visual::VisualParams* vparams,
                   sofa::component::visual::BaseCamera* camera,
                   GLFWwindow* glfwWindow,
                   const ViewportRect& viewport,
                   const sofa::type::RGBAColor& background) override;

    void setBackgroundImage(const std::string& filename) override;
    void clearBackgroundImage() override;
    void releaseResources() override;

private:
    bool drawBackgroundImage(uint16_t vpX, uint16_t vpY, uint16_t vpW, uint16_t vpH,
                             int fbW, int fbH);

    struct Background
    {
        std::unique_ptr<bgfxplugin::Texture> texture;
    };

    std::map<std::string, Background> m_backgrounds;
    std::string m_currentBackgroundFilename{};
    bgfx_program_handle_t m_bgProgram{UINT16_MAX};
    bgfx_uniform_handle_t m_bgTexUniform{UINT16_MAX};
};

} // namespace sofaglfw::render
