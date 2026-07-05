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

#include <sofa/helper/io/Image.h>
#include <sofa/gl/Texture.h>

#include <map>
#include <string>

namespace sofaglfw::render
{

/// Legacy OpenGL implementation of ISceneRenderer: immediate-mode fixed-function
/// scene draw with camera matrices loaded via glMultMatrix, and a tiled textured
/// background quad (ported from master's SofaGLFWWindow).
class SceneRendererGL : public ISceneRenderer
{
public:
    SceneRendererGL() = default;
    ~SceneRendererGL() override;

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
    void drawBackgroundImage(sofa::component::visual::BaseCamera* camera);

    struct Background
    {
        sofa::helper::io::Image* image{nullptr};
        sofa::gl::Texture* texture{nullptr};
    };

    std::map<std::string, Background> m_backgrounds;
    std::string m_currentBackgroundFilename{};
};

} // namespace sofaglfw::render
