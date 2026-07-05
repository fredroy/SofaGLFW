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

#include <SofaGLFW/config.h>
#include <SofaGLFW/render/RenderTypes.h>

#include <sofa/simulation/fwd.h>
#include <sofa/type/RGBAColor.h>

#include <string>

struct GLFWwindow;

namespace sofa::core::visual
{
    class VisualParams;
}
namespace sofa::component::visual
{
    class BaseCamera;
}

namespace sofaglfw::render
{

/// Backend-agnostic per-window scene renderer. Owns everything that used to
/// live in SofaGLFWWindow::draw()/drawBackgroundImage(): camera matrix setup,
/// clear, viewport, background image drawing, and the final node::draw call.
/// One concrete implementation per backend; each includes only its own API.
class SOFAGLFW_API ISceneRenderer
{
public:
    virtual ~ISceneRenderer() = default;

    /// Draw the whole scene for one window.
    /// @param groot    scene root
    /// @param vparams  visual params (matrices/viewport are written here)
    /// @param camera   current camera (never null when called)
    /// @param glfwWindow window, for content-scale queries
    /// @param viewport target rectangle in framebuffer pixels
    /// @param background clear color
    virtual void drawScene(sofa::simulation::Node* groot,
                           sofa::core::visual::VisualParams* vparams,
                           sofa::component::visual::BaseCamera* camera,
                           GLFWwindow* glfwWindow,
                           const ViewportRect& viewport,
                           const sofa::type::RGBAColor& background) = 0;

    /// Set (and lazily load/cache) the background image by filename.
    virtual void setBackgroundImage(const std::string& filename) = 0;

    /// Clear any background image (a solid color will be used instead).
    virtual void clearBackgroundImage() = 0;

    /// Release backend GPU resources held by this renderer.
    virtual void releaseResources() = 0;
};

} // namespace sofaglfw::render
