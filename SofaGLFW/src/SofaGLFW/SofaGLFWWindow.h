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
#include <sofa/simulation/Node.h>

#include <sofa/type/fwd.h>
#include <sofa/type/RGBAColor.h>
#include <sofa/component/visual/BaseCamera.h>
#include <SofaGLFW/render/RenderAPI.h>

#include <functional>
#include <memory>
#include <string>

struct GLFWwindow;

namespace sofaglfw::render
{
    class ISceneRenderer;
}

namespace sofaglfw
{

class SofaGLFWBaseGUI;

class SOFAGLFW_API SofaGLFWWindow
{
public:
    /// @param backend the resolved backend used to build the matching scene renderer
    SofaGLFWWindow(GLFWwindow* glfwWindow, sofa::component::visual::BaseCamera::SPtr camera,
                   render::RenderAPI backend);
    virtual ~SofaGLFWWindow();

    void draw(sofa::simulation::NodeSPtr groot, sofa::core::visual::VisualParams* vparams);
    /// Draws the scene with its camera into a recording, for replay(): from a simulation
    /// thread that owns the scene. @p afterDraw draws more into it. @return false if the
    /// renderer cannot record, or has not drawn a frame yet.
    bool record(sofa::simulation::NodeSPtr groot, sofa::core::visual::VisualParams* vparams,
                const std::function<void()>& afterDraw);
    bool canReplay() const;
    /// Draws the scene from the draw calls of the last record(), with @p camera (the
    /// GUI's own), without the scene graph. @return false if nothing was drawn.
    bool replay(sofa::component::visual::BaseCamera* camera, const render::ViewportRect& viewport);
    /// Release the scene renderer's GPU resources (the window stays open).
    void releaseResources();
    void close();

    void mouseMoveEvent(int xpos, int ypos,SofaGLFWBaseGUI* gui);
    void mouseButtonEvent(int button, int action, int mods);
    void scrollEvent(double xoffset, double yoffset);
    void setBackgroundColor(const sofa::type::RGBAColor& newColor);
    void setBackgroundImage(const std::string& filename);

    void setCamera(sofa::component::visual::BaseCamera::SPtr newCamera);
    /// The camera the mouse moves, if not the scene's: a copy owned by the GUI, which a
    /// simulation thread does not touch (see SofaGLFWBaseGUI). Null for the scene's.
    void setInteractionCamera(sofa::component::visual::BaseCamera::SPtr camera);
    void centerCamera(sofa::simulation::NodeSPtr node, sofa::core::visual::VisualParams* vparams) const;
    bool mouseEvent(GLFWwindow* window,int width,int height ,int button, int action, int mods, double xpos, double ypos) const;

    enum class CameraAlignement { TOP, BOTTOM, LEFT, RIGHT, FRONT, BACK };

    static void alignCamera(sofaglfw::SofaGLFWBaseGUI* baseGUI, const CameraAlignement& align);

private:
    GLFWwindow* m_glfwWindow{nullptr};
    sofa::component::visual::BaseCamera::SPtr m_currentCamera;
    sofa::component::visual::BaseCamera::SPtr m_interactionCamera;
    sofa::component::visual::BaseCamera* interactionCamera() const
    {
        return m_interactionCamera ? m_interactionCamera.get() : m_currentCamera.get();
    }
    int m_currentButton{ -1 };
    int m_currentAction{ -1 };
    int m_currentMods{ -1 };
    int m_currentXPos{ -1 };
    int m_currentYPos{ -1 };
    sofa::type::RGBAColor m_backgroundColor{ sofa::type::RGBAColor::black() };

    std::unique_ptr<render::ISceneRenderer> m_sceneRenderer;
};

} // namespace sofaglfw
