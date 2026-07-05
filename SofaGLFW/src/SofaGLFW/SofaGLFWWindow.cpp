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
#include <SofaGLFW/SofaGLFWWindow.h>
#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/gui/common/PickHandler.h>

#include <SofaGLFW/render/ISceneRenderer.h>
#include <SofaGLFW/render/RenderBackendFactory.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>

using namespace sofa;

namespace sofaglfw
{
SofaGLFWWindow::SofaGLFWWindow(GLFWwindow* glfwWindow, component::visual::BaseCamera::SPtr camera,
                               render::RenderAPI backend)
        : m_glfwWindow(glfwWindow)
        , m_currentCamera(camera)
        , m_sceneRenderer(render::RenderBackendFactory::getInstance().createSceneRenderer(backend))
{
}

SofaGLFWWindow::~SofaGLFWWindow() = default;

void SofaGLFWWindow::close()
{
    glfwDestroyWindow(m_glfwWindow);

    if (m_sceneRenderer)
        m_sceneRenderer->releaseResources();
}

void SofaGLFWWindow::draw(simulation::NodeSPtr groot, core::visual::VisualParams* vparams)
{
    if (!m_currentCamera)
    {
        msg_error("SofaGLFWWindow") << "No camera defined.";
        return;
    }
    if (!m_sceneRenderer)
    {
        msg_error("SofaGLFWWindow") << "No scene renderer available.";
        return;
    }

    const render::ViewportRect viewport{
        vparams->viewport()[0], vparams->viewport()[1],
        vparams->viewport()[2], vparams->viewport()[3]};

    m_sceneRenderer->drawScene(groot.get(), vparams, m_currentCamera.get(),
                               m_glfwWindow, viewport, m_backgroundColor);
}

void SofaGLFWWindow::setBackgroundColor(const sofa::type::RGBAColor& newColor)
{
    m_backgroundColor = newColor;
    if (m_sceneRenderer)
        m_sceneRenderer->clearBackgroundImage();
}

void SofaGLFWWindow::setBackgroundImage(const std::string& filename)
{
    if (m_sceneRenderer)
        m_sceneRenderer->setBackgroundImage(filename);
}

void SofaGLFWWindow::alignCamera(sofaglfw::SofaGLFWBaseGUI* baseGUI, const CameraAlignement& align)
{
    if (baseGUI)
    {
        sofa::core::sptr<sofa::simulation::Node> groot = baseGUI->getRootNode();
        if (groot)
        {
            sofa::component::visual::BaseCamera::SPtr camera;
            groot->get(camera);

            if (camera)
            {
                sofa::type::Quat<float> orientation;

                switch (align)
                {
                case CameraAlignement::TOP:
                    orientation = sofa::type::Quat(-0.707f, 0.f, 0.f, 0.707f);
                    break;
                case CameraAlignement::BOTTOM:
                    orientation = sofa::type::Quat(0.707f, 0.f, 0.f, 0.707f);
                    break;
                case CameraAlignement::FRONT:
                    orientation = sofa::type::Quat(0.f, 0.f, 0.f, 1.f);
                    break;
                case CameraAlignement::BACK:
                    orientation = sofa::type::Quat(0.f, 1.f, 0.f, 0.f);
                    break;
                case CameraAlignement::LEFT:
                    orientation = sofa::type::Quat(0.f, 0.707f, 0.f, 0.707f);
                    break;
                case CameraAlignement::RIGHT:
                    orientation = sofa::type::Quat(0.f, -0.707f, 0.f, 0.707f);
                    break;
                }

                auto bbCenter = (groot->f_bbox.getValue().maxBBox() + groot->f_bbox.getValue().minBBox()) * 0.5f;
                const auto cameraPosition = camera->getPositionFromOrientation(sofa::type::Vec3(0., 0., 0.), -camera->getDistance(), orientation);
                camera->setView(cameraPosition + bbCenter, orientation);
                camera->d_lookAt.setValue(bbCenter);
                camera->setCameraType(sofa::core::visual::VisualParams::ORTHOGRAPHIC_TYPE);
            }
        }
    }
}

void SofaGLFWWindow::setCamera(component::visual::BaseCamera::SPtr newCamera)
{
    m_currentCamera = newCamera;
}

void SofaGLFWWindow::centerCamera(simulation::NodeSPtr node, core::visual::VisualParams* vparams) const
{
    if (m_currentCamera)
    {
        int width, height;
        glfwGetFramebufferSize(m_glfwWindow, &width, &height);
        if (node->f_bbox.getValue().isValid())
        {
            vparams->sceneBBox() = node->f_bbox.getValue();
            m_currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
        }

        // Update the visual params
        vparams->viewport() = { 0, 0, width, height };
        vparams->zNear() = m_currentCamera->getZNear();
        vparams->zFar() = m_currentCamera->getZFar();

        m_currentCamera->fitBoundingBox(node->f_bbox.getValue().minBBox(), node->f_bbox.getValue().maxBBox());
    }
}

void SofaGLFWWindow::mouseMoveEvent(int xpos, int ypos, SofaGLFWBaseGUI* gui)
{
    m_currentXPos = xpos;
    m_currentYPos = ypos;
    switch (m_currentAction)
    {
        case GLFW_PRESS:
        {
            core::objectmodel::MouseEvent* mEvent = nullptr;
            if (m_currentButton == GLFW_MOUSE_BUTTON_LEFT)
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::LeftPressed, xpos, ypos);
            else if (m_currentButton == GLFW_MOUSE_BUTTON_RIGHT)
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::RightPressed, xpos, ypos);
            else if (m_currentButton == GLFW_MOUSE_BUTTON_MIDDLE)
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::MiddlePressed, xpos, ypos);
            else
            {
                // A fallback event to rule them all...
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::AnyExtraButtonPressed, xpos, ypos);
            }
            m_currentCamera->manageEvent(mEvent);

            auto rootNode = gui->getRootNode();

            rootNode->propagateEvent(core::execparams::defaultInstance(), mEvent);
            delete mEvent;

            break;
        }
        case GLFW_RELEASE:
        {
            core::objectmodel::MouseEvent* mEvent = nullptr;
            if (m_currentButton == GLFW_MOUSE_BUTTON_LEFT)
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::LeftReleased, xpos, ypos);
            else if (m_currentButton == GLFW_MOUSE_BUTTON_RIGHT)
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::RightReleased, xpos, ypos);
            else if (m_currentButton == GLFW_MOUSE_BUTTON_MIDDLE)
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::MiddleReleased, xpos, ypos);
            else
            {
                // A fallback event to rules them all...
                mEvent = new core::objectmodel::MouseEvent(core::objectmodel::MouseEvent::AnyExtraButtonReleased, xpos, ypos);
            }
            m_currentCamera->manageEvent(mEvent);

            auto rootNode = gui->getRootNode();

            rootNode->propagateEvent(core::execparams::defaultInstance(), mEvent);
            delete mEvent;

            break;
        }
        default:
        {
            core::objectmodel::MouseEvent me(core::objectmodel::MouseEvent::Move, xpos, ypos);
            m_currentCamera->manageEvent(&me);
            break;
        }
    }

    m_currentButton = -1;
    m_currentAction = -1;
    m_currentMods = -1;
}
void SofaGLFWWindow::mouseButtonEvent(int button, int action, int mods)
{
    // Only change state on button press; release resets state to neutral
        m_currentButton = button;
        m_currentAction = action;
        m_currentMods = mods;
}

bool SofaGLFWWindow::mouseEvent(GLFWwindow* window, int width, int height,int button, int action, int mods, double xpos, double ypos) const
{
    SOFA_UNUSED(mods);

    if (!m_currentCamera)
        return true;

    SofaGLFWBaseGUI *gui = static_cast<SofaGLFWBaseGUI *>(glfwGetWindowUserPointer(window));

    MousePosition mousepos;
    mousepos.screenWidth = width;
    mousepos.screenHeight = height;
    mousepos.x = static_cast<int>(xpos);
    mousepos.y = static_cast<int>(ypos);
    auto rootNode = gui->getRootNode();

    if (GLFW_MOD_SHIFT)
    {
        gui->getPickHandler()->activateRay(width, height, rootNode.get());
        gui->getPickHandler()->updateMouse2D(mousepos);

        if (action == GLFW_PRESS)
        {
            if (button == GLFW_MOUSE_BUTTON_LEFT)
            {
                gui->getPickHandler()->handleMouseEvent(PRESSED, LEFT);
            }
            else if (button == GLFW_MOUSE_BUTTON_RIGHT)
            {
                gui->getPickHandler()->handleMouseEvent(PRESSED, RIGHT);
            }
            else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
            {
                gui->getPickHandler()->handleMouseEvent(PRESSED, MIDDLE);
            }
        }
        else if (action == GLFW_RELEASE)
        {
            if (action == GLFW_RELEASE)
            {
                if (button == GLFW_MOUSE_BUTTON_LEFT)
                {
                    gui->getPickHandler()->handleMouseEvent(RELEASED, LEFT);
                    gui->getPickHandler()->deactivateRay();
                }
                else if (button == GLFW_MOUSE_BUTTON_RIGHT)
                {
                    gui->getPickHandler()->handleMouseEvent(RELEASED, RIGHT);
                }
                else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
                {
                    gui->getPickHandler()->handleMouseEvent(RELEASED, MIDDLE);
                }
            }
        }
        gui->moveRayPickInteractor(xpos, ypos);
    }
    else
    {
        gui->getPickHandler()->activateRay(width, height, rootNode.get());
    }
    return true;
}

void SofaGLFWWindow::scrollEvent(double xoffset, double yoffset)
{
    SOFA_UNUSED(xoffset);
    const double yFactor = 10.f;
    core::objectmodel::MouseEvent me(core::objectmodel::MouseEvent::Wheel, static_cast<int>(yoffset * yFactor));
    m_currentCamera->manageEvent(&me);
}

} // namespace sofaglfw
