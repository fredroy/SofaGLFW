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
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/gui/common/PickHandler.h>


#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/helper/io/Image.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>

#include <ranges>

using namespace sofa;

#include <bgfx/c99/bgfx.h>
#include <bx/math.h>

namespace sofaglfw
{
SofaGLFWWindow::SofaGLFWWindow(GLFWwindow* glfwWindow, component::visual::BaseCamera::SPtr camera)
        : m_glfwWindow(glfwWindow)
        , m_currentCamera(camera)
{

}

void SofaGLFWWindow::close()
{
    glfwDestroyWindow(m_glfwWindow);
    
    if(m_currentBackgroundTexture)
    {
        delete m_currentBackgroundTexture;
        m_currentBackgroundTexture = nullptr;
    }
    
    for(auto& [_, background] : m_backgrounds)
    {
        delete background.texture;
    }
    
    m_backgrounds.clear();
}

void SofaGLFWWindow::draw(simulation::NodeSPtr groot, core::visual::VisualParams* vparams)
{
    std::size_t width = vparams->viewport()[2];
    std::size_t height = vparams->viewport()[3];

    // draw the scene
    if (!m_currentCamera)
    {
        msg_error("SofaGLFWGUI") << "No camera defined.";
        return;
    }

    // Set view and projection matrix for view 0.
    {
        double viewd[16]{};
        float view[16]{};

        double projd[16]{};
        float proj[16]{};

        m_currentCamera->getOpenGLModelViewMatrix(viewd);
        m_currentCamera->getOpenGLProjectionMatrix(projd);

        for (unsigned int i = 0; i < 16; i++)
        {
            view[i] = static_cast<float>(viewd[i]);
            proj[i] = static_cast<float>(projd[i]);
        }

        bgfx_set_view_transform(0, view, proj);

        // Set view 0 default viewport.
        bgfx_set_view_rect(0, 0, 0, uint16_t(width), uint16_t(height));

        // Update the visual params
        vparams->zNear() = m_currentCamera->getZNear();
        vparams->zFar() = m_currentCamera->getZFar();
        vparams->setProjectionMatrix(viewd);
        vparams->setModelViewMatrix(projd);
    }

    // This dummy draw call is here to make sure that view 0 is cleared
    // if no other draw calls are submitted to view 0.
    bgfx_touch(0);


    // This dummy draw call is here to make sure that view 0 is cleared
    // if no other draw calls are submitted to view 0.
    bgfx_encoder_t* encoder = bgfx_encoder_begin(true);
    bgfx_encoder_touch(encoder, 0);
    bgfx_encoder_end(encoder);

    sofa::simulation::node::draw(vparams, groot.get());

    // Advance to next frame. Rendering thread will be kicked to
    // process submitted rendering primitives.

    try
    {
        bgfx_frame(false);
    }
    catch (std::exception e)
    {
        std::cout << e.what() << std::endl;
    }

}

void SofaGLFWWindow::setBackgroundColor(const RGBAColor& newColor)
{
    m_backgroundColor = newColor;
    m_currentBackgroundFilename = "";
}


void SofaGLFWWindow::setBackgroundImage(const std::string& filename)
{
    // when setting a background image, we check if it was not loaded and cached first
    if(!m_backgrounds.contains(filename))
    {
        std::string tempFilename = filename;
        if( sofa::helper::system::DataRepository.findFile(tempFilename) )
        {
            const auto backgroundImageFilename = sofa::helper::system::DataRepository.getFile(tempFilename);
            
            std::string extension = sofa::helper::system::SetDirectory::GetExtension(filename.c_str());
            std::ranges::transform(extension, extension.begin(), ::tolower );
            
            auto* backgroundImage = helper::io::Image::FactoryImage::getInstance()->createObject(extension, backgroundImageFilename);
            if( !backgroundImage )
            {
                msg_warning("GUI") << "Could not load the file " << filename;
                return;
            }
            else
            {
                //auto* texture = new gl::Texture(backgroundImage);
                //if(texture)
                //{
                //    texture->init();
                //    m_backgrounds.emplace(filename, Background{backgroundImage, texture});
                //}
            }
        }
    }
    m_currentBackgroundFilename = filename;
}


void SofaGLFWWindow::drawBackgroundImage()
{
    if(!m_backgrounds.contains(m_currentBackgroundFilename))
        return;
        
    const auto& background = m_backgrounds[m_currentBackgroundFilename];

    if(!background.image)
        return;
    
    const int imageWidth = background.image->getWidth();
    const int imageHeight = background.image->getHeight();
    
    int screenWidth = 0;
    int screenHeight = 0;
    
    glfwGetFramebufferSize(m_glfwWindow, &screenWidth, &screenHeight);

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
