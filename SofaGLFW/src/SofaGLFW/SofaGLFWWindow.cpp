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

using namespace sofa;

#include <cstring>
#include <bgfx/c99/bgfx.h>
#include <bgfx/bgfx.h>
#include <bx/math.h>
#include <BGFXPlugin/DrawToolBGFX.h>
#include <BGFXPlugin/BGFXShaderUtils.h>

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

    m_backgrounds.clear();

    if (m_bgProgram.idx != UINT16_MAX)
    {
        bgfx_destroy_program(m_bgProgram);
        m_bgProgram.idx = UINT16_MAX;
    }
    if (m_bgTexUniform.idx != UINT16_MAX)
    {
        bgfx_destroy_uniform(m_bgTexUniform);
        m_bgTexUniform.idx = UINT16_MAX;
    }
}

void SofaGLFWWindow::draw(simulation::NodeSPtr groot, core::visual::VisualParams* vparams)
{
    constexpr uint16_t kViewBackground = 0;
    constexpr uint16_t kViewScene = 1;

    // ImGui viewport rect is in logical pixels; bgfx needs framebuffer pixels
    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(m_glfwWindow, &xscale, &yscale);

    const uint16_t vpX = static_cast<uint16_t>(vparams->viewport()[0] * xscale);
    const uint16_t vpY = static_cast<uint16_t>(vparams->viewport()[1] * yscale);
    const uint16_t width = static_cast<uint16_t>(vparams->viewport()[2] * xscale);
    const uint16_t height = static_cast<uint16_t>(vparams->viewport()[3] * yscale);

    const uint32_t clearColor =
        (uint32_t(m_backgroundColor[0] * 255.0f) << 24) |
        (uint32_t(m_backgroundColor[1] * 255.0f) << 16) |
        (uint32_t(m_backgroundColor[2] * 255.0f) <<  8) |
        (uint32_t(m_backgroundColor[3] * 255.0f));

    // Ensure views render in order: background(0), scene(1)
    const uint16_t viewOrder[] = { kViewBackground, kViewScene };
    bgfx_set_view_order(0, 2, viewOrder);

    // Get full backbuffer size
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(m_glfwWindow, &fbWidth, &fbHeight);

    // View 0: clear entire backbuffer + optional background texture in viewport area
    bgfx_set_view_rect(kViewBackground, 0, 0, static_cast<uint16_t>(fbWidth), static_cast<uint16_t>(fbHeight));
    bgfx_set_view_clear(kViewBackground, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, clearColor, 1.0f, 0);

    if (!drawBackgroundImage(vpX, vpY, width, height, fbWidth, fbHeight))
        bgfx_touch(kViewBackground);

    // draw the scene
    if (!m_currentCamera)
    {
        msg_error("SofaGLFWGUI") << "No camera defined.";
        return;
    }

    if (groot->f_bbox.getValue().isValid())
    {
        vparams->sceneBBox() = groot->f_bbox.getValue();
        m_currentCamera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
    }
    m_currentCamera->computeZ();
    m_currentCamera->d_widthViewport.setValue(vparams->viewport()[2]);
    m_currentCamera->d_heightViewport.setValue(vparams->viewport()[3]);

    // View 1: 3D scene with camera (viewport subset)
    bgfx_set_view_rect(kViewScene, vpX, vpY, width, height);
    bgfx_set_view_clear(kViewScene, BGFX_CLEAR_DEPTH, 0, 1.0f, 0);
    float projY5 = 1.0f;
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

        // OpenGL projection maps depth to [-1,1]; remap to [0,1] for non-GL backends (Metal/D3D)
        if (!bgfx::getCaps()->homogeneousDepth)
        {
            proj[2]  = proj[2]  * 0.5f + proj[3]  * 0.5f;
            proj[6]  = proj[6]  * 0.5f + proj[7]  * 0.5f;
            proj[10] = proj[10] * 0.5f + proj[11] * 0.5f;
            proj[14] = proj[14] * 0.5f + proj[15] * 0.5f;
        }

        bgfx_set_view_transform(kViewScene, view, proj);

        // Update the visual params
        vparams->zNear() = m_currentCamera->getZNear();
        vparams->zFar() = m_currentCamera->getZFar();
        vparams->setModelViewMatrix(viewd);
        vparams->setProjectionMatrix(projd);

        projY5 = proj[5];
    }

    bgfx_touch(kViewScene);

    auto* drawTool = dynamic_cast<bgfxplugin::DrawToolBGFX*>(vparams->drawTool());
    if (drawTool)
    {
        drawTool->setViewId(kViewScene);
        drawTool->setCameraPosition(m_currentCamera->getPosition());
        bool isOrtho = (m_currentCamera->getCameraType() == sofa::core::visual::VisualParams::ORTHOGRAPHIC_TYPE);
        drawTool->setScreenParams(static_cast<float>(height), projY5, isOrtho);
    }

    sofa::simulation::node::draw(vparams, groot.get());

}

void SofaGLFWWindow::setBackgroundColor(const RGBAColor& newColor)
{
    m_backgroundColor = newColor;
    m_currentBackgroundFilename = "";
}


void SofaGLFWWindow::setBackgroundImage(const std::string& filename)
{
    if(!m_backgrounds.contains(filename))
    {
        std::string tempFilename = filename;
        if( sofa::helper::system::DataRepository.findFile(tempFilename) )
        {
            const auto backgroundImageFilename = sofa::helper::system::DataRepository.getFile(tempFilename);

            std::string extension = sofa::helper::system::SetDirectory::GetExtension(filename.c_str());
            std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

            auto* backgroundImage = helper::io::Image::FactoryImage::getInstance()->createObject(extension, backgroundImageFilename);
            if( !backgroundImage )
            {
                msg_warning("GUI") << "Could not load the file " << filename;
                return;
            }

            auto tex = std::make_unique<bgfxplugin::Texture>(backgroundImage, true, true, false);
            tex->init();
            if (tex->isValid())
            {
                m_backgrounds[filename] = Background{ std::move(tex) };
            }
        }
    }
    m_currentBackgroundFilename = filename;
}


bool SofaGLFWWindow::drawBackgroundImage(uint16_t vpX, uint16_t vpY, uint16_t vpW, uint16_t vpH, int fbW, int fbH)
{
    if (m_currentBackgroundFilename.empty())
        return false;

    if (!m_backgrounds.contains(m_currentBackgroundFilename))
        return false;

    const auto& background = m_backgrounds[m_currentBackgroundFilename];
    if (!background.texture || !background.texture->isValid())
        return false;

    if (m_bgProgram.idx == UINT16_MAX)
    {
        m_bgProgram = bgfxplugin::loadProgram("vs_imgui", "fs_imgui",
            std::string(SOFAIMGUI_RESOURCES_DIR) + "/shaders");
        m_bgTexUniform = bgfx_create_uniform("s_texColor", BGFX_UNIFORM_TYPE_SAMPLER, 1);
    }

    if (m_bgProgram.idx == UINT16_MAX)
        return false;

    struct PosTexColorVertex
    {
        float x, y;
        float u, v;
        uint32_t col;
    };

    const float x0 = static_cast<float>(vpX);
    const float y0 = static_cast<float>(vpY);
    const float x1 = static_cast<float>(vpX + vpW);
    const float y1 = static_cast<float>(vpY + vpH);

    float uMax = 1.0f;
    float vMax = 1.0f;
    if (auto* img = background.texture->getImage())
    {
        const float texW = static_cast<float>(img->getWidth());
        const float texH = static_cast<float>(img->getHeight());
        if (texW > 0.0f && texH > 0.0f)
        {
            uMax = static_cast<float>(vpW) / texW;
            vMax = static_cast<float>(vpH) / texH;
        }
    }

    constexpr uint32_t white = 0xFFFFFFFF;
    const PosTexColorVertex vertices[] = {
        { x0, y0, 0.0f, vMax, white },
        { x1, y0, uMax, vMax, white },
        { x1, y1, uMax, 0.0f, white },
        { x0, y1, 0.0f, 0.0f, white },
    };
    const uint16_t indices[] = { 0, 1, 2, 0, 2, 3 };

    bgfx_vertex_layout_t layout;
    bgfx_vertex_layout_begin(&layout, bgfx_get_renderer_type());
    bgfx_vertex_layout_add(&layout, BGFX_ATTRIB_POSITION, 2, BGFX_ATTRIB_TYPE_FLOAT, false, false);
    bgfx_vertex_layout_add(&layout, BGFX_ATTRIB_TEXCOORD0, 2, BGFX_ATTRIB_TYPE_FLOAT, false, false);
    bgfx_vertex_layout_add(&layout, BGFX_ATTRIB_COLOR0, 4, BGFX_ATTRIB_TYPE_UINT8, true, false);
    bgfx_vertex_layout_end(&layout);

    bgfx_transient_vertex_buffer_t tvb;
    bgfx_transient_index_buffer_t tib;
    bgfx_alloc_transient_vertex_buffer(&tvb, 4, &layout);
    bgfx_alloc_transient_index_buffer(&tib, 6, false);

    memcpy(tvb.data, vertices, sizeof(vertices));
    memcpy(tib.data, indices, sizeof(indices));

    float view[16];
    bx::mtxIdentity(view);
    float proj[16];
    const bool homogeneousDepth = bgfx::getCaps()->homogeneousDepth;
    bx::mtxOrtho(proj, 0.0f, static_cast<float>(fbW), static_cast<float>(fbH), 0.0f, 0.0f, 1.0f, 0.0f, homogeneousDepth);
    bgfx_set_view_transform(0, view, proj);

    bgfx_set_transient_vertex_buffer(0, &tvb, 0, 4);
    bgfx_set_transient_index_buffer(&tib, 0, 6);

    background.texture->bind(0, m_bgTexUniform);

    bgfx_set_state(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A, 0);
    bgfx_submit(0, m_bgProgram, 0, BGFX_DISCARD_ALL);
    return true;
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
