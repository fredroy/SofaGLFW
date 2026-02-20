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
#include <sofa/gl/gl.h>
#include <sofa/gl/Texture.h>
#include <sofa/gl/DrawToolGL.h>

#include <ranges>

using namespace sofa;
namespace sofaglfw
{
SofaGLFWWindow::SofaGLFWWindow(GLFWwindow* glfwWindow, component::visual::BaseCamera::SPtr camera)
        : m_glfwWindow(glfwWindow)
        , m_currentCamera(camera)
{

}

void SofaGLFWWindow::close()
{
    if (m_bgGLInitialized)
    {
        glDeleteProgram(m_bgShaderProgram);
        glDeleteVertexArrays(1, &m_bgVAO);
        glDeleteBuffers(1, &m_bgVBO);
        m_bgGLInitialized = false;
    }

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
    glClearColor(m_backgroundColor.r(), m_backgroundColor.g(), m_backgroundColor.b(), m_backgroundColor.a());
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_currentBackgroundFilename.empty())
        drawBackgroundImage();

    glEnable(GL_DEPTH_TEST);

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

    // matrices
    double lastModelviewMatrix [16];
    double lastProjectionMatrix [16];

    m_currentCamera->getOpenGLProjectionMatrix(lastProjectionMatrix);
    m_currentCamera->getOpenGLModelViewMatrix(lastModelviewMatrix);

    glViewport(0, 0, vparams->viewport()[2], vparams->viewport()[3]);

    // Pass matrices to DrawToolGL for shader-based rendering
    if (auto* drawToolGL = dynamic_cast<sofa::gl::DrawToolGL*>(vparams->drawTool()))
    {
        drawToolGL->setProjectionMatrix(lastProjectionMatrix);
        drawToolGL->setModelViewMatrix(lastModelviewMatrix);
        // Default headlight: directional light in view space pointing into the scene
        drawToolGL->setLightPosition(0.0f, 0.5f, 1.0f, 0.0f);
    }

#ifndef __APPLE__
    // Compatibility profile: also set legacy GL matrices for other renderers
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(lastProjectionMatrix);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(lastModelviewMatrix);
#endif

    // Update the visual params
    vparams->zNear() = m_currentCamera->getZNear();
    vparams->zFar() = m_currentCamera->getZFar();
    vparams->setProjectionMatrix(lastProjectionMatrix);
    vparams->setModelViewMatrix(lastModelviewMatrix);

    simulation::node::draw(vparams, groot.get());
    
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
                auto* texture = new gl::Texture(backgroundImage);
                if(texture)
                {
                    texture->init();
                    m_backgrounds.emplace(filename, Background{backgroundImage, texture});
                }
            }
        }
    }
    m_currentBackgroundFilename = filename;
}


void SofaGLFWWindow::initBackgroundGL()
{
    if (m_bgGLInitialized)
        return;

    static const char* vertSrc = R"(
        #version 410
        layout(location=0) in vec2 a_position;
        layout(location=1) in vec2 a_texcoord;
        out vec2 v_texcoord;
        void main() {
            gl_Position = vec4(a_position, 0.0, 1.0);
            v_texcoord = a_texcoord;
        }
    )";

    static const char* fragSrc = R"(
        #version 410
        in vec2 v_texcoord;
        out vec4 fragColor;
        uniform sampler2D u_texture;
        uniform vec2 u_texScale;
        void main() {
            fragColor = texture(u_texture, v_texcoord * u_texScale);
        }
    )";

    auto compileShader = [](GLenum type, const char* src) -> GLuint {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = 0;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(s, 512, nullptr, log);
            msg_error("SofaGLFWWindow") << "Background shader compile error: " << log;
        }
        return s;
    };

    GLuint vs = compileShader(GL_VERTEX_SHADER, vertSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragSrc);

    m_bgShaderProgram = glCreateProgram();
    glAttachShader(m_bgShaderProgram, vs);
    glAttachShader(m_bgShaderProgram, fs);
    glLinkProgram(m_bgShaderProgram);

    GLint ok = 0;
    glGetProgramiv(m_bgShaderProgram, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(m_bgShaderProgram, 512, nullptr, log);
        msg_error("SofaGLFWWindow") << "Background shader link error: " << log;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);

    m_bgLocTexture = glGetUniformLocation(m_bgShaderProgram, "u_texture");
    m_bgLocTexScale = glGetUniformLocation(m_bgShaderProgram, "u_texScale");

    // Fullscreen quad in NDC: two triangles
    static const float quadVerts[] = {
        // pos.x, pos.y,  tex.x, tex.y
        -1.f, -1.f,   0.f, 0.f,
         1.f, -1.f,   1.f, 0.f,
         1.f,  1.f,   1.f, 1.f,
        -1.f, -1.f,   0.f, 0.f,
         1.f,  1.f,   1.f, 1.f,
        -1.f,  1.f,   0.f, 1.f,
    };

    glGenVertexArrays(1, &m_bgVAO);
    glGenBuffers(1, &m_bgVBO);

    glBindVertexArray(m_bgVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_bgVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    m_bgGLInitialized = true;
}

void SofaGLFWWindow::drawBackgroundImage()
{
    if(!m_backgrounds.contains(m_currentBackgroundFilename))
        return;

    const auto& background = m_backgrounds[m_currentBackgroundFilename];

    if(!background.image)
        return;

    initBackgroundGL();
    if (!m_bgGLInitialized)
        return;

    const float imageWidth  = static_cast<float>(background.image->getWidth());
    const float imageHeight = static_cast<float>(background.image->getHeight());
    const float screenWidth  = static_cast<float>(m_currentCamera->d_widthViewport.getValue());
    const float screenHeight = static_cast<float>(m_currentCamera->d_heightViewport.getValue());

    glDisable(GL_DEPTH_TEST);

    glUseProgram(m_bgShaderProgram);

    glActiveTexture(GL_TEXTURE0);
    background.texture->bind();
    glUniform1i(m_bgLocTexture, 0);
    glUniform2f(m_bgLocTexScale, screenWidth / imageWidth, screenHeight / imageHeight);

    glBindVertexArray(m_bgVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    background.texture->unbind();
    glUseProgram(0);

    glEnable(GL_DEPTH_TEST);
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
