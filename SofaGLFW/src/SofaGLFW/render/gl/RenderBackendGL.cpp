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
#include <SofaGLFW/render/gl/RenderBackendGL.h>
#include <SofaGLFW/render/gl/SceneRendererGL.h>
#include <SofaGLFW/render/RenderBackendFactory.h>

#include <sofa/gl/gl.h> // pulls in <GL/glew.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/gl/DrawToolGL.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/io/STBImage.h>

namespace sofaglfw::render
{

bool RenderBackendGL::initEngine(GLFWwindow* window, uint32_t width, uint32_t height)
{
    SOFA_UNUSED(width);
    SOFA_UNUSED(height);

    m_window = window;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(m_vsync ? 1 : 0);

    if (!m_glewInitialized)
    {
        if (glewInit() != GLEW_OK)
        {
            msg_error("RenderBackendGL") << "Failed to initialize GLEW.";
            return false;
        }
        m_glewInitialized = true;
    }

    // Legacy fixed-function GL state and 'light 0' setup (ported from master).
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0);
    glEnable(GL_NORMALIZE);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    float lightAmbient[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
    float lightDiffuse[4] = { 0.9f, 0.9f, 0.9f, 1.0f };
    float lightSpecular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    float lightPosition[4] = { -0.7f, 0.3f, 0.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    float materialSpecular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
    glMateriali(GL_FRONT, GL_SHININESS, 128);

    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHT0);

    return true;
}

void RenderBackendGL::resize(uint32_t width, uint32_t height)
{
    SOFA_UNUSED(width);
    SOFA_UNUSED(height);
    // The viewport is set per-frame from VisualParams; nothing to reset here.
}

uint32_t RenderBackendGL::present(GLFWwindow* window)
{
    if (window)
        glfwSwapBuffers(window);
    return ++m_frameCounter;
}

void RenderBackendGL::terminate()
{
    // Nothing GL-global to release; contexts are owned by GLFW windows.
}

bool RenderBackendGL::requestBackbufferScreenshot(GLFWwindow* window, const std::string& path, int compressionLevel)
{
    // Synchronous, like master's NullGUIEngine: read the current viewport.
    SOFA_UNUSED(window);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    sofa::helper::io::STBImage image;
    image.init(viewport[2], viewport[3], 1, 1, sofa::helper::io::Image::DataType::UINT32,
               sofa::helper::io::Image::ChannelFormat::RGBA);
    glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, image.getPixels());
    return image.save(path, compressionLevel < 0 ? 90 : compressionLevel);
}

sofa::type::Vec2i RenderBackendGL::backbufferViewportSize(GLFWwindow* window) const
{
    // glViewport takes framebuffer pixels.
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    return { width, height };
}

sofa::type::Vec2i RenderBackendGL::readBackbufferPixels(GLFWwindow* window, std::vector<uint8_t>& pixels)
{
    SOFA_UNUSED(window);
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    pixels.resize(static_cast<std::size_t>(viewport[2]) * viewport[3] * 4);
    glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    return { viewport[2], viewport[3] };
}

std::unique_ptr<sofa::helper::visual::DrawTool> RenderBackendGL::makeDrawTool()
{
    return std::make_unique<sofa::gl::DrawToolGL>();
}

void RenderBackendGL::configureVisualParams()
{
    sofa::core::visual::VisualParams::defaultInstance()->setSupported(sofa::core::visual::API_OpenGL);
}

void RenderBackendGL::registerVisualModelAliases()
{
    // OpenGL uses the default OglModel; no alias override needed.
}

void RenderBackendGL::setVsync(bool enabled)
{
    m_vsync = enabled;
    if (m_window)
    {
        glfwMakeContextCurrent(m_window);
        glfwSwapInterval(enabled ? 1 : 0);
    }
}

void RenderBackendGL::setMsaa(int level)
{
    // MSAA is selected at window-creation time via GLFW_SAMPLES; store for query.
    m_msaa = level;
}

// Self-register this backend at static-init time.
static const RenderBackendRegistrar s_registrar(
    RenderAPI::OpenGL,
    [] { return std::unique_ptr<IRenderBackend>(new RenderBackendGL()); },
    [] { return std::unique_ptr<ISceneRenderer>(new SceneRendererGL()); });

} // namespace sofaglfw::render
