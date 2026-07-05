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
#include <SofaImGui/render/gl/ImGuiPlatformGL.h>
#include <SofaImGui/render/ImGuiPlatformFactory.h>

#include <backends/imgui_impl_glfw.h>
// GL2 vs GL3 is a build-time choice local to this OpenGL backend TU. It never
// leaks into shared GUI logic.
#if SOFAIMGUI_FORCE_OPENGL2 == 1
#include <backends/imgui_impl_opengl2.h>
#else
#include <backends/imgui_impl_opengl3.h>
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <sofa/helper/io/STBImage.h>

#include <cstring>

namespace sofaimgui::render
{

namespace
{
    void implInit()
    {
#if SOFAIMGUI_FORCE_OPENGL2 == 1
        ImGui_ImplOpenGL2_Init();
#else
        ImGui_ImplOpenGL3_Init(nullptr);
#endif
    }
    void implNewFrame()
    {
#if SOFAIMGUI_FORCE_OPENGL2 == 1
        ImGui_ImplOpenGL2_NewFrame();
#else
        ImGui_ImplOpenGL3_NewFrame();
#endif
    }
    void implRender(ImDrawData* dd)
    {
#if SOFAIMGUI_FORCE_OPENGL2 == 1
        ImGui_ImplOpenGL2_RenderDrawData(dd);
#else
        ImGui_ImplOpenGL3_RenderDrawData(dd);
#endif
    }
    void implShutdown()
    {
#if SOFAIMGUI_FORCE_OPENGL2 == 1
        ImGui_ImplOpenGL2_Shutdown();
#else
        ImGui_ImplOpenGL3_Shutdown();
#endif
    }
    void implRecreateFonts()
    {
#if SOFAIMGUI_FORCE_OPENGL2 == 1
        ImGui_ImplOpenGL2_DestroyFontsTexture();
        ImGui_ImplOpenGL2_CreateFontsTexture();
#else
        ImGui_ImplOpenGL3_DestroyFontsTexture();
        ImGui_ImplOpenGL3_CreateFontsTexture();
#endif
    }
}

ImGuiPlatformGL::~ImGuiPlatformGL()
{
    if (m_pbosInitialized)
    {
        glDeleteBuffers(s_NB_PBOS, m_pbos);
        m_pbosInitialized = false;
    }
}

void ImGuiPlatformGL::initBackend(GLFWwindow* window)
{
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    implInit();

    glGenBuffers(s_NB_PBOS, m_pbos);
    m_pbosInitialized = true;
}

void ImGuiPlatformGL::newFrame()
{
    implNewFrame();
}

void ImGuiPlatformGL::renderDrawData(ImDrawData* drawData)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    implRender(drawData);
}

void ImGuiPlatformGL::shutdown()
{
    if (m_pbosInitialized)
    {
        glDeleteBuffers(s_NB_PBOS, m_pbos);
        m_pbosInitialized = false;
    }
    implShutdown();
}

void ImGuiPlatformGL::recreateFontsTexture()
{
    implRecreateFonts();
}

void ImGuiPlatformGL::beginSceneTarget(int width, int height)
{
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    const unsigned int w = static_cast<unsigned int>(std::max(1, width));
    const unsigned int h = static_cast<unsigned int>(std::max(1, height));

    if (!m_fbo)
    {
        m_fbo = std::make_unique<sofa::gl::FrameBufferObject>();
        m_currentFBOSize = {w, h};
        m_fbo->init(m_currentFBOSize.first, m_currentFBOSize.second);
    }
    else if (m_currentFBOSize.first != w || m_currentFBOSize.second != h)
    {
        m_fbo->setSize(w, h);
        m_currentFBOSize = {w, h};
    }

    m_fbo->start();
}

void ImGuiPlatformGL::endSceneTarget()
{
    if (!m_fbo)
        return;

    // Clear the alpha component so ImGui does not treat the image as transparent.
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    m_fbo->stop();
}

ImTextureID ImGuiPlatformGL::sceneTexture() const
{
    if (!m_fbo)
        return static_cast<ImTextureID>(0);
    return static_cast<ImTextureID>(const_cast<sofa::gl::FrameBufferObject*>(m_fbo.get())->getColorTexture());
}

ImGuiDockNodeFlags ImGuiPlatformGL::dockspaceFlags() const
{
    // GL renders the scene into the pass-through central node.
    return ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode;
}

void ImGuiPlatformGL::requestScreenshot(const std::string& path)
{
    if (!m_fbo)
        return;

    sofa::helper::io::STBImage image;
    image.init(m_currentFBOSize.first, m_currentFBOSize.second, 1, 1,
        sofa::helper::io::Image::DataType::UINT32, sofa::helper::io::Image::ChannelFormat::RGBA);

    glBindTexture(GL_TEXTURE_2D, m_fbo->getColorTexture());
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.getPixels());
    glBindTexture(GL_TEXTURE_2D, 0);

    image.save(path, 90);
}

void ImGuiPlatformGL::pumpScreenshot(uint32_t presentedFrame)
{
    // GL screenshots are synchronous (handled in requestScreenshot).
    SOFA_UNUSED(presentedFrame);
}

sofa::type::Vec2i ImGuiPlatformGL::readSceneTargetPixels(std::vector<uint8_t>& pixels)
{
    if (!m_fbo)
        return {0, 0};

    const int readIndex = m_frameCount % s_NB_PBOS;
    const int processIndex = (m_frameCount + 1) % s_NB_PBOS;
    ++m_frameCount;

    m_fbo->start();

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    if (m_pboSize[0] != viewport[2] || m_pboSize[1] != viewport[3])
    {
        const int size = viewport[2] * viewport[3] * 4;
        for (int i = 0; i < s_NB_PBOS; i++)
        {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, m_pbos[i]);
            glBufferData(GL_PIXEL_PACK_BUFFER, size, NULL, GL_STREAM_READ);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

        m_pboSize[0] = viewport[2];
        m_pboSize[1] = viewport[3];
    }

    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // Read to PBO (asynchronous), then map and copy the previous frame.
    glBindBuffer(GL_PIXEL_PACK_BUFFER, m_pbos[readIndex]);
    glReadPixels(0, 0, viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, 0);

    glBindBuffer(GL_PIXEL_PACK_BUFFER, m_pbos[processIndex]);
    void* data = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    if (data)
    {
        const int size = viewport[2] * viewport[3] * 4;
        pixels.resize(size);
        memcpy(pixels.data(), data, size);
        glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    m_fbo->stop();

    return {viewport[2], viewport[3]};
}

// Self-register at static-init time.
static const ImGuiPlatformRegistrar s_registrar(
    sofaglfw::render::RenderAPI::OpenGL,
    [] { return std::unique_ptr<IImGuiPlatform>(new ImGuiPlatformGL()); });

} // namespace sofaimgui::render
