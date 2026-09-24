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

#include <sofa/helper/logging/Messaging.h>

#include <algorithm>
#include <cstring>
#include <ios>

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
    releaseMsaaTarget();
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

void ImGuiPlatformGL::releaseMsaaTarget()
{
    if (m_msaaFbo)
        glDeleteFramebuffersEXT(1, &m_msaaFbo);
    if (m_msaaColor)
        glDeleteRenderbuffersEXT(1, &m_msaaColor);
    if (m_msaaDepth)
        glDeleteRenderbuffersEXT(1, &m_msaaDepth);
    m_msaaFbo = m_msaaColor = m_msaaDepth = 0;
    m_msaaSamples = 0;
    m_msaaSize = {0, 0};
}

bool ImGuiPlatformGL::ensureMsaaTarget(unsigned int width, unsigned int height, int samples)
{
    // GL_MAX_SAMPLES_EXT is queried once, and only with MSAA on: without
    // EXT_framebuffer_multisample it is an invalid enum, and the GL error left pending
    // would be reported by the next glGetError of a scene component.
    if (samples >= 2 && m_maxSamples < 0)
    {
        GLint maxSamples = 0;
        glGetIntegerv(GL_MAX_SAMPLES_EXT, &maxSamples);
        if (glGetError() != GL_NO_ERROR)
            maxSamples = 0;
        m_maxSamples = maxSamples;
    }
    samples = std::min(samples, std::max(m_maxSamples, 0));
    if (samples < 2 || m_msaaFailed)
    {
        releaseMsaaTarget();
        return false;
    }
    if (m_msaaFbo && m_msaaSamples == samples && m_msaaSize == std::make_pair(width, height))
        return true;

    releaseMsaaTarget();
    glGenRenderbuffersEXT(1, &m_msaaColor);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, m_msaaColor);
    glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, samples, GL_RGBA8, GLsizei(width), GLsizei(height));
    glGenRenderbuffersEXT(1, &m_msaaDepth);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, m_msaaDepth);
    glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, samples, GL_DEPTH_COMPONENT24, GLsizei(width), GLsizei(height));
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

    GLint previous = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &previous);
    glGenFramebuffersEXT(1, &m_msaaFbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_msaaFbo);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, m_msaaColor);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, m_msaaDepth);
    const GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, GLuint(previous));
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT)
    {
        msg_warning("ImGuiPlatformGL") << "Multisampled scene target incomplete (status 0x" << std::hex << status
                                       << "): the scene is drawn without MSAA.";
        releaseMsaaTarget();
        m_msaaFailed = true;
        return false;
    }
    m_msaaSamples = samples;
    m_msaaSize = {width, height};
    return true;
}

void ImGuiPlatformGL::beginSceneTarget(int width, int height, int msaa)
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

    // With MSAA (Settings > Rendering), the scene goes to a multisampled target of
    // the same size, resolved into m_fbo by endSceneTarget().
    m_msaaActive = ensureMsaaTarget(w, h, msaa);
    if (m_msaaActive)
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_msaaFbo);
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

    if (m_msaaActive)
    {
        const GLint w = GLint(m_msaaSize.first), h = GLint(m_msaaSize.second);
        glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, m_msaaFbo);
        glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, m_fbo->getID());
        glBlitFramebufferEXT(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbo->getID());
        m_msaaActive = false;
    }

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
