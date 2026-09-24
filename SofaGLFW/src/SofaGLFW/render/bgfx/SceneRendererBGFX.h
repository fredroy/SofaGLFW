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

#include <bgfx/c99/bgfx.h>
#include <BGFXPlugin/Context.h>
#include <BGFXPlugin/Texture.h>

#include <map>
#include <memory>
#include <string>

namespace sofaglfw::render
{

/// bgfx implementation of ISceneRenderer: the background (color, gradient or image
/// quad), the scene, its transparent models and DrawToolBGFX's overlays in their own
/// views (below), with the camera matrices and the depth-range remap of non-GL
/// renderers.
class SOFAGLFW_API SceneRendererBGFX : public ISceneRenderer, public bgfxplugin::GpuResourceOwner
{
public:
    /// bgfx views of the scene, all rendered into the scene target, in this order:
    /// background, scene, transparent models, then the overlays of DrawToolBGFX
    /// (OglSceneFrame, color map legends...).
    static constexpr uint16_t kViewBackground = 0;
    static constexpr uint16_t kViewScene = 1;
    static constexpr uint16_t kViewTransparent = 2;
    static constexpr uint16_t kFirstOverlayView = 3;
    static constexpr uint16_t kOverlayViewCount = 8;
    static constexpr uint16_t kSceneViewCount = kFirstOverlayView + kOverlayViewCount;

    /// Framebuffer pixels per window unit (ImGui's FramebufferScale): the factor from
    /// GLFW/ImGui sizes to render target pixels. 2 on a Retina Mac, but 1 on Windows
    /// and X11, where window sizes are already pixels whatever the content scale.
    static void framebufferScale(GLFWwindow* window, float& xscale, float& yscale);

    SceneRendererBGFX() = default;
    ~SceneRendererBGFX() override;

    void drawScene(sofa::simulation::Node* groot,
                   sofa::core::visual::VisualParams* vparams,
                   sofa::component::visual::BaseCamera* camera,
                   GLFWwindow* glfwWindow,
                   const ViewportRect& viewport,
                   const sofa::type::RGBAColor& background) override;

    void setBackgroundImage(const std::string& filename) override;
    void clearBackgroundImage() override;
    void releaseResources() override;
    void releaseGpuResources() override { releaseResources(); }

private:
    /// The rect is in framebuffer pixels; the scale converts it back to logical pixels
    /// so the image tiles at one texel per logical pixel.
    /// The background image quad over a (width, height) render target, in the
    /// background view; false when there is no image (or program) to draw.
    bool drawBackgroundImage(uint16_t width, uint16_t height, float xscale, float yscale);

    struct Background
    {
        std::unique_ptr<bgfxplugin::Texture> texture;
    };

    std::map<std::string, Background> m_backgrounds;
    std::string m_currentBackgroundFilename{};
    bgfx_program_handle_t m_bgProgram{UINT16_MAX};
    bgfx_uniform_handle_t m_bgTexUniform{UINT16_MAX};
    bool m_bgProgramTried{false}; ///< loaded once per context: a missing shader is not retried every frame
};

} // namespace sofaglfw::render
