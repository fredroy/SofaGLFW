/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
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
#include <vector>

struct GLFWwindow;

namespace sofaglfw
{

class SofaGLFWBaseGUI;

class BaseGUIEngine
{
public:
    
    virtual void init() = 0;
    virtual void initBackend(GLFWwindow*) = 0;
    /// Builds the frame (the GUI panels read and write the scene): called with the
    /// scene lock held, after the scene was drawn; or without the scene, if
    /// drawsFramesWithoutScene() (SofaGLFWBaseGUI::isSceneAvailable() tells which), in
    /// which case it must not touch the scene.
    virtual void startFrame(SofaGLFWBaseGUI*) = 0;
    virtual void endFrame() = 0;
    /// Submits and presents the frame built by startFrame(), without the scene lock:
    /// must not touch the scene (a simulation thread may be stepping).
    virtual void presentFrame(SofaGLFWBaseGUI*) {}
    /// Whether the engine draws frames without the scene (SofaGLFWBaseGUI::isSceneAvailable
    /// false, during a long step of the simulation thread): startFrame and presentFrame
    /// are then called without beforeDraw/afterDraw and the scene draw. Otherwise, no
    /// frame is drawn meanwhile, and the window keeps showing the last one.
    virtual bool drawsFramesWithoutScene() const { return false; }
    virtual void beforeDraw(GLFWwindow* window) = 0;
    /// beforeDraw() for a frame that draws the scene again from recorded draw calls
    /// (SofaGLFWWindow::replay), without the scene: must not touch it, nor the visual
    /// parameters, which the simulation thread reads. afterDraw() ends it.
    /// @param viewport where the scene goes, in window units
    /// @return false if the engine draws no scene in frames without it
    virtual bool beforeReplay(GLFWwindow* window, render::ViewportRect& viewport)
    {
        SOFA_UNUSED(window); SOFA_UNUSED(viewport);
        return false;
    }
    virtual void afterDraw() = 0;
    virtual void terminate() = 0;
    virtual bool isTerminated() const = 0;
    virtual bool dispatchMouseEvents() = 0;
    virtual void resetCounter() = 0;
    virtual sofa::type::Vec2i getFrameBufferPixels(std::vector<uint8_t>& pixels) = 0;
    virtual void openFile(SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot) { SOFA_UNUSED(baseGUI); SOFA_UNUSED(groot); };
    virtual void loadFile(SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot, std::string filePathName, bool reload = false)
    { SOFA_UNUSED(baseGUI); SOFA_UNUSED(groot); SOFA_UNUSED(filePathName); SOFA_UNUSED(reload); };
    virtual void contentScaleChanged(float xscale, float yscale) { SOFA_UNUSED(xscale); SOFA_UNUSED(yscale); };

    /// Rendering options that must be known BEFORE the graphics engine is
    /// initialized (e.g. bgfx bakes vsync into the Metal layer at init time, so
    /// applying it afterwards leaves the window display-synced for a while). The
    /// GUI engine that owns the settings (ImGui) overrides this to report the
    /// persisted preference. Vsync is off by default, as on master: the simulation
    /// steps once per frame, so vsync would cap it at the display refresh rate.
    struct InitialRenderConfig
    {
        bool vsync{false};
        int  msaa{0};
        /// Run the simulation in its own thread (SofaGLFWBaseGUI::setAsynchronousSimulation).
        bool asynchronousSimulation{false};
    };
    virtual InitialRenderConfig getInitialRenderConfig() const { return {}; }
    virtual void saveNamedScreenshot(SofaGLFWBaseGUI * baseGUI, std::string filename = std::string(""), int compression_level = -1)
    { SOFA_UNUSED(baseGUI); SOFA_UNUSED(filename); SOFA_UNUSED(compression_level); };
    virtual void setSimulation(sofa::core::sptr<sofa::simulation::Node>& groot, const std::string& filename = std::string("")) { SOFA_UNUSED(groot); SOFA_UNUSED(filename); };
};

} // namespace sofaglfw
