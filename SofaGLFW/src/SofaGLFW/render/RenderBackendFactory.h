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
#include <SofaGLFW/render/RenderAPI.h>

#include <functional>
#include <memory>

namespace sofaglfw::render
{

class IRenderBackend;
class ISceneRenderer;

/// Runtime registry + factory for rendering backends.
///
/// Each backend translation unit self-registers its constructors at static-init
/// time via RenderBackendRegistrar. This file (and the shared GUI code) contains
/// no backend includes and no #ifdef: which backends exist is discovered purely
/// from what got compiled and linked in.
class SOFAGLFW_API RenderBackendFactory
{
public:
    using BackendCreator = std::function<std::unique_ptr<IRenderBackend>()>;
    using SceneRendererCreator = std::function<std::unique_ptr<ISceneRenderer>()>;

    static RenderBackendFactory& getInstance();

    /// Register the constructors for one backend. Called by RenderBackendRegistrar.
    void registerBackend(RenderAPI api,
                         BackendCreator backendCreator,
                         SceneRendererCreator sceneCreator);

    /// True if the given concrete backend was compiled and registered.
    bool isAvailable(RenderAPI api) const;

    /// Resolve RenderAPI::Auto to a concrete, available backend, honoring
    /// (in order): the SOFA_RENDER_API env var, the provided preference, then
    /// the compiled-in default (bgfx if present, else OpenGL).
    /// @return a concrete API, or RenderAPI::Auto if none is available.
    RenderAPI resolve(RenderAPI preferred = RenderAPI::Auto) const;

    /// Create a render backend. Auto is resolved via resolve(). @return nullptr
    /// if no backend is available.
    std::unique_ptr<IRenderBackend> createBackend(RenderAPI api = RenderAPI::Auto) const;

    /// Create a scene renderer for the same backend. @see createBackend.
    std::unique_ptr<ISceneRenderer> createSceneRenderer(RenderAPI api = RenderAPI::Auto) const;

private:
    RenderBackendFactory() = default;

    struct Entry
    {
        BackendCreator backend;
        SceneRendererCreator scene;
        bool registered{false};
    };
    Entry& entry(RenderAPI api);
    const Entry* entryIfAvailable(RenderAPI api) const;

    Entry m_opengl;
    Entry m_bgfx;
};

/// Static-init helper: instantiate one at file scope in each backend TU to
/// register that backend. Example (in RenderBackendBGFX.cpp):
///   static const RenderBackendRegistrar s_reg(
///       RenderAPI::BGFX,
///       [] { return std::make_unique<RenderBackendBGFX>(); },
///       [] { return std::make_unique<SceneRendererBGFX>(); });
struct SOFAGLFW_API RenderBackendRegistrar
{
    RenderBackendRegistrar(RenderAPI api,
                           RenderBackendFactory::BackendCreator backendCreator,
                           RenderBackendFactory::SceneRendererCreator sceneCreator);
};

} // namespace sofaglfw::render
