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
#include <SofaGLFW/render/RenderBackendFactory.h>
#include <SofaGLFW/render/IRenderBackend.h>
#include <SofaGLFW/render/ISceneRenderer.h>
#include <SofaGLFW/render/IVideoRecorder.h>

#include <sofa/helper/logging/Messaging.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace sofaglfw::render
{

RenderAPI parseRenderAPI(std::string_view name)
{
    std::string lower(name);
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (lower == "opengl" || lower == "gl")
        return RenderAPI::OpenGL;
    if (lower == "bgfx")
        return RenderAPI::BGFX;
    return RenderAPI::Auto;
}

RenderBackendFactory& RenderBackendFactory::getInstance()
{
    static RenderBackendFactory instance;
    return instance;
}

RenderBackendFactory::Entry& RenderBackendFactory::entry(RenderAPI api)
{
    // OpenGL and bgfx are the only concrete backends; Auto maps to OpenGL's slot
    // only as a harmless default and is never registered against.
    return (api == RenderAPI::BGFX) ? m_bgfx : m_opengl;
}

const RenderBackendFactory::Entry* RenderBackendFactory::entryIfAvailable(RenderAPI api) const
{
    if (api == RenderAPI::BGFX && m_bgfx.registered)
        return &m_bgfx;
    if (api == RenderAPI::OpenGL && m_opengl.registered)
        return &m_opengl;
    return nullptr;
}

void RenderBackendFactory::registerBackend(RenderAPI api,
                                           BackendCreator backendCreator,
                                           SceneRendererCreator sceneCreator)
{
    Entry& e = entry(api);
    e.backend = std::move(backendCreator);
    e.scene = std::move(sceneCreator);
    e.registered = true;
}

bool RenderBackendFactory::isAvailable(RenderAPI api) const
{
    if (api == RenderAPI::Auto)
        return m_bgfx.registered || m_opengl.registered;
    return entryIfAvailable(api) != nullptr;
}

RenderAPI RenderBackendFactory::resolve(RenderAPI preferred) const
{
    // 1) explicit environment override always wins, if it names an available backend
    if (const char* env = std::getenv("SOFA_RENDER_API"))
    {
        const RenderAPI fromEnv = parseRenderAPI(env);
        if (fromEnv != RenderAPI::Auto)
        {
            if (isAvailable(fromEnv))
                return fromEnv;
            msg_warning("RenderBackendFactory")
                << "SOFA_RENDER_API=" << env << " requests the "
                << toString(fromEnv) << " backend, which is not compiled in. Ignoring.";
        }
    }

    // 2) caller preference, if available
    if (preferred != RenderAPI::Auto && isAvailable(preferred))
        return preferred;

    // 3) compiled-in default: bgfx if present, else OpenGL
    if (m_bgfx.registered)
        return RenderAPI::BGFX;
    if (m_opengl.registered)
        return RenderAPI::OpenGL;

    return RenderAPI::Auto; // nothing available
}

std::unique_ptr<IRenderBackend> RenderBackendFactory::createBackend(RenderAPI api) const
{
    const RenderAPI resolved = resolve(api);
    const Entry* e = entryIfAvailable(resolved);
    if (!e)
    {
        msg_error("RenderBackendFactory") << "No rendering backend available.";
        return nullptr;
    }
    return e->backend();
}

std::unique_ptr<ISceneRenderer> RenderBackendFactory::createSceneRenderer(RenderAPI api) const
{
    const RenderAPI resolved = resolve(api);
    const Entry* e = entryIfAvailable(resolved);
    if (!e)
    {
        msg_error("RenderBackendFactory") << "No rendering backend available.";
        return nullptr;
    }
    return e->scene();
}

namespace
{
    VideoRecorderCreator s_videoRecorderCreator = nullptr;
}

void registerVideoRecorder(VideoRecorderCreator creator)
{
    s_videoRecorderCreator = creator;
}

std::unique_ptr<IVideoRecorder> createVideoRecorder()
{
    if (s_videoRecorderCreator)
        return s_videoRecorderCreator();
    return nullptr;
}

RenderBackendRegistrar::RenderBackendRegistrar(RenderAPI api,
                                               RenderBackendFactory::BackendCreator backendCreator,
                                               RenderBackendFactory::SceneRendererCreator sceneCreator)
{
    RenderBackendFactory::getInstance().registerBackend(api, std::move(backendCreator), std::move(sceneCreator));
}

} // namespace sofaglfw::render
