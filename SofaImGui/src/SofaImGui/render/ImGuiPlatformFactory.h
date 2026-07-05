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

#include <SofaImGui/config.h>
#include <SofaGLFW/render/RenderAPI.h>

#include <functional>
#include <memory>

namespace sofaimgui::render
{

class IImGuiPlatform;

/// Runtime registry + factory for ImGui platform backends, mirroring
/// sofaglfw::render::RenderBackendFactory. Each ImGui platform TU self-registers
/// at static-init time; this file has no backend includes and no #ifdef.
class SOFAIMGUI_API ImGuiPlatformFactory
{
public:
    using Creator = std::function<std::unique_ptr<IImGuiPlatform>()>;
    using RenderAPI = sofaglfw::render::RenderAPI;

    static ImGuiPlatformFactory& getInstance();

    void registerPlatform(RenderAPI api, Creator creator);
    bool isAvailable(RenderAPI api) const;

    /// Create the ImGui platform for the given (already-resolved) backend.
    /// Pass the concrete RenderAPI chosen by RenderBackendFactory so the ImGui
    /// backend matches the render backend. @return nullptr if unavailable.
    std::unique_ptr<IImGuiPlatform> create(RenderAPI api) const;

private:
    ImGuiPlatformFactory() = default;

    Creator m_opengl;
    Creator m_bgfx;
};

/// Static-init helper; instantiate one at file scope in each ImGui platform TU.
struct SOFAIMGUI_API ImGuiPlatformRegistrar
{
    ImGuiPlatformRegistrar(sofaglfw::render::RenderAPI api,
                           ImGuiPlatformFactory::Creator creator);
};

} // namespace sofaimgui::render
