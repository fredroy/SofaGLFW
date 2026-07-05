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
#include <SofaImGui/render/ImGuiPlatformFactory.h>
#include <SofaImGui/render/IImGuiPlatform.h>

#include <sofa/helper/logging/Messaging.h>

using sofaglfw::render::RenderAPI;

namespace sofaimgui::render
{

ImGuiPlatformFactory& ImGuiPlatformFactory::getInstance()
{
    static ImGuiPlatformFactory instance;
    return instance;
}

void ImGuiPlatformFactory::registerPlatform(RenderAPI api, Creator creator)
{
    if (api == RenderAPI::BGFX)
        m_bgfx = std::move(creator);
    else if (api == RenderAPI::OpenGL)
        m_opengl = std::move(creator);
}

bool ImGuiPlatformFactory::isAvailable(RenderAPI api) const
{
    if (api == RenderAPI::BGFX)
        return static_cast<bool>(m_bgfx);
    if (api == RenderAPI::OpenGL)
        return static_cast<bool>(m_opengl);
    return static_cast<bool>(m_bgfx) || static_cast<bool>(m_opengl);
}

std::unique_ptr<IImGuiPlatform> ImGuiPlatformFactory::create(RenderAPI api) const
{
    if (api == RenderAPI::BGFX && m_bgfx)
        return m_bgfx();
    if (api == RenderAPI::OpenGL && m_opengl)
        return m_opengl();

    msg_error("ImGuiPlatformFactory")
        << "No ImGui platform available for the "
        << sofaglfw::render::toString(api) << " backend.";
    return nullptr;
}

ImGuiPlatformRegistrar::ImGuiPlatformRegistrar(RenderAPI api,
                                               ImGuiPlatformFactory::Creator creator)
{
    ImGuiPlatformFactory::getInstance().registerPlatform(api, std::move(creator));
}

} // namespace sofaimgui::render
