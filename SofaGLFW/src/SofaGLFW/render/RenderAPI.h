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

#include <string_view>

namespace sofaglfw::render
{

/// Identifies a rendering backend. This enum is backend-agnostic on purpose: it
/// carries no OpenGL nor bgfx type, so it can be included by any shared file and
/// shared across the SofaGLFW and SofaImGui plugins.
enum class RenderAPI
{
    Auto,   ///< Resolve at runtime (env var, then .ini, then compiled-in default)
    OpenGL, ///< Legacy OpenGL (requires Sofa.GL)
    BGFX    ///< bgfx (requires BGFXPlugin)
};

inline std::string_view toString(RenderAPI api)
{
    switch (api)
    {
    case RenderAPI::OpenGL: return "OpenGL";
    case RenderAPI::BGFX:   return "bgfx";
    case RenderAPI::Auto:   return "auto";
    }
    return "auto";
}

/// Parse a case-insensitive backend name ("opengl"/"gl", "bgfx") into a RenderAPI.
/// Anything unrecognized (including empty) resolves to RenderAPI::Auto.
RenderAPI parseRenderAPI(std::string_view name);

} // namespace sofaglfw::render
