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

#include <bgfx/c99/bgfx.h>

struct GLFWwindow;

namespace sofaglfw::render
{

/// Platform-specific native handles needed to hand the GLFW surface to bgfx.
/// The platform #if lives here, isolated in a bgfx-only translation unit.
void* bgfxNativeWindowHandle(GLFWwindow* window);
void* bgfxNativeDisplayHandle();
bgfx_native_window_handle_type bgfxNativeWindowHandleType();

} // namespace sofaglfw::render
