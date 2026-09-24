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
#include <bgfx/c99/bgfx.h>

#include <cstdint>

namespace sofaglfw::render
{

/// The bgfx callback interface of the backend (bgfx_init_t::callback): it saves the
/// back buffer screenshots of bgfx_request_screen_shot() via sofa::helper::io, and
/// reports fatal errors and aborts.
bgfx_callback_interface_t* bgfxCallback();

/// Save 8-bit RGBA (or BGRA) pixels read back from bgfx as an image file.
/// @param pitch bytes per row of @p pixels
/// @param bottomUp the rows of @p pixels start at the bottom of the image
SOFAGLFW_API void saveRgba8Screenshot(const char* path, uint32_t width, uint32_t height, uint32_t pitch,
                                      const uint8_t* pixels, bool bgra, bool bottomUp);

} // namespace sofaglfw::render
