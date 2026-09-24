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
#include <SofaGLFW/render/bgfx/BgfxCallback.h>

#include <sofa/helper/io/STBImage.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace sofaglfw::render
{

namespace
{

struct BgfxCallback : bgfx_callback_interface_t
{
    static bgfx_callback_vtbl_t s_vtbl;

    BgfxCallback()
    {
        vtbl = &s_vtbl;
    }

    /// bgfx is in an undefined state after a fatal error and expects the callback not
    /// to return (its default one aborts). It may be called from the render thread,
    /// hence stderr rather than SOFA's messaging. Debug checks (BGFX_CONFIG_DEBUG
    /// builds) are reported and execution goes on, as with bgfx's default callback
    /// outside a debugger.
    static void fatal(bgfx_callback_interface_t*, const char* filePath, uint16_t line, bgfx_fatal_t code,
                      const char* str)
    {
        std::fprintf(stderr, "[bgfx] %s(%u): fatal error %d: %s\n", filePath ? filePath : "?", unsigned(line),
                     int(code), str ? str : "");
        std::fflush(stderr);
        if (code != BGFX_FATAL_DEBUG_CHECK)
            std::abort();
    }
    static void traceVargs(bgfx_callback_interface_t*, const char*, uint16_t, const char*, va_list) {}
    static void profilerBegin(bgfx_callback_interface_t*, const char*, uint32_t, const char*, uint16_t) {}
    static void profilerBeginLiteral(bgfx_callback_interface_t*, const char*, uint32_t, const char*, uint16_t) {}
    static void profilerEnd(bgfx_callback_interface_t*) {}
    static uint32_t cacheReadSize(bgfx_callback_interface_t*, uint64_t) { return 0; }
    static bool cacheRead(bgfx_callback_interface_t*, uint64_t, void*, uint32_t) { return false; }
    static void cacheWrite(bgfx_callback_interface_t*, uint64_t, const void*, uint32_t) {}

    static void screenShot(bgfx_callback_interface_t*, const char* filePath, uint32_t width,
        uint32_t height, uint32_t pitch, bgfx_texture_format_t format, const void* data,
        uint32_t /*size*/, bool yflip)
    {
        if (!filePath || !data)
            return;

        sofa::helper::io::STBImage image;
        image.init(width, height, 1, 1,
            sofa::helper::io::Image::DataType::UINT32,
            sofa::helper::io::Image::ChannelFormat::RGBA);

        const uint8_t* src = static_cast<const uint8_t*>(data);
        uint8_t* dst = image.getPixels();
        const uint32_t dstPitch = width * 4;

        for (uint32_t row = 0; row < height; ++row)
        {
            uint32_t srcRow = yflip ? row : (height - 1 - row);
            const uint8_t* srcLine = src + srcRow * pitch;

            if (format == BGFX_TEXTURE_FORMAT_RGBA8)
            {
                memcpy(dst + row * dstPitch, srcLine, dstPitch);
            }
            else if (format == BGFX_TEXTURE_FORMAT_BGRA8)
            {
                for (uint32_t x = 0; x < width; ++x)
                {
                    dst[row * dstPitch + x * 4 + 0] = srcLine[x * 4 + 2];
                    dst[row * dstPitch + x * 4 + 1] = srcLine[x * 4 + 1];
                    dst[row * dstPitch + x * 4 + 2] = srcLine[x * 4 + 0];
                    dst[row * dstPitch + x * 4 + 3] = srcLine[x * 4 + 3];
                }
            }
        }
        image.save(filePath, 90);
    }

    static void captureBegin(bgfx_callback_interface_t*, uint32_t, uint32_t, uint32_t, bgfx_texture_format_t, bool) {}
    static void captureEnd(bgfx_callback_interface_t*) {}
    static void captureFrame(bgfx_callback_interface_t*, const void*, uint32_t) {}
};

bgfx_callback_vtbl_t BgfxCallback::s_vtbl = {
    BgfxCallback::fatal,
    BgfxCallback::traceVargs,
    BgfxCallback::profilerBegin,
    BgfxCallback::profilerBeginLiteral,
    BgfxCallback::profilerEnd,
    BgfxCallback::cacheReadSize,
    BgfxCallback::cacheRead,
    BgfxCallback::cacheWrite,
    BgfxCallback::screenShot,
    BgfxCallback::captureBegin,
    BgfxCallback::captureEnd,
    BgfxCallback::captureFrame,
};

BgfxCallback s_bgfxCallback;

} // anonymous namespace

bgfx_callback_interface_t* bgfxCallback()
{
    return &s_bgfxCallback;
}

} // namespace sofaglfw::render
