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
#include <SofaGLFW/render/bgfx/BgfxNativeWindow.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <bx/platform.h>

#if BX_PLATFORM_LINUX
// GLFW >= 3.4 picks X11 or Wayland at runtime (SOFAGLFW_USEX11 forces X11): both
// are exposed, Wayland when its client headers are available (glfw3native.h needs them).
#    define GLFW_EXPOSE_NATIVE_X11
#    define GLFW_EXPOSE_NATIVE_GLX
#    if __has_include(<wayland-client.h>)
#        define GLFW_EXPOSE_NATIVE_WAYLAND
#        define SOFAGLFW_BGFX_WAYLAND 1
#    endif
#elif BX_PLATFORM_OSX
#    define GLFW_EXPOSE_NATIVE_COCOA
#    define GLFW_EXPOSE_NATIVE_NSGL
#elif BX_PLATFORM_WINDOWS
#    define GLFW_EXPOSE_NATIVE_WIN32
#    define GLFW_EXPOSE_NATIVE_WGL
#endif

#include <GLFW/glfw3native.h>

namespace sofaglfw::render
{

void* bgfxNativeWindowHandle(GLFWwindow* window)
{
#if BX_PLATFORM_LINUX
#    if SOFAGLFW_BGFX_WAYLAND
    // bgfx takes the wl_surface itself (and wraps it in a wl_egl_window for OpenGL).
    if (glfwGetPlatform() == GLFW_PLATFORM_WAYLAND)
        return glfwGetWaylandWindow(window);
#    endif
    return (void*)(uintptr_t)glfwGetX11Window(window);
#elif BX_PLATFORM_OSX
    return glfwGetCocoaWindow(window);
#elif BX_PLATFORM_WINDOWS
    return glfwGetWin32Window(window);
#else
    return nullptr;
#endif
}

void* bgfxNativeDisplayHandle()
{
#if BX_PLATFORM_LINUX
#    if SOFAGLFW_BGFX_WAYLAND
    if (glfwGetPlatform() == GLFW_PLATFORM_WAYLAND)
        return glfwGetWaylandDisplay();
#    endif
    return glfwGetX11Display();
#else
    return nullptr;
#endif
}

bgfx_native_window_handle_type bgfxNativeWindowHandleType()
{
#if BX_PLATFORM_LINUX && SOFAGLFW_BGFX_WAYLAND
    if (glfwGetPlatform() == GLFW_PLATFORM_WAYLAND)
        return bgfx_native_window_handle_type::BGFX_NATIVE_WINDOW_HANDLE_TYPE_WAYLAND;
    return bgfx_native_window_handle_type::BGFX_NATIVE_WINDOW_HANDLE_TYPE_DEFAULT;
#else
    return bgfx_native_window_handle_type::BGFX_NATIVE_WINDOW_HANDLE_TYPE_DEFAULT;
#endif
}

} // namespace sofaglfw::render
