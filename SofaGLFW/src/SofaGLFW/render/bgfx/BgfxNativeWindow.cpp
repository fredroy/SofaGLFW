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
#    if ENTRY_CONFIG_USE_WAYLAND
#        include <wayland-egl.h>
#        define GLFW_EXPOSE_NATIVE_WAYLAND
#    else
#        define GLFW_EXPOSE_NATIVE_X11
#        define GLFW_EXPOSE_NATIVE_GLX
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
#    if ENTRY_CONFIG_USE_WAYLAND
    wl_egl_window* win_impl = (wl_egl_window*)glfwGetWindowUserPointer(window);
    if (!win_impl)
    {
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        struct wl_surface* surface = (struct wl_surface*)glfwGetWaylandWindow(window);
        if (!surface)
            return nullptr;
        win_impl = wl_egl_window_create(surface, width, height);
        glfwSetWindowUserPointer(window, (void*)(uintptr_t)win_impl);
    }
    return (void*)(uintptr_t)win_impl;
#    else
    return (void*)(uintptr_t)glfwGetX11Window(window);
#    endif
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
#    if ENTRY_CONFIG_USE_WAYLAND
    return glfwGetWaylandDisplay();
#    else
    return glfwGetX11Display();
#    endif
#else
    return nullptr;
#endif
}

bgfx_native_window_handle_type bgfxNativeWindowHandleType()
{
#if BX_PLATFORM_LINUX
#    if ENTRY_CONFIG_USE_WAYLAND
    return bgfx_native_window_handle_type::BGFX_NATIVE_WINDOW_HANDLE_TYPE_WAYLAND;
#    else
    return bgfx_native_window_handle_type::BGFX_NATIVE_WINDOW_HANDLE_TYPE_DEFAULT;
#    endif
#else
    return bgfx_native_window_handle_type::BGFX_NATIVE_WINDOW_HANDLE_TYPE_DEFAULT;
#endif
}

} // namespace sofaglfw::render
