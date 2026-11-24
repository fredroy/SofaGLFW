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
#include <SofaImGui/windows/WindowState.h>

#include <imgui.h>

#include <map>
#include <string>
#include <vector>

struct ImGuiWindow;

namespace windows
{

/// The windows that read the scene, as they were drawn in the last frame that had it,
/// for the frames that go without it during a long step of the simulation thread (see
/// SofaGLFWBaseGUI::isSceneAvailable).
///
/// In a frame with the scene, a window is drawn by its own code, then track() names it,
/// and capture() keeps what it drew (its draw lists and those of its child windows) once
/// the frame is rendered. In a frame without the scene, show() draws the same window,
/// which keeps its place, title and docking, with that content in it: the window looks
/// as it was, moved with the window if it moved. Its widgets do not work then: the input
/// meant for them is detected by wantsScene() before the frame, which then waits for the
/// scene instead.
class SOFAIMGUI_API FrozenWindows
{
public:
    /// In a frame with the scene: @p windowName was drawn, to capture().
    void track(const std::string& windowName);
    /// After ImGui::Render() of a frame with the scene: keeps what the tracked windows drew.
    void capture();
    /// Forgets every window (another scene).
    void clear();

    /// In a frame without the scene, instead of the window's code: the window with its
    /// content of the last frame with the scene (a placeholder before any).
    void show(const char* windowName, WindowState& winManager);

    /// Before a frame: whether the input not yet processed by ImGui is meant for a window
    /// kept here (a click or the wheel over it, a widget of it in use): that frame should
    /// have the scene, for the window's own code to get the input.
    bool wantsScene() const;

private:
    struct DrawList
    {
        std::vector<ImDrawCmd> commands;
        std::vector<ImDrawVert> vertices;
        std::vector<ImDrawIdx> indices;
    };

    struct Window
    {
        ImVec2 position;    ///< of the window when captured: the draw lists are in screen space
        ImVec2 contentSize; ///< kept in the frames without the scene, for the scrolling to stay
        std::vector<DrawList> drawLists; ///< the window's, then its child windows', in drawing order
    };

    static void captureDrawLists(ImGuiWindow* window, std::vector<DrawList>& drawLists);
    bool isKept(const ImGuiWindow* window) const;

    std::vector<std::string> m_tracked;
    std::map<std::string, Window> m_windows;
};

} // namespace windows
