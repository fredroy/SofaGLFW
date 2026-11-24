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
#include <SofaImGui/windows/FrozenWindows.h>

#include <imgui_internal.h>

#include <algorithm>

namespace windows
{

void FrozenWindows::track(const std::string& windowName)
{
    m_tracked.push_back(windowName);
}

void FrozenWindows::captureDrawLists(ImGuiWindow* window, std::vector<DrawList>& drawLists)
{
    // As ImGui adds a window to the draw data: its draw list, then its child windows'.
    if (!window->Active || window->Hidden)
        return;

    const ImDrawList* source = window->DrawList;
    DrawList& kept = drawLists.emplace_back();
    kept.vertices.assign(source->VtxBuffer.Data, source->VtxBuffer.Data + source->VtxBuffer.Size);
    kept.indices.assign(source->IdxBuffer.Data, source->IdxBuffer.Data + source->IdxBuffer.Size);
    for (const ImDrawCmd& command : source->CmdBuffer)
    {
        // Callbacks (render state resets) are not replayed: their data may be gone.
        if (command.UserCallback == nullptr && command.ElemCount > 0)
            kept.commands.push_back(command);
    }

    for (ImGuiWindow* child : window->DC.ChildWindows)
    {
        if (!(child->Flags & (ImGuiWindowFlags_Popup | ImGuiWindowFlags_Tooltip)))
            captureDrawLists(child, drawLists);
    }
}

void FrozenWindows::capture()
{
    for (const std::string& name : m_tracked)
    {
        ImGuiWindow* window = ImGui::FindWindowByName(name.c_str());
        if (!window || !window->Active)
        {
            m_windows.erase(name); // closed: drawn from scratch when shown again
            continue;
        }

        Window& kept = m_windows[name];
        kept.position = window->Pos;
        kept.contentSize = window->ContentSize;
        kept.drawLists.clear();
        captureDrawLists(window, kept.drawLists);
    }
    m_tracked.clear();
}

void FrozenWindows::clear()
{
    m_tracked.clear();
    m_windows.clear();
}

void FrozenWindows::show(const char* windowName, WindowState& winManager)
{
    if (!*winManager.getStatePtr())
        return;

    const auto it = m_windows.find(windowName);
    if (it == m_windows.end())
    {
        // Opened during the step, or before the first frame with the scene.
        if (ImGui::Begin(windowName, winManager.getStatePtr()))
        {
            ImGui::TextDisabled("The simulation is stepping...");
            ImGui::TextDisabled("Shown at the end of the step.");
        }
        ImGui::End();
        return;
    }

    const Window& kept = it->second;
    // The content's size as when it was drawn: without it, the window would compute an
    // empty one, and lose its scrolling position.
    ImGui::SetNextWindowContentSize(kept.contentSize);
    if (ImGui::Begin(windowName, winManager.getStatePtr()))
    {
        ImGuiWindow* window = ImGui::GetCurrentWindow();
        const ImVec2 offset(window->Pos.x - kept.position.x, window->Pos.y - kept.position.y);
        ImDrawList* target = window->DrawList;

        for (const DrawList& drawList : kept.drawLists)
        {
            for (const ImDrawCmd& command : drawList.commands)
            {
                // The vertices the command uses: a range, from its offset.
                const ImDrawIdx* indices = drawList.indices.data() + command.IdxOffset;
                ImDrawIdx first = indices[0], last = indices[0];
                for (unsigned int i = 1; i < command.ElemCount; ++i)
                {
                    first = std::min(first, indices[i]);
                    last = std::max(last, indices[i]);
                }
                const int numVertices = int(last) - int(first) + 1;

                // Clipped to the window too, if it shrank since.
                target->PushClipRect(ImVec2(command.ClipRect.x + offset.x, command.ClipRect.y + offset.y),
                                     ImVec2(command.ClipRect.z + offset.x, command.ClipRect.w + offset.y), true);
                target->PushTextureID(command.TextureId);
                target->PrimReserve(int(command.ElemCount), numVertices);

                const ImDrawVert* vertices = drawList.vertices.data() + command.VtxOffset + first;
                for (int v = 0; v < numVertices; ++v)
                {
                    ImDrawVert vertex = vertices[v];
                    vertex.pos.x += offset.x;
                    vertex.pos.y += offset.y;
                    target->_VtxWritePtr[v] = vertex;
                }
                const unsigned int base = target->_VtxCurrentIdx;
                for (unsigned int i = 0; i < command.ElemCount; ++i)
                    target->_IdxWritePtr[i] = static_cast<ImDrawIdx>(base + indices[i] - first);
                target->_VtxWritePtr += numVertices;
                target->_IdxWritePtr += command.ElemCount;
                target->_VtxCurrentIdx += static_cast<unsigned int>(numVertices);

                target->PopTextureID();
                target->PopClipRect();
            }
        }
    }
    ImGui::End();
}

bool FrozenWindows::isKept(const ImGuiWindow* window) const
{
    if (!window)
        return false;
    window = window->RootWindow;
    for (const auto& [name, kept] : m_windows)
        if (name == window->Name)
            return true;
    return false;
}

bool FrozenWindows::wantsScene() const
{
    if (m_windows.empty() || !ImGui::GetCurrentContext())
        return false;
    const ImGuiContext& g = *GImGui;

    // A widget of a kept window in use: a slider dragged, a text edited, a scrollbar
    // dragged... Not the window itself being moved, which works without the scene.
    if (g.ActiveId != 0 && g.MovingWindow == nullptr && isKept(g.ActiveIdWindow))
        return true;

    // A click or the wheel over a kept window, where the mouse is when it happens.
    ImVec2 mouse = g.IO.MousePos;
    for (const ImGuiInputEvent& event : g.InputEventsQueue)
    {
        if (event.Type == ImGuiInputEventType_MousePos)
            mouse = ImVec2(event.MousePos.PosX, event.MousePos.PosY);
        else if ((event.Type == ImGuiInputEventType_MouseButton && event.MouseButton.Down)
                 || event.Type == ImGuiInputEventType_MouseWheel)
        {
            ImGuiWindow* hovered = nullptr;
            ImGuiWindow* hoveredUnderMoving = nullptr;
            ImGui::FindHoveredWindowEx(mouse, false, &hovered, &hoveredUnderMoving);
            if (isKept(hovered))
                return true;
        }
    }
    return false;
}

} // namespace windows
