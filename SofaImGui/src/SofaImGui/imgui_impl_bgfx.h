// Derived from this Gist by pr0g:
//     https://gist.github.com/pr0g/aff79b71bf9804ddb03f39ca7c0c3bbb
// Originally from Richard Gale:
//     https://gist.github.com/RichardGale/6e2b74bc42b3005e08397236e4be0fd0

#pragma once

#include <imgui.h>

IMGUI_IMPL_API void ImGui_Implbgfx_Init(int view);
IMGUI_IMPL_API void ImGui_Implbgfx_Shutdown();
IMGUI_IMPL_API void ImGui_Implbgfx_NewFrame();
IMGUI_IMPL_API void ImGui_Implbgfx_RenderDrawLists(ImDrawData* draw_data);

IMGUI_IMPL_API void ImGui_Implbgfx_InvalidateDeviceObjects();
IMGUI_IMPL_API bool ImGui_Implbgfx_CreateDeviceObjects();
IMGUI_IMPL_API bool ImGui_Implbgfx_CreateFontsTexture();
IMGUI_IMPL_API void ImGui_Implbgfx_DestroyFontsTexture();
