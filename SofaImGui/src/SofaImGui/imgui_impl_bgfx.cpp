// Derived from this Gist by pr0g:
//     https://gist.github.com/pr0g/aff79b71bf9804ddb03f39ca7c0c3bbb
// Originally from Richard Gale:
//     https://gist.github.com/RichardGale/6e2b74bc42b3005e08397236e4be0fd0
// Adapted for ImGui 1.91.x

#include "imgui_impl_bgfx.h"

#include <bgfx/c99/bgfx.h>
#include <bx/math.h>

#include <BGFXPlugin/BGFXShaderUtils.h>

#include <cstring>
#include <string>

static bgfx_texture_handle_t g_FontTexture = BGFX_INVALID_HANDLE;
static bgfx_program_handle_t g_ShaderHandle = BGFX_INVALID_HANDLE;
static bgfx_uniform_handle_t g_AttribLocationTex = BGFX_INVALID_HANDLE;
static bgfx_vertex_layout_t  g_VertexLayout;
static uint8_t g_View = 255;

void ImGui_Implbgfx_RenderDrawLists(ImDrawData* draw_data)
{
    const int fb_width  = static_cast<int>(draw_data->DisplaySize.x * draw_data->FramebufferScale.x);
    const int fb_height = static_cast<int>(draw_data->DisplaySize.y * draw_data->FramebufferScale.y);
    if (fb_width <= 0 || fb_height <= 0)
        return;

    const bgfx_caps_t* caps = bgfx_get_caps();

    const float L = draw_data->DisplayPos.x;
    const float T = draw_data->DisplayPos.y;
    const float R = L + draw_data->DisplaySize.x;
    const float B = T + draw_data->DisplaySize.y;

    float ortho[16];
    bx::mtxOrtho(ortho, L, R, B, T, 0.0f, 1000.0f, 0.0f, caps->homogeneousDepth);

    bgfx_set_view_transform(g_View, nullptr, ortho);
    bgfx_set_view_rect(g_View, 0, 0, static_cast<uint16_t>(fb_width), static_cast<uint16_t>(fb_height), 0.0f, 1.0f);

    constexpr uint64_t state = BGFX_STATE_WRITE_RGB
        | BGFX_STATE_WRITE_A
        | BGFX_STATE_MSAA
        // As ImGui's own backends (glBlendFuncSeparate): the alpha is
        // src + dst * (1 - src), so an opaque window stays opaque under translucent
        // widgets (the screenshot PNGs keep no see-through holes).
        | BGFX_STATE_BLEND_FUNC_SEPARATE(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA,
                                         BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);

    const ImVec2 clip_off   = draw_data->DisplayPos;
    const ImVec2 clip_scale = draw_data->FramebufferScale;

    for (int n = 0; n < draw_data->CmdListsCount; ++n)
    {
        const ImDrawList* cmd_list = draw_data->CmdLists[n];

        const auto numVertices = static_cast<uint32_t>(cmd_list->VtxBuffer.Size);
        const auto numIndices  = static_cast<uint32_t>(cmd_list->IdxBuffer.Size);

        if (numVertices != bgfx_get_avail_transient_vertex_buffer(numVertices, &g_VertexLayout)
            || numIndices != bgfx_get_avail_transient_index_buffer(numIndices, false))
            break;

        bgfx_transient_vertex_buffer_t tvb;
        bgfx_transient_index_buffer_t  tib;

        bgfx_alloc_transient_vertex_buffer(&tvb, numVertices, &g_VertexLayout);
        bgfx_alloc_transient_index_buffer(&tib, numIndices, false);

        memcpy(tvb.data, cmd_list->VtxBuffer.Data, numVertices * sizeof(ImDrawVert));
        memcpy(tib.data, cmd_list->IdxBuffer.Data, numIndices * sizeof(ImDrawIdx));

        for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; ++cmd_i)
        {
            const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];

            if (pcmd->UserCallback)
            {
                pcmd->UserCallback(cmd_list, pcmd);
                continue;
            }

            ImVec2 clip_min(
                (pcmd->ClipRect.x - clip_off.x) * clip_scale.x,
                (pcmd->ClipRect.y - clip_off.y) * clip_scale.y);
            ImVec2 clip_max(
                (pcmd->ClipRect.z - clip_off.x) * clip_scale.x,
                (pcmd->ClipRect.w - clip_off.y) * clip_scale.y);

            if (clip_max.x <= clip_min.x || clip_max.y <= clip_min.y)
                continue;

            const auto sc_x = static_cast<uint16_t>(bx::max(clip_min.x, 0.0f));
            const auto sc_y = static_cast<uint16_t>(bx::max(clip_min.y, 0.0f));
            bgfx_set_scissor(sc_x, sc_y,
                static_cast<uint16_t>(bx::min(clip_max.x, 65535.0f) - sc_x),
                static_cast<uint16_t>(bx::min(clip_max.y, 65535.0f) - sc_y));

            bgfx_set_state(state, 0);

            const bgfx_texture_handle_t texture = {
                static_cast<uint16_t>(static_cast<uint64_t>(pcmd->GetTexID()) & 0xffff) };
            bgfx_set_texture(0, g_AttribLocationTex, texture, UINT32_MAX);

            bgfx_set_transient_vertex_buffer(0, &tvb, pcmd->VtxOffset, numVertices - pcmd->VtxOffset);
            bgfx_set_transient_index_buffer(&tib, pcmd->IdxOffset, pcmd->ElemCount);
            bgfx_submit(g_View, g_ShaderHandle, 0, BGFX_DISCARD_ALL);
        }
    }

    if (draw_data->CmdListsCount == 0)
        bgfx_touch(g_View);
}

bool ImGui_Implbgfx_CreateFontsTexture()
{
    ImGuiIO& io = ImGui::GetIO();

    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    g_FontTexture = bgfx_create_texture_2d(
        static_cast<uint16_t>(width),
        static_cast<uint16_t>(height),
        false, 1,
        BGFX_TEXTURE_FORMAT_RGBA8,
        0,
        bgfx_copy(pixels, width * height * 4), 0);

    io.Fonts->SetTexID(static_cast<ImTextureID>(g_FontTexture.idx));

    return true;
}

void ImGui_Implbgfx_DestroyFontsTexture()
{
    if (BGFX_HANDLE_IS_VALID(g_FontTexture))
    {
        bgfx_destroy_texture(g_FontTexture);
        g_FontTexture.idx = UINT16_MAX;
        ImGui::GetIO().Fonts->SetTexID(0);
    }
}

bool ImGui_Implbgfx_CreateDeviceObjects()
{
    // The ImGui shaders are compiled by the build with the pinned bgfx's shaderc (see
    // SofaImGui/CMakeLists.txt), and installed next to this library. Precompiled
    // embedded copies would be rejected as soon as bgfx changes its shader binary format.
    static const int anchor = 0;
    static const std::string shadersDir = bgfxplugin::findDataDirectory(
        &anchor, "share/sofa/SofaImGui/shaders", SOFAIMGUI_SHADERS_DIR);
    g_ShaderHandle = bgfxplugin::loadProgram("vs_ocornut_imgui", "fs_ocornut_imgui", shadersDir);

    bgfx_vertex_layout_begin(&g_VertexLayout, BGFX_RENDERER_TYPE_NOOP);
    bgfx_vertex_layout_add(&g_VertexLayout, BGFX_ATTRIB_POSITION, 2, BGFX_ATTRIB_TYPE_FLOAT, false, false);
    bgfx_vertex_layout_add(&g_VertexLayout, BGFX_ATTRIB_TEXCOORD0, 2, BGFX_ATTRIB_TYPE_FLOAT, false, false);
    bgfx_vertex_layout_add(&g_VertexLayout, BGFX_ATTRIB_COLOR0, 4, BGFX_ATTRIB_TYPE_UINT8, true, false);
    bgfx_vertex_layout_end(&g_VertexLayout);

    g_AttribLocationTex = bgfx_create_uniform("s_tex", BGFX_UNIFORM_TYPE_SAMPLER, 1);

    if (!BGFX_HANDLE_IS_VALID(g_FontTexture))
        ImGui_Implbgfx_CreateFontsTexture();

    return true;
}

void ImGui_Implbgfx_InvalidateDeviceObjects()
{
    ImGui_Implbgfx_DestroyFontsTexture();

    if (BGFX_HANDLE_IS_VALID(g_ShaderHandle))
    {
        bgfx_destroy_program(g_ShaderHandle);
        g_ShaderHandle.idx = UINT16_MAX;
    }

    if (BGFX_HANDLE_IS_VALID(g_AttribLocationTex))
    {
        bgfx_destroy_uniform(g_AttribLocationTex);
        g_AttribLocationTex.idx = UINT16_MAX;
    }
}

void ImGui_Implbgfx_Init(const int view)
{
    g_View = static_cast<uint8_t>(view & 0xff);

    ImGuiIO& io = ImGui::GetIO();
    io.BackendRendererName = "imgui_impl_bgfx";
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
}

void ImGui_Implbgfx_Shutdown()
{
    ImGui_Implbgfx_InvalidateDeviceObjects();
}

void ImGui_Implbgfx_NewFrame()
{
    if (!BGFX_HANDLE_IS_VALID(g_ShaderHandle))
    {
        ImGui_Implbgfx_CreateDeviceObjects();
    }
}
