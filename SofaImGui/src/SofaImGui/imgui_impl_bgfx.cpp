// Derived from this Gist by pr0g:
//     https://gist.github.com/pr0g/aff79b71bf9804ddb03f39ca7c0c3bbb
// Originally from Richard Gale:
//     https://gist.github.com/RichardGale/6e2b74bc42b3005e08397236e4be0fd0
// Adapted for ImGui 1.91.x

#include "imgui_impl_bgfx.h"

#include <bgfx/bgfx.h>
#include <bgfx/embedded_shader.h>
#include <bx/math.h>

#include <cstring>

#include "vs_ocornut_imgui.bin.h"
#include "fs_ocornut_imgui.bin.h"

static const bgfx::EmbeddedShader s_embeddedShaders[] =
{
    BGFX_EMBEDDED_SHADER(vs_ocornut_imgui),
    BGFX_EMBEDDED_SHADER(fs_ocornut_imgui),
    BGFX_EMBEDDED_SHADER_END()
};

static bgfx::TextureHandle g_FontTexture = BGFX_INVALID_HANDLE;
static bgfx::ProgramHandle g_ShaderHandle = BGFX_INVALID_HANDLE;
static bgfx::UniformHandle g_AttribLocationTex = BGFX_INVALID_HANDLE;
static bgfx::VertexLayout  g_VertexLayout;
static uint8_t g_View = 255;

void ImGui_Implbgfx_RenderDrawLists(ImDrawData* draw_data)
{
    const int fb_width  = static_cast<int>(draw_data->DisplaySize.x * draw_data->FramebufferScale.x);
    const int fb_height = static_cast<int>(draw_data->DisplaySize.y * draw_data->FramebufferScale.y);
    if (fb_width <= 0 || fb_height <= 0)
        return;

    const bgfx::Caps* caps = bgfx::getCaps();

    const float L = draw_data->DisplayPos.x;
    const float T = draw_data->DisplayPos.y;
    const float R = L + draw_data->DisplaySize.x;
    const float B = T + draw_data->DisplaySize.y;

    float ortho[16];
    bx::mtxOrtho(ortho, L, R, B, T, 0.0f, 1000.0f, 0.0f, caps->homogeneousDepth);

    bgfx::setViewTransform(g_View, nullptr, ortho);
    bgfx::setViewRect(g_View, 0, 0, static_cast<uint16_t>(fb_width), static_cast<uint16_t>(fb_height));

    constexpr uint64_t state = BGFX_STATE_WRITE_RGB
        | BGFX_STATE_WRITE_A
        | BGFX_STATE_MSAA
        | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA);

    const ImVec2 clip_off   = draw_data->DisplayPos;
    const ImVec2 clip_scale = draw_data->FramebufferScale;

    for (int n = 0; n < draw_data->CmdListsCount; ++n)
    {
        const ImDrawList* cmd_list = draw_data->CmdLists[n];

        const auto numVertices = static_cast<uint32_t>(cmd_list->VtxBuffer.Size);
        const auto numIndices  = static_cast<uint32_t>(cmd_list->IdxBuffer.Size);

        if (numVertices != bgfx::getAvailTransientVertexBuffer(numVertices, g_VertexLayout)
            || numIndices != bgfx::getAvailTransientIndexBuffer(numIndices))
            break;

        bgfx::TransientVertexBuffer tvb;
        bgfx::TransientIndexBuffer  tib;

        bgfx::allocTransientVertexBuffer(&tvb, numVertices, g_VertexLayout);
        bgfx::allocTransientIndexBuffer(&tib, numIndices);

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
            bgfx::setScissor(sc_x, sc_y,
                static_cast<uint16_t>(bx::min(clip_max.x, 65535.0f) - sc_x),
                static_cast<uint16_t>(bx::min(clip_max.y, 65535.0f) - sc_y));

            bgfx::setState(state);

            bgfx::TextureHandle texture = {
                static_cast<uint16_t>(static_cast<uint64_t>(pcmd->GetTexID()) & 0xffff) };
            bgfx::setTexture(0, g_AttribLocationTex, texture);

            bgfx::setVertexBuffer(0, &tvb, pcmd->VtxOffset, numVertices - pcmd->VtxOffset);
            bgfx::setIndexBuffer(&tib, pcmd->IdxOffset, pcmd->ElemCount);
            bgfx::submit(g_View, g_ShaderHandle);
        }
    }

    if (draw_data->CmdListsCount == 0)
        bgfx::touch(g_View);
}

bool ImGui_Implbgfx_CreateFontsTexture()
{
    ImGuiIO& io = ImGui::GetIO();

    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    g_FontTexture = bgfx::createTexture2D(
        static_cast<uint16_t>(width),
        static_cast<uint16_t>(height),
        false, 1,
        bgfx::TextureFormat::RGBA8,
        0,
        bgfx::copy(pixels, width * height * 4));

    io.Fonts->SetTexID(static_cast<ImTextureID>(g_FontTexture.idx));

    return true;
}

void ImGui_Implbgfx_DestroyFontsTexture()
{
    if (bgfx::isValid(g_FontTexture))
    {
        bgfx::destroy(g_FontTexture);
        g_FontTexture = BGFX_INVALID_HANDLE;
        ImGui::GetIO().Fonts->SetTexID(0);
    }
}

bool ImGui_Implbgfx_CreateDeviceObjects()
{
    const bgfx::RendererType::Enum type = bgfx::getRendererType();

    g_ShaderHandle = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, type, "vs_ocornut_imgui"),
        bgfx::createEmbeddedShader(s_embeddedShaders, type, "fs_ocornut_imgui"),
        true);

    g_VertexLayout
        .begin()
            .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
        .end();

    g_AttribLocationTex = bgfx::createUniform("s_tex", bgfx::UniformType::Sampler);

    if (!bgfx::isValid(g_FontTexture))
        ImGui_Implbgfx_CreateFontsTexture();

    return true;
}

void ImGui_Implbgfx_InvalidateDeviceObjects()
{
    ImGui_Implbgfx_DestroyFontsTexture();

    if (bgfx::isValid(g_ShaderHandle))
    {
        bgfx::destroy(g_ShaderHandle);
        g_ShaderHandle = BGFX_INVALID_HANDLE;
    }

    if (bgfx::isValid(g_AttribLocationTex))
    {
        bgfx::destroy(g_AttribLocationTex);
        g_AttribLocationTex = BGFX_INVALID_HANDLE;
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
    if (!bgfx::isValid(g_ShaderHandle))
    {
        ImGui_Implbgfx_CreateDeviceObjects();
    }
}
