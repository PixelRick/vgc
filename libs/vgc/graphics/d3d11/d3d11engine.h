// Copyright 2022 The VGC Developers
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/vgc/vgc/blob/master/COPYRIGHT
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VGC_GRAPHICS_D3D11_D3D11ENGINE_H
#define VGC_GRAPHICS_D3D11_D3D11ENGINE_H

#include <vgc/core/compiler.h>
#ifdef VGC_CORE_COMPILER_MSVC

#include <d3d11.h>

#include <array>
#include <chrono>
#include <memory>

#include <vgc/core/color.h>
#include <vgc/core/paths.h>
#include <vgc/geometry/mat4d.h>
#include <vgc/geometry/mat4f.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/geometry/vec2f.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/engine.h>
#include <vgc/graphics/detail/comptr.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(D3d11Engine);

/// \class vgc::widget::D3d11Engine
/// \brief The D3D11-based graphics::Engine.
///
/// This class is an implementation of Engine using Direct3D 11.0.
///
class VGC_GRAPHICS_API D3d11Engine : public Engine {
private:
    VGC_OBJECT(D3d11Engine, Engine)

protected:
    D3d11Engine();

    void onDestroyed() override;

public:
    /// Creates a new D3d11Engine.
    ///
    static D3d11EnginePtr create();

protected:
    // Implementation of Engine API

    // -- USER THREAD implementation functions --

    void createBuiltinShaders_() override;

    SwapChainPtr createSwapChain_(const SwapChainCreateInfo& createInfo) override;
    FramebufferPtr createFramebuffer_(const ImageViewPtr& colorImageView) override;
    BufferPtr createBuffer_(const BufferCreateInfo& createInfo) override;
    ImagePtr createImage_(const ImageCreateInfo& createInfo) override;
    ImageViewPtr createImageView_(const ImageViewCreateInfo& createInfo, const ImagePtr& image) override;
    ImageViewPtr createImageView_(const ImageViewCreateInfo& createInfo, const BufferPtr& buffer, ImageFormat format, UInt32 elementsCount) override;
    SamplerStatePtr createSamplerState_(const SamplerStateCreateInfo& createInfo) override;
    GeometryViewPtr createGeometryView_(const GeometryViewCreateInfo& createInfo) override;
    BlendStatePtr createBlendState_(const BlendStateCreateInfo& createInfo) override;
    RasterizerStatePtr createRasterizerState_(const RasterizerStateCreateInfo& createInfo) override;

    void resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height) override;

    //--  RENDER THREAD implementation functions --

    void initBuiltinShaders_() override;

    void initFramebuffer_(Framebuffer* framebuffer) override;
    void initBuffer_(Buffer* buffer, const Span<const char>* dataSpan, Int initialLengthInBytes) override;
    void initImage_(Image* image, const Span<const Span<const char>>* dataSpanSpan) override;
    void initImageView_(ImageView* view) override;
    void initSamplerState_(SamplerState* state) override;
    void initGeometryView_(GeometryView* view) override;
    void initBlendState_(BlendState* state) override;
    void initRasterizerState_(RasterizerState* state) override;

    void setSwapChain_(SwapChain* swapChain) override;
    void setFramebuffer_(Framebuffer* framebuffer) override;
    void setViewport_(Int x, Int y, Int width, Int height) override;
    void setProgram_(Program* program) override;
    void setBlendState_(BlendState* state, const geometry::Vec4f& blendFactor) override;
    void setRasterizerState_(RasterizerState* state) override;
    void setStageConstantBuffers_(Buffer* const* buffers, Int startIndex, Int count, ShaderStage shaderStage) override;
    void setStageImageViews_(ImageView* const* views, Int startIndex, Int count, ShaderStage shaderStage) override;
    void setStageSamplers_(SamplerState* const* states, Int startIndex, Int count, ShaderStage shaderStage) override;

    void updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes) override;

    void draw_(GeometryView* view, UInt primitiveCount, UInt instanceCount) override;
    void clear_(const core::Color& color) override;

    UInt64 present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags flags) override;

private:
    ComPtr<IDXGIFactory> factory_;
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> deviceCtx_;
    ComPtr<ID3D11DepthStencilState> depthStencilState_;

    bool loadBuffer_(ID3D11Buffer** bufferPtr, D3D11_BUFFER_DESC* desc, const void* data, Int dataSize);
    bool writeBufferReserved_(ID3D11Buffer* buffer, const void* data, Int dataSize);
};

} // namespace vgc::graphics

#endif // VGC_CORE_COMPILER_MSVC
#endif // VGC_GRAPHICS_D3D11_D3D11ENGINE_H
