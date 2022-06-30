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

#ifndef VGC_GRAPHICS_D3D11ENGINE_H
#define VGC_GRAPHICS_D3D11ENGINE_H

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

namespace vgc::graphics {

VGC_DECLARE_OBJECT(D3d11Engine);

template<typename T>
class ComPtr {
public:
    ComPtr() noexcept = default;
    ComPtr(T* p) : p_(p) {
        if (p_) {
            p_->AddRef();
        }
    }

    ~ComPtr() {
        if (p_) {
            p_->Release();
        }
    }

    ComPtr(const ComPtr&) = delete;
    ComPtr& operator=(const ComPtr&) = delete;

    ComPtr& operator=(T* p) {
        reset();
        p_ = p;
        if (p_) {
            p_->AddRef();
        }
    }

    T* get() const {
        return p_;
    }

    void reset() {
        if (p_) {
            p_->Release();
            p_ = nullptr;
        }
    }

    T** addressOf() {
        reset();
        return &p_;
    }

    explicit operator bool() const noexcept {
        return p_ != nullptr;
    }

    T& operator*() const noexcept {
        return *p_;
    }

    T* operator->() const noexcept {
        return p_;
    }

    T* const* operator&() {
        return &p_;
    }

private:
    T* p_ = nullptr;
};

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

    // USER THREAD pimpl functions

    SwapChain* createSwapChain_(const SwapChainDesc& desc) override;
    void resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height) override;
    Buffer* createBuffer_(Usage usage, BindFlags bindFlags, CpuAccessFlags cpuAccessFlags) override;

    // RENDER THREAD functions

    void bindSwapChain_(SwapChain* swapChain) override;
    UInt64 present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags flags) override;
    void bindFramebuffer_(Framebuffer* framebuffer) override;
    void setViewport_(Int x, Int y, Int width, Int height) override;
    void clear_(const core::Color& color) override;
    void setProjectionMatrix_(const geometry::Mat4f& m) override;
    void setViewMatrix_(const geometry::Mat4f& m) override;
    void bindPaintShader_() override;
    void releasePaintShader_() override;
    void initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes) override;
    void updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes) override;
    void setupVertexBufferForPaintShader_(Buffer* buffer) override;
    void drawPrimitives_(Buffer* buffer, PrimitiveType type) override;

private:
    ComPtr<IDXGIFactory> factory_;
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> deviceCtx_;
    ComPtr<ID3D11RasterizerState> rasterizerState_;
    ComPtr<ID3D11BlendState> blendState_;
    ComPtr<ID3D11DepthStencilState> depthStencilState_;
    ComPtr<ID3D11InputLayout> inputLayout_;

    ComPtr<ID3D11VertexShader> vertexShader_;
    ComPtr<ID3D11Buffer> vertexConstantBuffer_;
    ComPtr<ID3D11PixelShader> pixelShader_;
    struct PaintVertexShaderConstantBuffer {
        std::array<float, 16> projMatrix;
        std::array<float, 16> viewMatrix;
    } paintVertexShaderConstantBuffer_;

    std::chrono::steady_clock::time_point startTime_;

    bool loadBuffer_(ID3D11Buffer** bufferPtr, D3D11_BUFFER_DESC* desc, const void* data, Int dataSize);
    bool writeBufferReserved_(ID3D11Buffer* buffer, const void* data, Int dataSize);
};

} // namespace vgc::graphics

#endif // VGC_CORE_COMPILER_MSVC
#endif // VGC_GRAPHICS_D3D11ENGINE_H
