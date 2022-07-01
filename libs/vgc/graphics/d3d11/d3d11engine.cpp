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

#include <vgc/core/compiler.h>
#ifdef VGC_CORE_COMPILER_MSVC

#include <vgc/graphics/d3d11/d3d11engine.h>

#include <array>
#include <chrono>

#include <d3d11.h>
#include <d3dcompiler.h>
#pragma comment(lib, "d3dcompiler")
#pragma comment(lib, "d3d11")

#include <vgc/core/exceptions.h>
#include <vgc/core/paths.h>

namespace vgc::graphics {

namespace {

// core::resourcePath("graphics/d3d11/" + name);

struct XYRGBVertex {
    float x, y, r, g, b;
};

} // namespace

class D3d11Buffer : public Buffer {
public:
    D3d11Buffer(
        ResourceList* owningList,
        Usage usage,
        BindFlags bindFlags,
        CpuAccessFlags cpuAccessFlags);

    ID3D11Buffer* object() const {
        return object_.get();
    }

    ID3D11Buffer** objectAddress() {
        return object_.addressOf();
    }

    const D3D11_BUFFER_DESC& desc() const {
        return desc_;
    }

    D3D11_BUFFER_DESC* descAddress() {
        return &desc_;
    }

protected:
    void release_(Engine* /*engine*/) override {
        object_.reset();
    }

private:
    friend D3d11Engine;

    ComPtr<ID3D11Buffer> object_;
    D3D11_BUFFER_DESC desc_ = {};
};

class D3d11ImageView : public ImageView {
public:
    D3d11ImageView(ResourceList* owningList, ID3D11View* view)
        : ImageView(owningList)
        , view_(view)
    {}

    ID3D11View* view() const {
        return view_.get();
    }

    // resizing a swap chain requires releasing all views to its back buffers.
    void forceRelease() {
        view_.reset();
    }

protected:
    void release_(Engine* /*engine*/) override {
        forceRelease();
    }

private:
    ComPtr<ID3D11View> view_;
};
using D3d11ImageViewPtr = ResourcePtr<D3d11ImageView>;

// no equivalent in D3D11, see OMSetRenderTargets
class D3d11Framebuffer : public Framebuffer {
public:
    D3d11Framebuffer(
        ResourceList* owningList,
        const D3d11ImageViewPtr& colorView,
        const D3d11ImageViewPtr& depthStencilView,
        bool isDefault = false)
        : Framebuffer(owningList)
        , colorView_(colorView)
        , depthStencilView_(depthStencilView)
        , isDefault_(isDefault)
    {}

    bool isDefault() const {
        return isDefault_;
    }

    ID3D11RenderTargetView* colorView() const {
        return colorView_ ?
            static_cast<ID3D11RenderTargetView*>(colorView_->view()) : nullptr;
    }

    ID3D11DepthStencilView* depthStencilView() const {
        return depthStencilView_ ?
            static_cast<ID3D11DepthStencilView*>(depthStencilView_->view()) : nullptr;
    }

    // resizing a swap chain requires releasing all views to its back buffers.
    void forceReleaseColorView() {
        colorView_->forceRelease();
    }

protected:
    void clearSubResources_() override
    {
        colorView_.reset();
        depthStencilView_.reset();
    }

    void release_(Engine* /*engine*/) override {}

private:
    D3d11ImageViewPtr colorView_;
    D3d11ImageViewPtr depthStencilView_;

    bool isDefault_ = false;
};
using D3d11FramebufferPtr = ResourcePtr<D3d11Framebuffer>;

class D3d11SwapChain : public SwapChain {
public:
    D3d11SwapChain(ResourceList* owningList,
        const SwapChainDesc& desc,
        IDXGISwapChain* dxgiSwapChain)
        : SwapChain(owningList, desc)
        , dxgiSwapChain_(dxgiSwapChain)
    {}

    IDXGISwapChain* dxgiSwapChain() const {
        return dxgiSwapChain_.get();
    }

    D3d11Framebuffer* d3dDefaultFrameBuffer() const {
        return static_cast<D3d11Framebuffer*>(defaultFrameBuffer_.get());
    }

    // can't be called from render thread
    void setDefaultFramebuffer(const D3d11FramebufferPtr& defaultFrameBuffer) {
        defaultFrameBuffer_ = defaultFrameBuffer;
    }

    // can't be called from render thread
    void clearDefaultFramebuffer()
    {
        if (defaultFrameBuffer_) {
            d3dDefaultFrameBuffer()->forceReleaseColorView();
            defaultFrameBuffer_.reset();
        }
    }

protected:
    void release_(Engine* /*engine*/) override {}

private:
    ComPtr<IDXGISwapChain> dxgiSwapChain_;
};

D3d11Buffer::D3d11Buffer(
    ResourceList* owningList,
    Usage usage,
    BindFlags bindFlags,
    CpuAccessFlags cpuAccessFlags)
    : Buffer(owningList, usage, bindFlags, cpuAccessFlags)
{
    switch (usage) {
    case Usage::Default:
        desc_.Usage = D3D11_USAGE::D3D11_USAGE_DEFAULT; break;
    case Usage::Immutable:
        desc_.Usage = D3D11_USAGE::D3D11_USAGE_IMMUTABLE; break;
    case Usage::Dynamic:
        desc_.Usage = D3D11_USAGE::D3D11_USAGE_DYNAMIC; break;
    case Usage::Staging:
        desc_.Usage = D3D11_USAGE::D3D11_USAGE_STAGING; break;
    default:
        throw core::LogicError("D3d11Buffer: unsupported usage");
    }

    switch (bindFlags) {
    case BindFlags::VertexBuffer:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_VERTEX_BUFFER; break;
    case BindFlags::IndexBuffer:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_INDEX_BUFFER; break;
    case BindFlags::UniformBuffer:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER; break;
    case BindFlags::ShaderResource:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_SHADER_RESOURCE; break;
    case BindFlags::StreamOutput:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_STREAM_OUTPUT; break;
    case BindFlags::RenderTarget:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_RENDER_TARGET; break;
    case BindFlags::DepthStencil:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_DEPTH_STENCIL; break;
    case BindFlags::UnorderedAccess:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_UNORDERED_ACCESS; break;
    case BindFlags::DecoderOutput:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_DECODER; break;
    case BindFlags::EncoderInput:
        desc_.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_VIDEO_ENCODER; break;
    default:
        throw core::LogicError("D3d11Buffer: unsupported bind flags");
    }

    if (!!(cpuAccessFlags & CpuAccessFlags::Write)) {
        desc_.CPUAccessFlags |= D3D11_CPU_ACCESS_FLAG::D3D11_CPU_ACCESS_WRITE;
    }
    if (!!(cpuAccessFlags & CpuAccessFlags::Read)) {
        desc_.CPUAccessFlags |= D3D11_CPU_ACCESS_FLAG::D3D11_CPU_ACCESS_READ;
    }
}

D3d11Engine::D3d11Engine()
{
    // XXX add success checks (S_OK)

    startTime_ = std::chrono::steady_clock::now();

    const D3D_FEATURE_LEVEL featureLevels[1] = { D3D_FEATURE_LEVEL_11_0 };
    D3D11CreateDevice(
        NULL,
        D3D_DRIVER_TYPE_HARDWARE,
        NULL,
        D3D11_CREATE_DEVICE_DEBUG |
        0, // could use D3D11_CREATE_DEVICE_SINGLETHREADED
           // if we defer creation of buffers and swapchain.
        featureLevels, 1,
        D3D11_SDK_VERSION,
        device_.addressOf(),
        NULL,
        deviceCtx_.addressOf());

    // Retrieve DXGI factory from device.
    ComPtr<IDXGIDevice> dxgiDevice;
    ComPtr<IDXGIAdapter> dxgiAdapter;
    device_->QueryInterface(IID_PPV_ARGS(dxgiDevice.addressOf()));
    dxgiDevice->GetParent(IID_PPV_ARGS(dxgiAdapter.addressOf()));
    dxgiAdapter->GetParent(IID_PPV_ARGS(factory_.addressOf()));

    // Create the paint vertex shader
    {
        static const char* vertexShader = R"hlsl(

            cbuffer vertexBuffer : register(b0)
            {
                float4x4 projMatrix;
                float4x4 viewMatrix;
            };
            struct VS_INPUT
            {
                float2 pos : POSITION;
                float4 col : COLOR0;
            };
            struct PS_INPUT
            {
                float4 pos : SV_POSITION;
                float4 col : COLOR0;
            };

            PS_INPUT main(VS_INPUT input)
            {
                PS_INPUT output;
                float4 viewPos = mul(viewMatrix, float4(input.pos.xy, 0.f, 1.f));
                output.pos = mul(projMatrix, viewPos);
                output.col = input.col;
                return output;
            }

        )hlsl";

        ComPtr<ID3DBlob> errorBlob;
        ComPtr<ID3DBlob> vertexShaderBlob;
        if (0 > D3DCompile(
            vertexShader, strlen(vertexShader),
            NULL, NULL, NULL, "main", "vs_4_0", 0, 0,
            vertexShaderBlob.addressOf(), errorBlob.addressOf()))
        {
            const char* errString = static_cast<const char*>(errorBlob->GetBufferPointer());
            throw core::RuntimeError(errString);
        }
        errorBlob.reset();

        device_->CreateVertexShader(
            vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(),
            NULL, vertexShader_.addressOf());

        // Create the input layout
        D3D11_INPUT_ELEMENT_DESC layout[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32_FLOAT,    0, (UINT)offsetof(XYRGBVertex, x), D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "COLOR",    0, DXGI_FORMAT_R32G32B32_FLOAT, 0, (UINT)offsetof(XYRGBVertex, r), D3D11_INPUT_PER_VERTEX_DATA, 0 },
        };
        device_->CreateInputLayout(layout, 2, vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(), inputLayout_.addressOf());
        vertexShaderBlob.reset();

        // Create the constant buffer
        {
            D3D11_BUFFER_DESC desc = {};
            desc.ByteWidth = sizeof(PaintVertexShaderConstantBuffer);
            desc.Usage = D3D11_USAGE_DYNAMIC;
            desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            desc.MiscFlags = 0;
            device_->CreateBuffer(&desc, NULL, vertexConstantBuffer_.addressOf());
        }
    }

    // Create the paint pixel shader
    {
        static const char* pixelShader = R"hlsl(
            struct PS_INPUT
            {
                float4 pos : SV_POSITION;
                float4 col : COLOR0;
            };

            float4 main(PS_INPUT input) : SV_Target
            {
                return input.col;
            }

        )hlsl";

        ComPtr<ID3DBlob> errorBlob;
        ComPtr<ID3DBlob> pixelShaderBlob;
        if (0 > D3DCompile(
            pixelShader, strlen(pixelShader),
            NULL, NULL, NULL, "main", "ps_4_0", 0, 0,
            pixelShaderBlob.addressOf(), errorBlob.addressOf()))
        {
            const char* errString = static_cast<const char*>(errorBlob->GetBufferPointer());
            throw core::RuntimeError(errString);
        }
        errorBlob.reset();

        device_->CreatePixelShader(
            pixelShaderBlob->GetBufferPointer(), pixelShaderBlob->GetBufferSize(),
            NULL, pixelShader_.addressOf());

    }

    // Create the blending setup
    {
        D3D11_BLEND_DESC desc = {};
        desc.AlphaToCoverageEnable = false;
        desc.RenderTarget[0].BlendEnable = true;
        desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
        desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
        desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
        desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
        desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
        desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
        desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
        device_->CreateBlendState(&desc, blendState_.addressOf());
    }

    // Create the rasterizer state
    {
        D3D11_RASTERIZER_DESC desc = {};
        desc.FillMode = D3D11_FILL_SOLID;
        desc.CullMode = D3D11_CULL_NONE;
        desc.ScissorEnable = false;
        desc.DepthClipEnable = false;
        device_->CreateRasterizerState(&desc, rasterizerState_.addressOf());
    }

    // Create depth-stencil State
    {
        D3D11_DEPTH_STENCIL_DESC desc = {};
        desc.DepthEnable = false;
        desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        desc.DepthFunc = D3D11_COMPARISON_ALWAYS;
        desc.StencilEnable = false;
        desc.FrontFace.StencilFailOp = desc.FrontFace.StencilDepthFailOp = desc.FrontFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
        desc.FrontFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
        desc.BackFace = desc.FrontFace;
        device_->CreateDepthStencilState(&desc, depthStencilState_.addressOf());
    }
}

void D3d11Engine::onDestroyed()
{
    Engine::onDestroyed();
}

/* static */
D3d11EnginePtr D3d11Engine::create()
{
    return D3d11EnginePtr(new D3d11Engine());
}

// USER THREAD pimpl functions

SwapChain* D3d11Engine::createSwapChain_(const SwapChainDesc& desc)
{
    if (!device_) {
        throw core::LogicError("device_ is null.");
    }

    if (desc.windowNativeHandleType() != WindowNativeHandleType::Win32) {
        return nullptr;
    }

    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = desc.bufferCount();
    sd.BufferDesc.Width = desc.width();
    sd.BufferDesc.Height = desc.height();
    switch(desc.format()) {
    case SwapChainTargetFormat::R8G8B8A8_UNORM:
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM; break;
    case SwapChainTargetFormat::B8G8R8A8_SRGB:
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB; break;
    default:
        throw core::LogicError("D3d11SwapChain: unsupported format");
        break;
    }
    sd.BufferDesc.RefreshRate.Numerator = 0;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    // do we need DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT ?
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = static_cast<HWND>(desc.windowNativeHandle());
    sd.SampleDesc.Count = desc.sampleCount();
    sd.SampleDesc.Quality = 0;
    sd.Windowed = true;
    //sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    sd.SwapEffect = DXGI_SWAP_EFFECT_SEQUENTIAL;

    ComPtr<IDXGISwapChain> swapChain;
    if (factory_->CreateSwapChain(device_.get(), &sd, swapChain.addressOf()) < 0) {
        throw core::LogicError("D3d11Engine: could not create swap chain");
    }

    ComPtr<ID3D11Texture2D> backBuffer;
    swapChain->GetBuffer(0, IID_PPV_ARGS(backBuffer.addressOf()));
    ComPtr<ID3D11RenderTargetView> backBufferView;
    device_->CreateRenderTargetView(backBuffer.get(), NULL, backBufferView.addressOf());
    D3d11ImageViewPtr colorView(new D3d11ImageView(resourceList_, backBufferView.get()));
    D3d11FramebufferPtr newFramebuffer(
        new D3d11Framebuffer(
            resourceList_,
            colorView,
            D3d11ImageViewPtr()));

    D3d11SwapChain* d3dSwapChain = new D3d11SwapChain(resourceList_, desc, swapChain.get());
    d3dSwapChain->setDefaultFramebuffer(newFramebuffer);

    return d3dSwapChain;
}

void D3d11Engine::resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    d3dSwapChain->clearDefaultFramebuffer();

    HRESULT hres = d3dSwapChain->dxgiSwapChain()->ResizeBuffers(
        0, width, height, DXGI_FORMAT_UNKNOWN, 0);
    if (hres < 0) {
        throw core::LogicError("D3d11Engine: could not resize swap chain buffers");
    }

    ComPtr<ID3D11Texture2D> backBuffer;
    d3dSwapChain->dxgiSwapChain()->GetBuffer(0, IID_PPV_ARGS(backBuffer.addressOf()));
    ComPtr<ID3D11RenderTargetView> backBufferView;
    device_->CreateRenderTargetView(backBuffer.get(), NULL, backBufferView.addressOf());
    D3d11FramebufferPtr newFramebuffer(
        new D3d11Framebuffer(
            resourceList_,
            D3d11ImageViewPtr(new D3d11ImageView(resourceList_, backBufferView.get())),
            D3d11ImageViewPtr()));
    d3dSwapChain->setDefaultFramebuffer(newFramebuffer);
}

Buffer* D3d11Engine::createBuffer_(Usage usage, BindFlags bindFlags, CpuAccessFlags cpuAccessFlags)
{
    return new D3d11Buffer(resourceList_, usage, bindFlags, cpuAccessFlags);
}

// RENDER THREAD functions

void D3d11Engine::bindSwapChain_(SwapChain* swapChain)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    bindFramebuffer_(d3dSwapChain->d3dDefaultFrameBuffer());
    const float blend_factor[4] = { 0.f, 0.f, 0.f, 0.f };
    deviceCtx_->OMSetBlendState(blendState_.get(), blend_factor, 0xffffffff);
    deviceCtx_->OMSetDepthStencilState(depthStencilState_.get(), 0);
    deviceCtx_->RSSetState(rasterizerState_.get());
}

UInt64 D3d11Engine::present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags /*flags*/)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    d3dSwapChain->dxgiSwapChain()->Present(syncInterval, 0);
    return std::chrono::nanoseconds(std::chrono::steady_clock::now() - startTime_).count();
}

void D3d11Engine::bindFramebuffer_(Framebuffer* framebuffer)
{
    if (!framebuffer) {
        deviceCtx_->OMSetRenderTargets(0, NULL, NULL);
        return;
    }
    D3d11Framebuffer* d3dFramebuffer = static_cast<D3d11Framebuffer*>(framebuffer);
    ID3D11RenderTargetView* rtvArray[1] = { d3dFramebuffer->colorView() };
    deviceCtx_->OMSetRenderTargets(1, rtvArray, d3dFramebuffer->depthStencilView());
}

void D3d11Engine::setViewport_(Int x, Int y, Int width, Int height)
{
    D3D11_VIEWPORT vp = {};
    vp.TopLeftX = static_cast<float>(x);
    vp.TopLeftY = static_cast<float>(y);
    vp.Width = static_cast<float>(width);
    vp.Height = static_cast<float>(height);
    vp.MinDepth = 0.0f;
    vp.MaxDepth = 1.0f;
    deviceCtx_->RSSetViewports(1, &vp);
}

void D3d11Engine::clear_(const core::Color& color)
{
    ComPtr<ID3D11RenderTargetView> rtv;
    deviceCtx_->OMGetRenderTargets(1, rtv.addressOf(), NULL);
    std::array<float, 4> c = {
        static_cast<float>(color.r()),
        static_cast<float>(color.g()),
        static_cast<float>(color.b()),
        static_cast<float>(color.a())};
    deviceCtx_->ClearRenderTargetView(rtv.get(), c.data());
}

void D3d11Engine::setProjectionMatrix_(const geometry::Mat4f& m)
{
    paintVertexShaderConstantBuffer_.projMatrix = {
        m(0, 0), m(1, 0), m(2, 0), m(3, 0),
        m(0, 1), m(1, 1), m(2, 1), m(3, 1),
        m(0, 2), m(1, 2), m(2, 2), m(3, 2),
        m(0, 3), m(1, 3), m(2, 3), m(3, 3)
    };
    writeBufferReserved_(
        vertexConstantBuffer_.get(), &paintVertexShaderConstantBuffer_,
        sizeof(PaintVertexShaderConstantBuffer));
}

void D3d11Engine::setViewMatrix_(const geometry::Mat4f& m)
{
    paintVertexShaderConstantBuffer_.viewMatrix = {
        m(0, 0), m(1, 0), m(2, 0), m(3, 0),
        m(0, 1), m(1, 1), m(2, 1), m(3, 1),
        m(0, 2), m(1, 2), m(2, 2), m(3, 2),
        m(0, 3), m(1, 3), m(2, 3), m(3, 3)
    };
    writeBufferReserved_(
        vertexConstantBuffer_.get(), &paintVertexShaderConstantBuffer_,
        sizeof(PaintVertexShaderConstantBuffer));
}

void D3d11Engine::bindPaintShader_()
{
    deviceCtx_->IASetInputLayout(inputLayout_.get());
    deviceCtx_->VSSetShader(vertexShader_.get(), NULL, 0);
    deviceCtx_->VSSetConstantBuffers(0, 1, &vertexConstantBuffer_);
    deviceCtx_->PSSetShader(pixelShader_.get(), NULL, 0);
    deviceCtx_->GSSetShader(NULL, NULL, 0);
    deviceCtx_->HSSetShader(NULL, NULL, 0);
    deviceCtx_->DSSetShader(NULL, NULL, 0);
    deviceCtx_->CSSetShader(NULL, NULL, 0);
}

void D3d11Engine::releasePaintShader_()
{
    deviceCtx_->VSSetShader(NULL, NULL, 0);
    deviceCtx_->VSSetConstantBuffers(0, 0, NULL);
    deviceCtx_->PSSetShader(NULL, NULL, 0);
}

void D3d11Engine::initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes)
{
    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
    if (initialLengthInBytes) {
        loadBuffer_(d3dBuffer->objectAddress(), d3dBuffer->descAddress(), data, initialLengthInBytes);
    }
    d3dBuffer->lengthInBytes_ = initialLengthInBytes;
}

void D3d11Engine::updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes)
{
    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
    loadBuffer_(d3dBuffer->objectAddress(), d3dBuffer->descAddress(), data, lengthInBytes);
    d3dBuffer->lengthInBytes_ = lengthInBytes;
}

void D3d11Engine::setupVertexBufferForPaintShader_(Buffer* /*buffer*/)
{
    // no-op
}

void D3d11Engine::drawPrimitives_(Buffer* buffer, PrimitiveType type)
{
    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
    ID3D11Buffer* object = d3dBuffer->object();
    if (!object) return;

    D3D_PRIMITIVE_TOPOLOGY d3dTopology = {};
    switch (type) {
    case PrimitiveType::Point:
        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_POINTLIST; break;
    case PrimitiveType::LineList:
        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_LINELIST; break;
    case PrimitiveType::LineStrip:
        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP; break;
    case PrimitiveType::TriangleList:
        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST; break;
    case PrimitiveType::TriangleStrip:
        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP; break;
    default:
        throw core::LogicError("D3d11Buffer: unsupported primitive type");
    }

    Int numVertices = d3dBuffer->lengthInBytes_ / sizeof(XYRGBVertex);
    unsigned int stride = sizeof(XYRGBVertex);
    unsigned int offset = 0;
    deviceCtx_->IASetVertexBuffers(0, 1, &object, &stride, &offset);
    deviceCtx_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    deviceCtx_->Draw(core::int_cast<UINT>(numVertices), 0);
}

bool D3d11Engine::loadBuffer_(ID3D11Buffer** bufferPtr, D3D11_BUFFER_DESC* desc, const void* data, Int dataSize)
{
    if (dataSize == 0) {
        return false;
    }
    if ((dataSize > desc->ByteWidth) || (dataSize * 4 < desc->ByteWidth) || !*bufferPtr) {
        desc->ByteWidth = core::int_cast<UINT>(dataSize);
        D3D11_SUBRESOURCE_DATA srData = {};
        srData.pSysMem = data;
        if (device_->CreateBuffer(desc, (data ? &srData : NULL), bufferPtr) < 0) {
            desc->ByteWidth = 0;
            return false;
        }
        return true;
    }
    return writeBufferReserved_(*bufferPtr, data, dataSize);
}

bool D3d11Engine::writeBufferReserved_(ID3D11Buffer* buffer, const void* data, Int dataSize)
{
    D3D11_MAPPED_SUBRESOURCE mappedResource = {};
    if (deviceCtx_->Map(buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource) < 0) {
        return false;
    }
    memcpy(static_cast<char*>(mappedResource.pData), data, dataSize);
    deviceCtx_->Unmap(buffer, 0);
    return true;
}

} // namespace vgc::graphics

#endif // VGC_CORE_COMPILER_MSVC
