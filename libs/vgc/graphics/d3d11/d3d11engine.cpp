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

#include <algorithm>
#include <array>
#include <chrono>

#include <d3d11.h>
#include <d3dcompiler.h>
#pragma comment(lib, "d3dcompiler")
#pragma comment(lib, "d3d11")

#include <vgc/core/exceptions.h>

namespace vgc::graphics {

namespace {

// core::resourcePath("graphics/d3d11/" + name);

struct XYRGBVertex {
    float x, y, r, g, b;
};

} // namespace

class D3d11Buffer : public Buffer {
protected:
    friend D3d11Engine;
    using Buffer::Buffer;

public:
    ID3D11Buffer* object() const
    {
        return object_.get();
    }

    ID3D11Buffer** objectAddress()
    {
        return object_.addressOf();
    }

    const D3D11_BUFFER_DESC& desc() const
    {
        return desc_;
    }

    D3D11_BUFFER_DESC* descAddress()
    {
        return &desc_;
    }

protected:
    void release_(Engine* engine) override
    {
        Buffer::release_(engine);
        object_.reset();
    }

private:
    ComPtr<ID3D11Buffer> object_;
    D3D11_BUFFER_DESC desc_ = {};
};
using D3d11BufferPtr = ResourcePtr<D3d11Buffer>;

class D3d11Image : public Image {
protected:
    friend D3d11Engine;
    using Image::Image;

public:
    ID3D11Resource* object() const
    {
        return object_.get();
    }

    DXGI_FORMAT dxgiFormat() const
    {
        return dxgiFormat_;
    }

protected:
    void release_(Engine* engine) override
    {
        Image::release_(engine);
        object_.reset();
    }

private:
    ComPtr<ID3D11Resource> object_;
    DXGI_FORMAT dxgiFormat_;
};
using D3d11ImagePtr = ResourcePtr<D3d11Image>;

class D3d11ImageView : public ImageView {
protected:
    friend D3d11Engine;
    using ImageView::ImageView;

public:
    ID3D11ShaderResourceView* srvObject() const
    {
        return srv_.get();
    }

    ID3D11RenderTargetView* rtvObject() const
    {
        return rtv_.get();
    }

    ID3D11DepthStencilView* dsvObject() const
    {
        return dsv_.get();
    }

    DXGI_FORMAT dxgiFormat() const
    {
        return dxgiFormat_;
    }

    // resizing a swap chain requires releasing all views to its back buffers.
    void forceRelease()
    {
        srv_.reset();
        rtv_.reset();
        dsv_.reset();
    }

    ID3D11Resource* d3dViewedResource() const
    {
        Buffer* buffer = viewedBuffer().get();
        if (buffer) {
            D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
            return d3dBuffer->object();
        }
        else {
            D3d11Image* d3dImage = static_cast<D3d11Image*>(viewedImage().get());
            return d3dImage->object();
        }
    }

protected:
    void release_(Engine* engine) override
    {
        ImageView::release_(engine);
        forceRelease();
    }

private:
    ComPtr<ID3D11ShaderResourceView> srv_;
    ComPtr<ID3D11RenderTargetView> rtv_;
    ComPtr<ID3D11DepthStencilView> dsv_;
    DXGI_FORMAT dxgiFormat_;
};
using D3d11ImageViewPtr = ResourcePtr<D3d11ImageView>;

class D3d11SamplerState : public SamplerState {
protected:
    friend D3d11Engine;
    using SamplerState::SamplerState;

public:
    ID3D11SamplerState* object() const {
        return object_.get();
    }

protected:
    void release_(Engine* engine) override
    {
        SamplerState::release_(engine);
        object_.reset();
    }

private:
    ComPtr<ID3D11SamplerState> object_;
};
using D3d11SamplerStatePtr = ResourcePtr<D3d11SamplerState>;

class D3d11GeometryView : public GeometryView {
protected:
    friend D3d11Engine;
    using GeometryView::GeometryView;

public:
    D3D_PRIMITIVE_TOPOLOGY topology() const {
        return topology_;
    }

protected:
    void release_(Engine* engine) override
    {
        GeometryView::release_(engine);
    }

private:
    D3D_PRIMITIVE_TOPOLOGY topology_;
};
using D3d11GeometryViewPtr = ResourcePtr<D3d11GeometryView>;

class D3d11Program : public Program {
protected:
    friend D3d11Engine;
    using Program::Program;

public:
    // ...

protected:
    void release_(Engine* engine) override
    {
        Program::release_(engine);
        vertexShader_.reset();
        geometryShader_.reset();
        pixelShader_.reset();
        for (auto& x : builtinLayouts_) {
            x.reset();
        }
    }

private:
    ComPtr<ID3D11VertexShader> vertexShader_;
    ComPtr<ID3D11GeometryShader> geometryShader_;
    ComPtr<ID3D11PixelShader> pixelShader_;
    std::array<ComPtr<ID3D11InputLayout>, core::toUnderlying(BuiltinGeometryLayout::Max_) - 1> builtinLayouts_;
};
using D3d11ProgramPtr = ResourcePtr<D3d11Program>;

class D3d11BlendState : public BlendState {
protected:
    friend D3d11Engine;
    using BlendState::BlendState;

public:
    ID3D11BlendState* object() const
    {
        return object_.get();
    }

protected:
    void release_(Engine* engine) override
    {
        BlendState::release_(engine);
        object_.reset();
    }

private:
    ComPtr<ID3D11BlendState> object_;
};
using D3d11BlendStatePtr = ResourcePtr<D3d11BlendState>;

class D3d11RasterizerState : public RasterizerState {
protected:
    friend D3d11Engine;
    using RasterizerState::RasterizerState;

public:
    ID3D11RasterizerState* object() const
    {
        return object_.get();
    }

protected:
    void release_(Engine* engine) override
    {
        RasterizerState::release_(engine);
        object_.reset();
    }

private:
    ComPtr<ID3D11RasterizerState> object_;
};
using D3d11RasterizerStatePtr = ResourcePtr<D3d11RasterizerState>;

// no equivalent in D3D11, see OMSetRenderTargets
class D3d11Framebuffer : public Framebuffer {
public:
    D3d11Framebuffer(
        ResourceList* gcList,
        const D3d11ImageViewPtr& colorView,
        const D3d11ImageViewPtr& depthStencilView,
        bool isDefault)
        : Framebuffer(gcList)
        , colorView_(colorView)
        , depthStencilView_(depthStencilView)
        , isDefault_(isDefault)
    {
    }

    bool isDefault() const
    {
        return isDefault_;
    }

    ID3D11RenderTargetView* rtvObject() const
    {
        return colorView_ ?
            static_cast<ID3D11RenderTargetView*>(colorView_->rtvObject()) : nullptr;
    }

    ID3D11DepthStencilView* dsvObject() const
    {
        return depthStencilView_ ?
            static_cast<ID3D11DepthStencilView*>(depthStencilView_->dsvObject()) : nullptr;
    }

    // resizing a swap chain requires releasing all views to its back buffers.
    void forceReleaseColorView()
    {
        colorView_->forceRelease();
    }

protected:
    void clearSubResources_() override
    {
        colorView_.reset();
        depthStencilView_.reset();
    }

    void release_(Engine* engine) override
    {
        Framebuffer::release_(engine);
    }

private:
    D3d11ImageViewPtr colorView_;
    D3d11ImageViewPtr depthStencilView_;

    bool isDefault_ = false;
};
using D3d11FramebufferPtr = ResourcePtr<D3d11Framebuffer>;

class D3d11SwapChain : public SwapChain {
public:
    D3d11SwapChain(ResourceList* gcList,
        const SwapChainCreateInfo& desc,
        IDXGISwapChain* dxgiSwapChain)
        : SwapChain(gcList, desc)
        , dxgiSwapChain_(dxgiSwapChain)
    {
    }

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
    void release_(Engine* engine) override
    {
        SwapChain::release_(engine);
    }

private:
    ComPtr<IDXGISwapChain> dxgiSwapChain_;
};

// ENUM CONVERSIONS

DXGI_FORMAT imageFormatToDxgiFormat(ImageFormat format)
{
    switch (format) {
        // Depth
    case ImageFormat::D_16_UNORM:               return DXGI_FORMAT_D16_UNORM;
    case ImageFormat::D_32_FLOAT:               return DXGI_FORMAT_D32_FLOAT;
        // Depth + Stencil
    case ImageFormat::DS_24_UNORM_8_UINT:       return DXGI_FORMAT_D24_UNORM_S8_UINT;
    case ImageFormat::DS_32_FLOAT_8_UINT_24_X:  return DXGI_FORMAT_D32_FLOAT_S8X24_UINT;
        // Red
    case ImageFormat::R_8_UNORM:                return DXGI_FORMAT_R8_UNORM;
    case ImageFormat::R_8_SNORM:                return DXGI_FORMAT_R8_SNORM;
    case ImageFormat::R_8_UINT:                 return DXGI_FORMAT_R8_UINT;
    case ImageFormat::R_8_SINT:                 return DXGI_FORMAT_R8_SINT;
    case ImageFormat::R_16_UNORM:               return DXGI_FORMAT_R16_UNORM;
    case ImageFormat::R_16_SNORM:               return DXGI_FORMAT_R16_SNORM;
    case ImageFormat::R_16_UINT:                return DXGI_FORMAT_R16_UINT;
    case ImageFormat::R_16_SINT:                return DXGI_FORMAT_R16_SINT;
    case ImageFormat::R_16_FLOAT:               return DXGI_FORMAT_R16_FLOAT;
    case ImageFormat::R_32_UINT:                return DXGI_FORMAT_R32_UINT;
    case ImageFormat::R_32_SINT:                return DXGI_FORMAT_R32_SINT;
    case ImageFormat::R_32_FLOAT:               return DXGI_FORMAT_R32_FLOAT;
        // RG
    case ImageFormat::RG_8_UNORM:               return DXGI_FORMAT_R8G8_UNORM;
    case ImageFormat::RG_8_SNORM:               return DXGI_FORMAT_R8G8_SNORM;
    case ImageFormat::RG_8_UINT:                return DXGI_FORMAT_R8G8_UINT;
    case ImageFormat::RG_8_SINT:                return DXGI_FORMAT_R8G8_SINT;
    case ImageFormat::RG_16_UNORM:              return DXGI_FORMAT_R16G16_UNORM;
    case ImageFormat::RG_16_SNORM:              return DXGI_FORMAT_R16G16_SNORM;
    case ImageFormat::RG_16_UINT:               return DXGI_FORMAT_R16G16_UINT;
    case ImageFormat::RG_16_SINT:               return DXGI_FORMAT_R16G16_SINT;
    case ImageFormat::RG_16_FLOAT:              return DXGI_FORMAT_R16G16_FLOAT;
    case ImageFormat::RG_32_UINT:               return DXGI_FORMAT_R32G32_UINT;
    case ImageFormat::RG_32_SINT:               return DXGI_FORMAT_R32G32_SINT;
    case ImageFormat::RG_32_FLOAT:              return DXGI_FORMAT_R32G32_FLOAT;
        // RGB
    case ImageFormat::RGB_11_11_10_FLOAT:       return DXGI_FORMAT_R11G11B10_FLOAT;
    case ImageFormat::RGB_32_UINT:              return DXGI_FORMAT_R32G32B32_UINT;
    case ImageFormat::RGB_32_SINT:              return DXGI_FORMAT_R32G32B32_SINT;
    case ImageFormat::RGB_32_FLOAT:             return DXGI_FORMAT_R32G32B32_FLOAT;
        // RGBA
    case ImageFormat::RGBA_8_UNORM:             return DXGI_FORMAT_R8G8B8A8_UNORM;
    case ImageFormat::RGBA_8_UNORM_SRGB:        return DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    case ImageFormat::RGBA_8_SNORM:             return DXGI_FORMAT_R8G8B8A8_SNORM;
    case ImageFormat::RGBA_8_UINT:              return DXGI_FORMAT_R8G8B8A8_UINT;
    case ImageFormat::RGBA_8_SINT:              return DXGI_FORMAT_R8G8B8A8_SINT;
    case ImageFormat::RGBA_10_10_10_2_UNORM:    return DXGI_FORMAT_R10G10B10A2_UNORM;
    case ImageFormat::RGBA_10_10_10_2_UINT:     return DXGI_FORMAT_R10G10B10A2_UINT;
    case ImageFormat::RGBA_16_UNORM:            return DXGI_FORMAT_R16G16B16A16_UNORM;
    case ImageFormat::RGBA_16_UINT:             return DXGI_FORMAT_R16G16B16A16_UINT;
    case ImageFormat::RGBA_16_SINT:             return DXGI_FORMAT_R16G16B16A16_SINT;
    case ImageFormat::RGBA_16_FLOAT:            return DXGI_FORMAT_R16G16B16A16_FLOAT;
    case ImageFormat::RGBA_32_UINT:             return DXGI_FORMAT_R32G32B32A32_UINT;
    case ImageFormat::RGBA_32_SINT:             return DXGI_FORMAT_R32G32B32A32_SINT;
    case ImageFormat::RGBA_32_FLOAT:            return DXGI_FORMAT_R32G32B32A32_FLOAT;
    default:
        break;
    }
    return DXGI_FORMAT_UNKNOWN;
}

D3D_PRIMITIVE_TOPOLOGY primitiveTypeToD3DPrimitiveTopology(PrimitiveType type)
{
    switch (type) {
    case PrimitiveType::Point:          return D3D11_PRIMITIVE_TOPOLOGY_POINTLIST;
    case PrimitiveType::LineList:       return D3D11_PRIMITIVE_TOPOLOGY_LINELIST;
    case PrimitiveType::LineStrip:      return D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP;
    case PrimitiveType::TriangleList:   return D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
    case PrimitiveType::TriangleStrip:  return D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP;
    default:
        break;
    }
    return D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;
}

D3D11_USAGE usageToD3DUsage(Usage usage)
{
    switch (usage) {
    case Usage::Default:
        return D3D11_USAGE_DEFAULT;
    case Usage::Immutable:
        return D3D11_USAGE_IMMUTABLE;
    case Usage::Dynamic:
        return D3D11_USAGE_DYNAMIC;
    case Usage::Staging:
        return D3D11_USAGE_STAGING;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unsupported usage");
}

UINT resourceMiscFlagsToD3DResourceMiscFlags(ResourceMiscFlags resourceMiscFlags)
{
    UINT x;
    if (!!(resourceMiscFlags & ResourceMiscFlags::GenerateMips)) {
        x |= D3D11_RESOURCE_MISC_GENERATE_MIPS;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::Shared)) {
        x |= D3D11_RESOURCE_MISC_SHARED;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::TextureCube)) {
        x |= D3D11_RESOURCE_MISC_TEXTURECUBE;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::DrawIndirectArgs)) {
        x |= D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::BufferRaw)) {
        x |= D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::BufferStructured)) {
        x |= D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::ResourceClamp)) {
        x |= D3D11_RESOURCE_MISC_RESOURCE_CLAMP;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::SharedKeyedMutex)) {
        x |= D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX;
    }
    return x;
}

D3D11_TEXTURE_ADDRESS_MODE imageWrapModeToD3DTextureAddressMode(ImageWrapMode mode)
{
    switch (mode) {
    case ImageWrapMode::ConstantColor:
        return D3D11_TEXTURE_ADDRESS_BORDER;
    case ImageWrapMode::Clamp:
        return D3D11_TEXTURE_ADDRESS_CLAMP;
    case ImageWrapMode::MirrorClamp:
        return D3D11_TEXTURE_ADDRESS_MIRROR_ONCE;
    case ImageWrapMode::Repeat:
        return D3D11_TEXTURE_ADDRESS_WRAP;
    case ImageWrapMode::MirrorRepeat:
        return D3D11_TEXTURE_ADDRESS_MIRROR;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unknown image wrap mode");
}

D3D11_COMPARISON_FUNC comparisonFunctionToD3DComparisonFunc(ComparisonFunction func)
{
    switch (func) {
    case ComparisonFunction::Disabled:
        return D3D11_COMPARISON_FUNC{};
    case ComparisonFunction::Always:
        return D3D11_COMPARISON_ALWAYS;
    case ComparisonFunction::Never:
        return D3D11_COMPARISON_NEVER;
    case ComparisonFunction::Equal:
        return D3D11_COMPARISON_EQUAL;
    case ComparisonFunction::NotEqual:
        return D3D11_COMPARISON_NOT_EQUAL;
    case ComparisonFunction::Less:
        return D3D11_COMPARISON_LESS;
    case ComparisonFunction::LessEqual:
        return D3D11_COMPARISON_LESS_EQUAL;
    case ComparisonFunction::Greater:
        return D3D11_COMPARISON_GREATER;
    case ComparisonFunction::GreaterEqual:
        return D3D11_COMPARISON_GREATER_EQUAL;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unknown comparison func");
}

D3D11_BLEND blendFactorToD3DBlend(BlendFactor factor)
{
    switch (factor) {
    case BlendFactor::One:
        return D3D11_BLEND_ONE;
    case BlendFactor::Zero:
        return D3D11_BLEND_ZERO;
    case BlendFactor::SourceColor:
        return D3D11_BLEND_SRC_COLOR;
    case BlendFactor::OneMinusSourceColor:
        return D3D11_BLEND_INV_SRC_COLOR;
    case BlendFactor::SourceAlpha:
        return D3D11_BLEND_SRC_ALPHA;
    case BlendFactor::OneMinusSourceAlpha:
        return D3D11_BLEND_INV_SRC_ALPHA;
    case BlendFactor::TargetColor:
        return D3D11_BLEND_DEST_COLOR;
    case BlendFactor::OneMinusTargetColor:
        return D3D11_BLEND_INV_DEST_COLOR;
    case BlendFactor::TargetAlpha:
        return D3D11_BLEND_DEST_ALPHA;
    case BlendFactor::OneMinusTargetAlpha:
        return D3D11_BLEND_INV_DEST_ALPHA;
    case BlendFactor::SourceAlphaSaturated:
        return D3D11_BLEND_SRC_ALPHA_SAT;
    case BlendFactor::Constant:
        return D3D11_BLEND_BLEND_FACTOR;
    case BlendFactor::OneMinusConstant:
        return D3D11_BLEND_INV_BLEND_FACTOR;
    case BlendFactor::SecondSourceColor:
        return D3D11_BLEND_SRC1_COLOR;
    case BlendFactor::OneMinusSecondSourceColor:
        return D3D11_BLEND_INV_SRC1_COLOR;
    case BlendFactor::SecondSourceAlpha:
        return D3D11_BLEND_SRC1_ALPHA;
    case BlendFactor::OneMinusSecondSourceAlpha:
        return D3D11_BLEND_INV_SRC1_ALPHA;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unknown blend factor");
}

D3D11_BLEND_OP blendOpToD3DBlendOp(BlendOp op)
{
    switch (op) {
    case BlendOp::Add:
        return D3D11_BLEND_OP_ADD;
    case BlendOp::SourceMinusTarget:
        return D3D11_BLEND_OP_SUBTRACT;
    case BlendOp::TargetMinusSource:
        return D3D11_BLEND_OP_REV_SUBTRACT;
    case BlendOp::Min:
        return D3D11_BLEND_OP_MIN;
    case BlendOp::Max:
        return D3D11_BLEND_OP_MAX;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unknown blend op");
}

D3D11_FILL_MODE fillModeToD3DFilleMode(FillMode mode)
{
    switch (mode) {
    case FillMode::Solid:
        return D3D11_FILL_SOLID;
    case FillMode::Wireframe:
        return D3D11_FILL_WIREFRAME;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unknown fill mode");
}

D3D11_CULL_MODE cullModeToD3DCulleMode(CullMode mode)
{
    switch (mode) {
    case CullMode::None:
        return D3D11_CULL_NONE;
    case CullMode::Front:
        return D3D11_CULL_FRONT;
    case CullMode::Back:
        return D3D11_CULL_BACK;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unknown cull mode");
}

// ENGINE FUNCTIONS

D3d11Engine::D3d11Engine()
{
    // XXX add success checks (S_OK)

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

void D3d11Engine::createBuiltinShaders_()
{
    D3d11ProgramPtr simpleProgram(new D3d11Program(gcResourceList_));
    simpleProgram_ = simpleProgram;

    // Create the simple shader
    {
        static const char* vertexShaderSrc = R"hlsl(

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
            vertexShaderSrc, strlen(vertexShaderSrc),
            NULL, NULL, NULL, "main", "vs_4_0", 0, 0,
            vertexShaderBlob.addressOf(), errorBlob.addressOf()))
        {
            const char* errString = static_cast<const char*>(errorBlob->GetBufferPointer());
            throw core::RuntimeError(errString);
        }
        errorBlob.reset();

        ComPtr<ID3D11VertexShader> vertexShader;
        device_->CreateVertexShader(
            vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(),
            NULL, vertexShader.addressOf());
        simpleProgram->vertexShader_ = vertexShader;

        // Create the input layout
        ComPtr<ID3D11InputLayout> inputLayout;
        D3D11_INPUT_ELEMENT_DESC layout[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32_FLOAT,    0, (UINT)offsetof(XYRGBVertex, x), D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "COLOR",    0, DXGI_FORMAT_R32G32B32_FLOAT, 0, (UINT)offsetof(XYRGBVertex, r), D3D11_INPUT_PER_VERTEX_DATA, 0 },
        };
        device_->CreateInputLayout(layout, 2, vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(), inputLayout.addressOf());
        simpleProgram->builtinLayouts_[core::toUnderlying(BuiltinGeometryLayout::XYRGB)] = inputLayout;

        // Create the constant buffer
        /*{
            D3D11_BUFFER_DESC desc = {};
            desc.ByteWidth = sizeof(PaintVertexShaderConstantBuffer);
            desc.Usage = D3D11_USAGE_DYNAMIC;
            desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
            desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            desc.MiscFlags = 0;
            device_->CreateBuffer(&desc, NULL, vertexConstantBuffer_.addressOf());
        }*/
    }

    // Create the paint pixel shader
    {
        static const char* pixelShaderSrc = R"hlsl(
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
            pixelShaderSrc, strlen(pixelShaderSrc),
            NULL, NULL, NULL, "main", "ps_4_0", 0, 0,
            pixelShaderBlob.addressOf(), errorBlob.addressOf()))
        {
            const char* errString = static_cast<const char*>(errorBlob->GetBufferPointer());
            throw core::RuntimeError(errString);
        }
        errorBlob.reset();

        ComPtr<ID3D11PixelShader> pixelShader;
        device_->CreatePixelShader(
            pixelShaderBlob->GetBufferPointer(), pixelShaderBlob->GetBufferSize(),
            NULL, pixelShader.addressOf());
        simpleProgram->pixelShader_ = pixelShader;
    }

    // Create the blending setup
    //{
    //    D3D11_BLEND_DESC desc = {};
    //    desc.AlphaToCoverageEnable = false;
    //    desc.RenderTarget[0].BlendEnable = true;
    //    desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
    //    desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    //    desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    //    desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
    //    desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
    //    desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    //    desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
    //    device_->CreateBlendState(&desc, blendState_.addressOf());
    //}

    //// Create the rasterizer state
    //{
    //    D3D11_RASTERIZER_DESC desc = {};
    //    desc.FillMode = D3D11_FILL_SOLID;
    //    desc.CullMode = D3D11_CULL_NONE;
    //    desc.ScissorEnable = false;
    //    desc.DepthClipEnable = false;
    //    device_->CreateRasterizerState(&desc, rasterizerState_.addressOf());
    //}

    //// Create depth-stencil State
    //{
    //    D3D11_DEPTH_STENCIL_DESC desc = {};
    //    desc.DepthEnable = false;
    //    desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    //    desc.DepthFunc = D3D11_COMPARISON_ALWAYS;
    //    desc.StencilEnable = false;
    //    desc.FrontFace.StencilFailOp = desc.FrontFace.StencilDepthFailOp = desc.FrontFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
    //    desc.FrontFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
    //    desc.BackFace = desc.FrontFace;
    //    device_->CreateDepthStencilState(&desc, depthStencilState_.addressOf());
    //}
}

SwapChainPtr D3d11Engine::createSwapChain_(const SwapChainCreateInfo& createInfo)
{
    if (!device_) {
        throw core::LogicError("device_ is null.");
    }

    if (createInfo.windowNativeHandleType() != WindowNativeHandleType::Win32) {
        return nullptr;
    }

    ImageFormat colorViewFormat = {};

    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = createInfo.bufferCount();
    sd.BufferDesc.Width = createInfo.width();
    sd.BufferDesc.Height = createInfo.height();
    switch(createInfo.format()) {
    case SwapChainTargetFormat::RGBA_8_UNORM: {
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        colorViewFormat = ImageFormat::RGBA_8_UNORM;
        break;
    }
    case SwapChainTargetFormat::RGBA_8_UNORM_SRGB: {
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
        colorViewFormat = ImageFormat::RGBA_8_UNORM_SRGB;
        break;
    }
    default:
        throw core::LogicError("D3d11SwapChain: unsupported format");
        break;
    }
    sd.BufferDesc.RefreshRate.Numerator = 0;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    // do we need DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT ?
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = static_cast<HWND>(createInfo.windowNativeHandle());
    sd.SampleDesc.Count = createInfo.sampleCount();
    sd.SampleDesc.Quality = 0;
    sd.Windowed = true;
    //sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    sd.SwapEffect = DXGI_SWAP_EFFECT_SEQUENTIAL;

    ComPtr<IDXGISwapChain> dxgiSwapChain;
    if (factory_->CreateSwapChain(device_.get(), &sd, dxgiSwapChain.addressOf()) < 0) {
        throw core::LogicError("D3d11Engine: could not create swap chain");
    }

    ComPtr<ID3D11Texture2D> backBuffer;
    dxgiSwapChain->GetBuffer(0, IID_PPV_ARGS(backBuffer.addressOf()));

    D3D11_TEXTURE2D_DESC backBufferDesc = {};
    backBuffer->GetDesc(&backBufferDesc);

    ImageCreateInfo imageCreateInfo = {};
    imageCreateInfo.setRank(ImageRank::_2D);
    // XXX fill it using backBufferDesc

    D3d11ImagePtr backBufferImage(new D3d11Image(gcResourceList_, imageCreateInfo));
    backBufferImage->object_ = backBuffer;

    ImageViewCreateInfo viewCreateInfo = {};
    viewCreateInfo.setBindFlags(ImageBindFlags::RenderTarget);
    D3d11ImageViewPtr colorView(new D3d11ImageView(gcResourceList_, viewCreateInfo, backBufferImage, colorViewFormat, 0));

    ComPtr<ID3D11RenderTargetView> backBufferView;
    device_->CreateRenderTargetView(backBuffer.get(), NULL, backBufferView.addressOf());
    colorView->rtv_ = backBufferView.get();

    D3d11FramebufferPtr newFramebuffer(
        new D3d11Framebuffer(
            gcResourceList_,
            colorView,
            D3d11ImageViewPtr(),
            true));

    auto swapChain = std::make_unique<D3d11SwapChain>(gcResourceList_, createInfo, dxgiSwapChain.get());
    swapChain->setDefaultFramebuffer(newFramebuffer);

    return SwapChainPtr(swapChain.release());
}

FramebufferPtr D3d11Engine::createFramebuffer_(const ImageViewPtr& colorImageView)
{
    auto framebuffer = std::make_unique<D3d11Framebuffer>(gcResourceList_, colorImageView, nullptr, false);
    return FramebufferPtr(framebuffer.release());
}

BufferPtr D3d11Engine::createBuffer_(const BufferCreateInfo& createInfo)
{
    auto buffer = std::make_unique<D3d11Buffer>(gcResourceList_, createInfo);
    D3D11_BUFFER_DESC& desc = buffer->desc_;

    desc.Usage = usageToD3DUsage(createInfo.usage());

    const BindFlags bindFlags = createInfo.bindFlags();
    if (!!(bindFlags & BindFlags::ConstantBuffer)) {
        desc.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER;
        if (bindFlags != BindFlags::ConstantBuffer) {
            throw core::LogicError("D3d11Buffer: BindFlags::UniformBuffer cannot be combined with any other bind flag");
        }
    }
    else {
        if (!!(bindFlags & BindFlags::VertexBuffer)) {
            desc.BindFlags |= D3D11_BIND_VERTEX_BUFFER;
        }
        if (!!(bindFlags & BindFlags::IndexBuffer)) {
            desc.BindFlags |= D3D11_BIND_INDEX_BUFFER;
        }
        if (!!(bindFlags & BindFlags::ConstantBuffer)) {
            desc.BindFlags |= D3D11_BIND_CONSTANT_BUFFER;
        }
        if (!!(bindFlags & BindFlags::ShaderResource)) {
            desc.BindFlags |= D3D11_BIND_SHADER_RESOURCE;
        }
        if (!!(bindFlags & BindFlags::RenderTarget)) {
            desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
        }
        if (!!(bindFlags & BindFlags::DepthStencil)) {
            desc.BindFlags |= D3D11_BIND_DEPTH_STENCIL;
        }
        if (!!(bindFlags & BindFlags::UnorderedAccess)) {
            desc.BindFlags |= D3D11_BIND_UNORDERED_ACCESS;
        }
        if (!!(bindFlags & BindFlags::StreamOutput)) {
            desc.BindFlags |= D3D11_BIND_STREAM_OUTPUT;
        }
    }

    const ResourceMiscFlags resourceMiscFlags = createInfo.resourceMiscFlags();
    desc.MiscFlags = resourceMiscFlagsToD3DResourceMiscFlags(resourceMiscFlags);

    const CpuAccessFlags cpuAccessFlags = createInfo.cpuAccessFlags();
    if (!!(cpuAccessFlags & CpuAccessFlags::Write)) {
        desc.CPUAccessFlags |= D3D11_CPU_ACCESS_WRITE;
    }
    if (!!(cpuAccessFlags & CpuAccessFlags::Read)) {
        desc.CPUAccessFlags |= D3D11_CPU_ACCESS_READ;
    }

    return BufferPtr(buffer.release());
}

ImagePtr D3d11Engine::createImage_(const ImageCreateInfo& createInfo)
{
    DXGI_FORMAT dxgiFormat = imageFormatToDxgiFormat(createInfo.format());
    if (dxgiFormat == DXGI_FORMAT_UNKNOWN) {
        throw core::LogicError("D3d11: unknown image format");
    }

    auto image = std::make_unique<D3d11Image>(gcResourceList_, createInfo);
    image->dxgiFormat_ = dxgiFormat;
    return ImagePtr(image.release());
}

ImageViewPtr D3d11Engine::createImageView_(const ImageViewCreateInfo& createInfo, const ImagePtr& image)
{
    // XXX should check bind flags compatibility in abstract engine

    auto imageView = std::make_unique<D3d11ImageView>(gcResourceList_, createInfo, image, 0);
    imageView->dxgiFormat_ = static_cast<D3d11Image*>(image.get())->dxgiFormat();
    return ImageViewPtr(imageView.release());
}

ImageViewPtr D3d11Engine::createImageView_(const ImageViewCreateInfo& createInfo, const BufferPtr& buffer, ImageFormat format, UInt32 elementsCount)
{
    // XXX should check bind flags compatibility in abstract engine

    DXGI_FORMAT dxgiFormat = imageFormatToDxgiFormat(format);
    if (dxgiFormat == DXGI_FORMAT_UNKNOWN) {
        throw core::LogicError("D3d11: unknown image format");
    }

    auto view = std::make_unique<D3d11ImageView>(gcResourceList_, createInfo, buffer, format);
    view->dxgiFormat_ = dxgiFormat;
    return ImageViewPtr(view.release());
}

SamplerStatePtr D3d11Engine::createSamplerState_(const SamplerStateCreateInfo& createInfo)
{
    auto state = std::make_unique<D3d11SamplerState>(gcResourceList_, createInfo);
    return SamplerStatePtr(state.release());
}

GeometryViewPtr D3d11Engine::createGeometryView_(const GeometryViewCreateInfo& createInfo)
{
    D3D_PRIMITIVE_TOPOLOGY topology = primitiveTypeToD3DPrimitiveTopology(createInfo.primitiveType());
    if (topology == D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED) {
        throw core::LogicError("D3d11: unknown primitive type");
    }

    auto view = std::make_unique<D3d11GeometryView>(gcResourceList_, createInfo);
    view->topology_ = topology;
    return GeometryViewPtr(view.release());
}

BlendStatePtr D3d11Engine::createBlendState_(const BlendStateCreateInfo& createInfo)
{
    auto state = std::make_unique<D3d11BlendState>(gcResourceList_, createInfo);
    return BlendStatePtr(state.release());
}

RasterizerStatePtr D3d11Engine::createRasterizerState_(const RasterizerStateCreateInfo& createInfo)
{
    auto state = std::make_unique<D3d11RasterizerState>(gcResourceList_, createInfo);
    return RasterizerStatePtr(state.release());
}

// -- RENDER THREAD functions --

void D3d11Engine::initBuiltinShaders_()
{
    // no-op, everything was done on create
}

void D3d11Engine::initFramebuffer_(Framebuffer* framebuffer)
{
    // no-op
}

void D3d11Engine::initBuffer_(Buffer* buffer, const Span<const char>* dataSpan, Int initialLengthInBytes)
{
    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
    if (initialLengthInBytes) {
        loadBuffer_(d3dBuffer->objectAddress(),
                    d3dBuffer->descAddress(),
                    dataSpan ? dataSpan->data() : nullptr,
                    initialLengthInBytes);
    }
    d3dBuffer->lengthInBytes_ = initialLengthInBytes;
}

void D3d11Engine::initImage_(Image* image, const Span<const Span<const char>>* dataSpanSpan)
{
    D3d11Image* d3dImage = static_cast<D3d11Image*>(image);

    // XXX add size checks, see https://docs.microsoft.com/en-us/windows/win32/api/d3d11/nf-d3d11-id3d11device-createtexture2d
    core::Array<D3D11_SUBRESOURCE_DATA> initData;
    if (dataSpanSpan) {
        initData.resize(dataSpanSpan->size());
        for (Int i = 0; i < dataSpanSpan->size(); ++i) {
            initData[i].pSysMem = dataSpanSpan->data()[i].data();
        }
    }

    D3D11_USAGE d3dUsage = usageToD3DUsage(d3dImage->usage());

    UINT d3dBindFlags = 0;
    if (!!(d3dImage->bindFlags() & ImageBindFlags::ShaderResource)) {
        d3dBindFlags |= D3D11_BIND_SHADER_RESOURCE;
    }
    if (!!(d3dImage->bindFlags() & ImageBindFlags::RenderTarget)) {
        d3dBindFlags |= D3D11_BIND_RENDER_TARGET;
    }
    if (!!(d3dImage->bindFlags() & ImageBindFlags::DepthStencil)) {
        d3dBindFlags |= D3D11_BIND_DEPTH_STENCIL;
    }

    const CpuAccessFlags cpuAccessFlags = d3dImage->cpuAccessFlags();
    UINT d3dCPUAccessFlags = 0;
    if (!!(cpuAccessFlags & CpuAccessFlags::Write)) {
        d3dCPUAccessFlags |= D3D11_CPU_ACCESS_WRITE;
    }
    if (!!(cpuAccessFlags & CpuAccessFlags::Read)) {
        d3dCPUAccessFlags |= D3D11_CPU_ACCESS_READ;
    }

    UINT d3dMiscFlags = resourceMiscFlagsToD3DResourceMiscFlags(d3dImage->resourceMiscFlags());

    if (d3dImage->rank() == ImageRank::_1D) {
        D3D11_TEXTURE1D_DESC desc = {};
        desc.Width = d3dImage->width();
        desc.MipLevels = d3dImage->mipLevelCount();
        desc.ArraySize = d3dImage->layerCount();
        desc.Format = d3dImage->dxgiFormat();
        desc.Usage = d3dUsage;
        desc.BindFlags = d3dBindFlags;
        desc.CPUAccessFlags = d3dCPUAccessFlags;
        desc.MiscFlags = d3dMiscFlags;

        ComPtr<ID3D11Texture1D> texture;
        device_->CreateTexture1D(&desc, initData.size() ? initData.data() : nullptr, texture.addressOf());
        d3dImage->object_ = texture;
    }
    else {
        VGC_CORE_ASSERT(d3dImage->rank() == ImageRank::_2D);
        D3D11_TEXTURE2D_DESC desc = {};
        desc.Width = d3dImage->width();
        desc.Height = d3dImage->height();
        desc.MipLevels = d3dImage->mipLevelCount();
        desc.ArraySize = std::max<UInt8>(1, d3dImage->layerCount());
        desc.Format = d3dImage->dxgiFormat();
        desc.SampleDesc.Count = d3dImage->sampleCount();
        desc.Usage = d3dUsage;
        desc.BindFlags = d3dBindFlags;
        desc.CPUAccessFlags = d3dCPUAccessFlags;
        desc.MiscFlags = d3dMiscFlags;

        ComPtr<ID3D11Texture2D> texture;
        device_->CreateTexture2D(&desc, initData.size() ? initData.data() : nullptr, texture.addressOf());
        d3dImage->object_ = texture;
    }
}

void D3d11Engine::initImageView_(ImageView* view)
{
    D3d11ImageView* d3dImageView = static_cast<D3d11ImageView*>(view);
    if (!!(d3dImageView->bindFlags() & ImageBindFlags::ShaderResource)) {
        D3D11_SHADER_RESOURCE_VIEW_DESC desc = {};
        desc.Format = d3dImageView->dxgiFormat();
        if (view->isBuffer()) {
            BufferPtr buffer = view->viewedBuffer();
            desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
            desc.Buffer.FirstElement = 0;
            desc.Buffer.NumElements = d3dImageView->bufferElementsCount();
        }
        else {
            ImagePtr image = view->viewedImage();
            switch (image->rank()) {
            case ImageRank::_1D: {
                if (image->layerCount() > 1) {
                    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE1DARRAY;
                    desc.Texture1DArray.FirstArraySlice = d3dImageView->layerStart();
                    desc.Texture1DArray.ArraySize = d3dImageView->layerCount();
                    desc.Texture1DArray.MostDetailedMip = d3dImageView->mipLevelStart();
                    desc.Texture1DArray.MipLevels = d3dImageView->mipLevelCount();
                }
                else {
                    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE1D;
                    desc.Texture1D.MostDetailedMip = d3dImageView->mipLevelStart();
                    desc.Texture1D.MipLevels = d3dImageView->mipLevelCount();
                }
                break;
            }
            case ImageRank::_2D: {
                if (image->layerCount() > 1) {
                    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2DARRAY;
                    desc.Texture2DArray.FirstArraySlice = d3dImageView->layerStart();
                    desc.Texture2DArray.ArraySize = d3dImageView->layerCount();
                    desc.Texture2DArray.MostDetailedMip = d3dImageView->mipLevelStart();
                    desc.Texture2DArray.MipLevels = d3dImageView->mipLevelCount();
                }
                else {
                    desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
                    desc.Texture2D.MostDetailedMip = d3dImageView->mipLevelStart();
                    desc.Texture2D.MipLevels = d3dImageView->mipLevelCount();
                }
                break;
            }
            default:
                throw core::LogicError("D3d11: unknown image rank");
                break;
            }
        }
        device_->CreateShaderResourceView(d3dImageView->d3dViewedResource(), &desc, d3dImageView->srv_.addressOf());
    }
    if (!!(d3dImageView->bindFlags() & ImageBindFlags::RenderTarget)) {
        D3D11_RENDER_TARGET_VIEW_DESC desc = {};
        desc.Format = d3dImageView->dxgiFormat();
        if (view->isBuffer()) {
            BufferPtr buffer = view->viewedBuffer();
            desc.ViewDimension = D3D11_RTV_DIMENSION_BUFFER;
            desc.Buffer.FirstElement = 0;
            desc.Buffer.NumElements = d3dImageView->bufferElementsCount();
        }
        else {
            ImagePtr image = view->viewedImage();
            switch (image->rank()) {
            case ImageRank::_1D: {
                if (image->layerCount() > 1) {
                    desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE1DARRAY;
                    desc.Texture1DArray.FirstArraySlice = d3dImageView->layerStart();
                    desc.Texture1DArray.ArraySize = d3dImageView->layerCount();
                    desc.Texture1DArray.MipSlice = d3dImageView->mipLevelStart();
                }
                else {
                    desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE1D;
                    desc.Texture1D.MipSlice = d3dImageView->mipLevelStart();
                }
                break;
            }
            case ImageRank::_2D: {
                if (image->layerCount() > 1) {
                    desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2DARRAY;
                    desc.Texture2DArray.FirstArraySlice = d3dImageView->layerStart();
                    desc.Texture2DArray.ArraySize = d3dImageView->layerCount();
                    desc.Texture2DArray.MipSlice = d3dImageView->mipLevelStart();
                }
                else {
                    desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
                    desc.Texture2D.MipSlice = d3dImageView->mipLevelStart();
                }
                break;
            }
            default:
                throw core::LogicError("D3d11: unknown image rank");
                break;
            }
        }
        device_->CreateRenderTargetView(d3dImageView->d3dViewedResource(), &desc, d3dImageView->rtv_.addressOf());
    }
    if (!!(d3dImageView->bindFlags() & ImageBindFlags::DepthStencil)) {
        D3D11_DEPTH_STENCIL_VIEW_DESC desc = {};
        desc.Format = d3dImageView->dxgiFormat();
        if (view->isBuffer()) {
            throw core::LogicError("D3d11: buffer cannot be bound as Depth Stencil");
        }
        else {
            ImagePtr image = view->viewedImage();
            switch (image->rank()) {
            case ImageRank::_1D: {
                if (image->layerCount() > 1) {
                    desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE1DARRAY;
                    desc.Texture1DArray.FirstArraySlice = d3dImageView->layerStart();
                    desc.Texture1DArray.ArraySize = d3dImageView->layerCount();
                    desc.Texture1DArray.MipSlice = d3dImageView->mipLevelStart();
                }
                else {
                    desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE1D;
                    desc.Texture1D.MipSlice = d3dImageView->mipLevelStart();
                }
                break;
            }
            case ImageRank::_2D: {
                if (image->layerCount() > 1) {
                    desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2DARRAY;
                    desc.Texture2DArray.FirstArraySlice = d3dImageView->layerStart();
                    desc.Texture2DArray.ArraySize = d3dImageView->layerCount();
                    desc.Texture2DArray.MipSlice = d3dImageView->mipLevelStart();
                }
                else {
                    desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
                    desc.Texture2D.MipSlice = d3dImageView->mipLevelStart();
                }
                break;
            }
            default:
                throw core::LogicError("D3d11: unknown image rank");
                break;
            }
        }
        device_->CreateDepthStencilView(d3dImageView->d3dViewedResource(), &desc, d3dImageView->dsv_.addressOf());
    }
}

void D3d11Engine::initSamplerState_(SamplerState* state)
{
    D3d11SamplerState* d3dSamplerState = static_cast<D3d11SamplerState*>(state);
    D3D11_SAMPLER_DESC desc = {};
    UINT filter = 0;
    if (d3dSamplerState->maxAnisotropy() >= 1) {
        filter = D3D11_FILTER_ANISOTROPIC;
    }
    else {
        if (d3dSamplerState->magFilter() == FilterMode::Linear) {
            filter |= D3D11_FILTER_MIN_POINT_MAG_LINEAR_MIP_POINT;
        }
        if (d3dSamplerState->minFilter() == FilterMode::Linear) {
            filter |= D3D11_FILTER_MIN_LINEAR_MAG_MIP_POINT;
        }
        if (d3dSamplerState->mipFilter() == FilterMode::Linear) {
            filter |= D3D11_FILTER_MIN_MAG_POINT_MIP_LINEAR;
        }
    }
    if (d3dSamplerState->comparisonFunction() != ComparisonFunction::Disabled) {
        filter |= D3D11_FILTER_COMPARISON_MIN_MAG_MIP_POINT;
    }
    desc.Filter = static_cast<D3D11_FILTER>(filter);
    desc.AddressU = imageWrapModeToD3DTextureAddressMode(d3dSamplerState->wrapModeU());
    desc.AddressV = imageWrapModeToD3DTextureAddressMode(d3dSamplerState->wrapModeV());
    desc.AddressW = imageWrapModeToD3DTextureAddressMode(d3dSamplerState->wrapModeW());
    desc.MipLODBias = d3dSamplerState->mipLODBias();
    desc.MaxAnisotropy = d3dSamplerState->maxAnisotropy();
    desc.ComparisonFunc = comparisonFunctionToD3DComparisonFunc(d3dSamplerState->comparisonFunction());
    // XXX add data() in vec4f
    memcpy(desc.BorderColor, &d3dSamplerState->wrapColor(), 4 * sizeof(float));
    desc.MinLOD = d3dSamplerState->minLOD();
    desc.MaxLOD = d3dSamplerState->maxLOD();
    device_->CreateSamplerState(&desc, d3dSamplerState->object_.addressOf());
}

void D3d11Engine::initGeometryView_(GeometryView* view)
{
    D3d11GeometryView* d3dGeometryView = static_cast<D3d11GeometryView*>(view);
    // no-op ?
}

void D3d11Engine::initBlendState_(BlendState* state)
{
    D3d11BlendState* d3dBlendState = static_cast<D3d11BlendState*>(state);
    D3D11_BLEND_DESC desc = {};
    desc.AlphaToCoverageEnable = state->isAlphaToCoverageEnabled();
    desc.IndependentBlendEnable = state->isIndependentBlendEnabled();
    for (Int i = 0; i < 8; ++i) {
        const TargetBlendState& subState = state->targetBlendState(i);
        D3D11_RENDER_TARGET_BLEND_DESC& subDesc = desc.RenderTarget[i];
        subDesc.BlendEnable = subState.isEnabled();
        subDesc.SrcBlend = blendFactorToD3DBlend(subState.equationRGB().sourceFactor());
        subDesc.DestBlend = blendFactorToD3DBlend(subState.equationRGB().targetFactor());
        subDesc.BlendOp = blendOpToD3DBlendOp(subState.equationRGB().operation());
        subDesc.SrcBlendAlpha = blendFactorToD3DBlend(subState.equationAlpha().sourceFactor());
        subDesc.DestBlendAlpha = blendFactorToD3DBlend(subState.equationAlpha().targetFactor());
        subDesc.BlendOpAlpha = blendOpToD3DBlendOp(subState.equationAlpha().operation());
        subDesc.RenderTargetWriteMask = 0;
        if (!!(subState.writeMask() & BlendWriteMask::R)) {
            subDesc.RenderTargetWriteMask |= D3D11_COLOR_WRITE_ENABLE_RED;
        }
        if (!!(subState.writeMask() & BlendWriteMask::G)) {
            subDesc.RenderTargetWriteMask |= D3D11_COLOR_WRITE_ENABLE_GREEN;
        }
        if (!!(subState.writeMask() & BlendWriteMask::B)) {
            subDesc.RenderTargetWriteMask |= D3D11_COLOR_WRITE_ENABLE_BLUE;
        }
        if (!!(subState.writeMask() & BlendWriteMask::A)) {
            subDesc.RenderTargetWriteMask |= D3D11_COLOR_WRITE_ENABLE_ALPHA;
        }
    }
    device_->CreateBlendState(&desc, d3dBlendState->object_.addressOf());
}

void D3d11Engine::initRasterizerState_(RasterizerState* state)
{
    D3d11RasterizerState* d3dRasterizerState = static_cast<D3d11RasterizerState*>(state);
    D3D11_RASTERIZER_DESC desc = {};
    desc.FillMode = fillModeToD3DFilleMode(state->fillMode());
    desc.CullMode = cullModeToD3DCulleMode(state->cullMode());
    desc.FrontCounterClockwise = state->isFrontCounterClockwise();
    //desc.DepthBias;
    //desc.DepthBiasClamp;
    //desc.SlopeScaledDepthBias;
    desc.DepthClipEnable = state->isDepthClippingEnabled();
    desc.ScissorEnable = state->isScissoringEnabled();
    desc.MultisampleEnable = state->isMultisamplingEnabled();
    desc.AntialiasedLineEnable = state->isLineAntialiasingEnabled();
    device_->CreateRasterizerState(&desc, d3dRasterizerState->object_.addressOf());
}



void D3d11Engine::setSwapChain_(SwapChain* swapChain)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    setFramebuffer_(d3dSwapChain->d3dDefaultFrameBuffer());
    const float blend_factor[4] = { 0.f, 0.f, 0.f, 0.f };
    deviceCtx_->OMSetDepthStencilState(depthStencilState_.get(), 0);
    deviceCtx_->HSSetShader(NULL, NULL, 0);
    deviceCtx_->DSSetShader(NULL, NULL, 0);
    deviceCtx_->CSSetShader(NULL, NULL, 0);
}

void D3d11Engine::setFramebuffer_(Framebuffer* framebuffer)
{
    if (!framebuffer) {
        deviceCtx_->OMSetRenderTargets(0, NULL, NULL);
        return;
    }
    D3d11Framebuffer* d3dFramebuffer = static_cast<D3d11Framebuffer*>(framebuffer);
    ID3D11RenderTargetView* rtvArray[1] = { d3dFramebuffer->rtvObject() };
    deviceCtx_->OMSetRenderTargets(1, rtvArray, d3dFramebuffer->dsvObject());
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

void D3d11Engine::setProgram_(Program* program)
{
    D3d11Program* d3dProgram = static_cast<D3d11Program*>(program);
    deviceCtx_->VSSetShader(d3dProgram->vertexShader_.get(), NULL, 0);
    deviceCtx_->PSSetShader(d3dProgram->pixelShader_.get(), NULL, 0);
    deviceCtx_->GSSetShader(d3dProgram->geometryShader_.get(), NULL, 0);
    //deviceCtx_->VSSetConstantBuffers(0, 1, &vertexConstantBuffer_);
    //deviceCtx_->IASetInputLayout(inputLayout_.get());
}

void D3d11Engine::setBlendState_(BlendState* state, const geometry::Vec4f& blendFactor)
{
    D3d11BlendState* d3dBlendState = static_cast<D3d11BlendState*>(state);
    // XXX blendFactor.data()
    deviceCtx_->OMSetBlendState(d3dBlendState->object(), (float*)&blendFactor, 0xFFFFFFFF);
}


// XXX left todo:
/*
void setRasterizerState_(RasterizerState* state) override;
void setStageConstantBuffers_(Buffer* const* buffers, Int startIndex, Int count, ShaderStage shaderStage) override;
void setStageImageViews_(ImageView* const* views, Int startIndex, Int count, ShaderStage shaderStage) override;
void setStageSamplers_(SamplerState* const* states, Int startIndex, Int count, ShaderStage shaderStage) override;
*/



//deviceCtx_->RSSetState(rasterizerState_.get());
//deviceCtx_->VSSetConstantBuffers(0, 1, &vertexConstantBuffer_);
//deviceCtx_->IASetInputLayout(inputLayout_.get());


void D3d11Engine::resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    IDXGISwapChain* dxgiSwapChain = d3dSwapChain->dxgiSwapChain();

    d3dSwapChain->clearDefaultFramebuffer();

    HRESULT hres = dxgiSwapChain->ResizeBuffers(
        0, width, height, DXGI_FORMAT_UNKNOWN, 0);
    if (hres < 0) {
        throw core::LogicError("D3d11Engine: could not resize swap chain buffers");
    }

    ComPtr<ID3D11Texture2D> backBuffer;
    dxgiSwapChain->GetBuffer(0, IID_PPV_ARGS(backBuffer.addressOf()));

    D3D11_TEXTURE2D_DESC backBufferDesc = {};
    backBuffer->GetDesc(&backBufferDesc);

    ImageCreateInfo imageCreateInfo = {};
    imageCreateInfo.setRank(ImageRank::_2D);
    // XXX fill it using backBufferDesc

    D3d11ImagePtr backBufferImage(new D3d11Image(gcResourceList_, imageCreateInfo));
    backBufferImage->object_ = backBuffer;

    ImageViewCreateInfo viewCreateInfo = {};
    viewCreateInfo.setBindFlags(ImageBindFlags::RenderTarget);
    D3d11ImageViewPtr colorView(new D3d11ImageView(gcResourceList_, viewCreateInfo, backBufferImage, d3dSwapChain->backBufferFormat(), 0));

    ComPtr<ID3D11RenderTargetView> backBufferView;
    device_->CreateRenderTargetView(backBuffer.get(), NULL, backBufferView.addressOf());
    colorView->rtv_ = backBufferView.get();

    D3d11FramebufferPtr newFramebuffer(
        new D3d11Framebuffer(
            gcResourceList_,
            colorView,
            D3d11ImageViewPtr(),
            true));

    d3dSwapChain->setDefaultFramebuffer(newFramebuffer);
}

void D3d11Engine::updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes)
{
    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
    loadBuffer_(d3dBuffer->objectAddress(), d3dBuffer->descAddress(), data, lengthInBytes);
    d3dBuffer->lengthInBytes_ = lengthInBytes;
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

UInt64 D3d11Engine::present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags /*flags*/)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    d3dSwapChain->dxgiSwapChain()->Present(syncInterval, 0);
    return std::chrono::nanoseconds(std::chrono::steady_clock::now() - startTime_).count();
}


//void D3d11Engine::setProjectionMatrix_(const geometry::Mat4f& m)
//{
//    paintVertexShaderConstantBuffer_.projMatrix = m;
//    writeBufferReserved_(
//        vertexConstantBuffer_.get(), &paintVertexShaderConstantBuffer_,
//        sizeof(PaintVertexShaderConstantBuffer));
//}
//
//void D3d11Engine::setViewMatrix_(const geometry::Mat4f& m)
//{
//    paintVertexShaderConstantBuffer_.viewMatrix = m;
//    writeBufferReserved_(
//        vertexConstantBuffer_.get(), &paintVertexShaderConstantBuffer_,
//        sizeof(PaintVertexShaderConstantBuffer));
//}




//void D3d11Engine::setupVertexBufferForPaintShader_(Buffer* /*buffer*/)
//{
//    // no-op
//}

//void D3d11Engine::drawPrimitives_(Buffer* buffer, PrimitiveType type)
//{
//    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
//    ID3D11Buffer* object = d3dBuffer->object();
//    if (!object) return;
//
//    D3D_PRIMITIVE_TOPOLOGY d3dTopology = {};
//    switch (type) {
//    case PrimitiveType::Point:
//        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_POINTLIST; break;
//    case PrimitiveType::LineList:
//        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_LINELIST; break;
//    case PrimitiveType::LineStrip:
//        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP; break;
//    case PrimitiveType::TriangleList:
//        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST; break;
//    case PrimitiveType::TriangleStrip:
//        d3dTopology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP; break;
//    default:
//        throw core::LogicError("D3d11Buffer: unsupported primitive type");
//    }
//
//    Int numVertices = d3dBuffer->lengthInBytes_ / sizeof(XYRGBVertex);
//    unsigned int stride = sizeof(XYRGBVertex);
//    unsigned int offset = 0;
//    deviceCtx_->IASetVertexBuffers(0, 1, &object, &stride, &offset);
//    deviceCtx_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
//    deviceCtx_->Draw(core::int_cast<UINT>(numVertices), 0);
//}



//void D3d11Engine::releasePaintShader_()
//{
//    deviceCtx_->VSSetShader(NULL, NULL, 0);
//    deviceCtx_->VSSetConstantBuffers(0, 0, NULL);
//    deviceCtx_->PSSetShader(NULL, NULL, 0);
//}

// Private methods

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

//void D3d11Engine::updatePaintVertexShaderConstantBuffer_()
//{
//    PaintVertexShaderConstantBuffer buffer = {};
//    memcpy(buffer.projMatrix.data(), projMatrix_.data(), 16 * sizeof(float));
//    memcpy(buffer.viewMatrix.data(), viewMatrix_.data(), 16 * sizeof(float));
//    writeBufferReserved_(
//        vertexConstantBuffer_.get(), &buffer,
//        sizeof(PaintVertexShaderConstantBuffer));
//}





} // namespace vgc::graphics

#endif // VGC_CORE_COMPILER_MSVC
