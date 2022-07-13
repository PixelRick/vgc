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
    D3d11Buffer(ResourceList* gcList, const BufferCreateInfo& createInfo);

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
    friend D3d11Engine;

    ComPtr<ID3D11Buffer> object_;
    D3D11_BUFFER_DESC desc_ = {};
};
using D3d11BufferPtr = ResourcePtr<D3d11Buffer>;

class D3d11Image : public Image {
public:
    D3d11Image(ResourceList* gcList, const ImageCreateInfo& createInfo);

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
    friend D3d11Engine;

    ComPtr<ID3D11Resource> object_;
    DXGI_FORMAT dxgiFormat_;
};
using D3d11ImagePtr = ResourcePtr<D3d11Image>;

class D3d11ImageView : public ImageView {
public:
    D3d11ImageView(ResourceList* gcList, const ImagePtr& image);
    D3d11ImageView(ResourceList* gcList, const BufferPtr& buffer, ImageFormat format);

    ID3D11View* object() const
    {
        return object_.get();
    }

    // resizing a swap chain requires releasing all views to its back buffers.
    void forceRelease()
    {
        object_.reset();
    }

protected:
    void release_(Engine* engine) override
    {
        ImageView::release_(engine);
        forceRelease();
    }

private:
    friend D3d11Engine;

    ComPtr<ID3D11View> object_;
    DXGI_FORMAT dxgiFormat_;
};
using D3d11ImageViewPtr = ResourcePtr<D3d11ImageView>;

class D3d11GeometryView : public GeometryView {
public:
    D3d11GeometryView(ResourceList* gcList, const GeometryViewCreateInfo& createInfo);

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
public:
    D3d11Program(ResourceList* gcList)
        : Program(gcList)
    {
    }

protected:
    void release_(Engine* engine) override
    {
        Program::release_(engine);
        vertexShader_.reset();
        geometryShader_.reset();
        pixelShader_.reset();
    }

private:
    friend D3d11Engine;

    ComPtr<ID3D11VertexShader> vertexShader_;
    ComPtr<ID3D11GeometryShader> geometryShader_;
    ComPtr<ID3D11PixelShader> pixelShader_;
};
using D3d11ProgramPtr = ResourcePtr<D3d11Program>;

class D3d11BlendState : public BlendState {
public:
    D3d11BlendState::D3d11BlendState(ResourceList* gcList, const BlendStateCreateInfo& createInfo)
        : BlendState(gcList, createInfo)
    {
    }

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
    friend D3d11Engine;

    ComPtr<ID3D11BlendState> object_;
};
using D3d11BlendStatePtr = ResourcePtr<D3d11BlendState>;

class D3d11RasterizerState : public RasterizerState {
public:
    D3d11RasterizerState(ResourceList* gcList, const RasterizerStateCreateInfo& createInfo)
        : RasterizerState(gcList, createInfo)
    {
    }

    ID3D11RasterizerState* object() const
    {
        return object_.get();
    }

protected:
    void release_(Engine* engine) override
    {
        RasterizerState::release_(engine);
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

    ID3D11RenderTargetView* colorView() const
    {
        return colorView_ ?
            static_cast<ID3D11RenderTargetView*>(colorView_->object()) : nullptr;
    }

    ID3D11DepthStencilView* depthStencilView() const
    {
        return depthStencilView_ ?
            static_cast<ID3D11DepthStencilView*>(depthStencilView_->object()) : nullptr;
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
        return D3D11_USAGE_DEFAULT; break;
    case Usage::Immutable:
        return D3D11_USAGE_IMMUTABLE; break;
    case Usage::Dynamic:
        return D3D11_USAGE_DYNAMIC; break;
    case Usage::Staging:
        return D3D11_USAGE_STAGING; break;
    default:
        break;
    }
    throw core::LogicError("D3d11Engine: unsupported usage");
}




// RESOURCE CONSTRUCTORS

D3d11Buffer::D3d11Buffer(ResourceList* gcList, const BufferCreateInfo& createInfo)
    : Buffer(gcList, createInfo)
{
    desc_.Usage = usageToD3DUsage(createInfo.usage());

    const BindFlags bindFlags = createInfo.bindFlags();
    if (!!(bindFlags & BindFlags::UniformBuffer)) {
        desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER;
        if (bindFlags != BindFlags::UniformBuffer) {
            throw core::LogicError("D3d11Buffer: BindFlags::UniformBuffer cannot be combined with any other bind flag");
        }
    }
    else {
        if (!!(bindFlags & BindFlags::VertexBuffer)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_VERTEX_BUFFER;
        }
        if (!!(bindFlags & BindFlags::IndexBuffer)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_INDEX_BUFFER;
        }
        if (!!(bindFlags & BindFlags::UniformBuffer)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_CONSTANT_BUFFER;
        }
        if (!!(bindFlags & BindFlags::ShaderResource)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_SHADER_RESOURCE;
        }
        if (!!(bindFlags & BindFlags::StreamOutput)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_STREAM_OUTPUT;
        }
        if (!!(bindFlags & BindFlags::Image)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_RENDER_TARGET;
        }
        if (!!(bindFlags & BindFlags::DepthStencil)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_DEPTH_STENCIL;
        }
        if (!!(bindFlags & BindFlags::UnorderedAccess)) {
            desc_.BindFlags |= D3D11_BIND_FLAG::D3D11_BIND_UNORDERED_ACCESS;
        }
    }

    const ResourceMiscFlags resourceMiscFlags = createInfo.resourceMiscFlags();
    if (!!(resourceMiscFlags & ResourceMiscFlags::GenerateMips)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_GENERATE_MIPS;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::Shared)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_SHARED;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::TextureCube)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_TEXTURECUBE;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::DrawIndirectArgs)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::BufferRaw)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::BufferStructured)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::ResourceClamp)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_RESOURCE_CLAMP;
    }
    if (!!(resourceMiscFlags & ResourceMiscFlags::SharedKeyedMutex)) {
        desc_.MiscFlags |= D3D11_RESOURCE_MISC_FLAG::D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX;
    }

    const CpuAccessFlags cpuAccessFlags = createInfo.cpuAccessFlags();
    if (!!(cpuAccessFlags & CpuAccessFlags::Write)) {
        desc_.CPUAccessFlags |= D3D11_CPU_ACCESS_FLAG::D3D11_CPU_ACCESS_WRITE;
    }
    if (!!(cpuAccessFlags & CpuAccessFlags::Read)) {
        desc_.CPUAccessFlags |= D3D11_CPU_ACCESS_FLAG::D3D11_CPU_ACCESS_READ;
    }
}

D3d11Image::D3d11Image(ResourceList* gcList, const ImageCreateInfo& createInfo)
    : Image(gcList, createInfo)
    , dxgiFormat_(imageFormatToDxgiFormat(createInfo.format()))
{
    if (dxgiFormat_ == DXGI_FORMAT_UNKNOWN) {
        throw core::LogicError("D3d11Image: unknown image format");
    }
}

D3d11ImageView::D3d11ImageView(ResourceList* gcList, const ImagePtr& image)
    : ImageView(gcList, image, image->format())
    , dxgiFormat_(static_cast<D3d11Image*>(image.get())->dxgiFormat())
{
}

D3d11ImageView::D3d11ImageView(ResourceList* gcList, const BufferPtr& buffer, ImageFormat format)
    : ImageView(gcList, buffer, format)
    , dxgiFormat_(imageFormatToDxgiFormat(format))
{
    if (dxgiFormat_ == DXGI_FORMAT_UNKNOWN) {
        throw core::LogicError("D3d11ImageView: unknown image format");
    }
}

D3d11GeometryView::D3d11GeometryView(ResourceList* gcList, const GeometryViewCreateInfo& createInfo)
    : GeometryView(gcList, createInfo)
    , topology_(primitiveTypeToD3DPrimitiveTopology(createInfo.primitiveType()))
{
    if (topology_ == D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED) {
        throw core::LogicError("D3d11GeometryView: unknown primitive type");
    }
}

// ENGINE FUNCTIONS

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

SwapChain* D3d11Engine::createSwapChain_(const SwapChainCreateInfo& desc)
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
    case SwapChainTargetFormat::RGBA_8_UNORM:
        sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM; break;
    case SwapChainTargetFormat::RGBA_8_UNORM_SRGB:
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
    D3d11ImageViewPtr colorView(new D3d11ImageView(gcResourceList_, backBufferView.get()));
    D3d11FramebufferPtr newFramebuffer(
        new D3d11Framebuffer(
            gcResourceList_,
            colorView,
            D3d11ImageViewPtr()));

    D3d11SwapChain* d3dSwapChain = new D3d11SwapChain(gcResourceList_, desc, swapChain.get());
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
            gcResourceList_,
            D3d11ImageViewPtr(new D3d11ImageView(gcResourceList_, backBufferView.get())),
            D3d11ImageViewPtr()));
    d3dSwapChain->setDefaultFramebuffer(newFramebuffer);
}


Buffer* D3d11Engine::createBuffer_(const BufferCreateInfo& createInfo)
{
    return new D3d11Buffer(gcResourceList_, createInfo);
}

Image* D3d11Engine::createImage_(const ImageCreateInfo& createInfo)
{
    return new D3d11Image(gcResourceList_, createInfo);
}

ImageView* D3d11Engine::createImageView_(const ImagePtr& image)
{
    return new D3d11ImageView(gcResourceList_, image);
}

ImageView* D3d11Engine::createImageView_(const BufferPtr& buffer, ImageFormat format)
{
    return new D3d11ImageView(gcResourceList_, buffer, format);
}

GeometryView* D3d11Engine::createGeometryView_(const GeometryViewCreateInfo& createInfo)
{
    return new D3d11GeometryView(gcResourceList_, createInfo);
}

BlendState* D3d11Engine::createBlendState_(const BlendStateCreateInfo& createInfo)
{
    return new D3d11BlendState(gcResourceList_, createInfo);
}

RasterizerState* D3d11Engine::createRasterizerState_(const RasterizerStateCreateInfo& createInfo)
{
    return new D3d11RasterizerState(gcResourceList_, createInfo);
}

Framebuffer* D3d11Engine::createFramebuffer_(const ImageViewPtr& colorImageView)
{
    return new D3d11Framebuffer(gcResourceList_, colorImageView, nullptr, false);
}

// RENDER THREAD functions

void D3d11Engine::setSwapChain_(SwapChain* swapChain)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    setFramebuffer_(d3dSwapChain->d3dDefaultFrameBuffer());
    const float blend_factor[4] = { 0.f, 0.f, 0.f, 0.f };
    deviceCtx_->OMSetDepthStencilState(depthStencilState_.get(), 0);

    //deviceCtx_->OMSetBlendState(blendState_.get(), blend_factor, 0xffffffff);
    //deviceCtx_->RSSetState(rasterizerState_.get());
}

UInt64 D3d11Engine::present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags /*flags*/)
{
    D3d11SwapChain* d3dSwapChain = static_cast<D3d11SwapChain*>(swapChain);
    d3dSwapChain->dxgiSwapChain()->Present(syncInterval, 0);
    return std::chrono::nanoseconds(std::chrono::steady_clock::now() - startTime_).count();
}


void D3d11Engine::initFramebuffer_(Framebuffer* framebuffer)
{
    // no-op
}

void D3d11Engine::initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes)
{
    D3d11Buffer* d3dBuffer = static_cast<D3d11Buffer*>(buffer);
    if (initialLengthInBytes) {
        loadBuffer_(d3dBuffer->objectAddress(), d3dBuffer->descAddress(), data, initialLengthInBytes);
    }
    d3dBuffer->lengthInBytes_ = initialLengthInBytes;
}

void D3d11Engine::initImage_(Image* image, const void* data)
{
    D3d11Image* d3dImage = static_cast<D3d11Image*>(image);
    if (d3dImage->rank() == ImageRank::_1D) {
        D3D11_TEXTURE1D_DESC desc = {};
        desc.Width = d3dImage->width();
        desc.MipLevels = d3dImage->mipLevelCount();
        desc.ArraySize = d3dImage->layerCount();
        desc.Format = d3dImage->dxgiFormat();
        desc.Usage = usageToD3DUsage(d3dImage->usage());
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        if (d3dImage->isRenderTargetable()) {
            desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
        }
        desc.CPUAccessFlags = d3dImage->cpuAccessFlags();
        desc.MiscFlags = d3dImage->resourceMiscFlags();
    }
    else {
        VGC_CORE_ASSERT(d3dImage->rank() == ImageRank::_2D);
        D3D11_TEXTURE2D_DESC desc = {};

    }
}

void D3d11Engine::initImageView_(ImageView* view)
{
    D3d11ImageView* d3dImageView = static_cast<D3d11ImageView*>(view);

}

void D3d11Engine::initGeometryView_(GeometryView* view)
{
    D3d11GeometryView* d3dGeometryView = static_cast<D3d11GeometryView*>(view);

}

void D3d11Engine::initBlendState_(BlendState* state)
{
    D3d11BlendState* d3dBlendState = static_cast<D3d11BlendState*>(state);

}

void D3d11Engine::initRasterizerState_(RasterizerState* state)
{
    D3d11RasterizerState* d3dRasterizerState = static_cast<D3d11RasterizerState*>(state);

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
    paintVertexShaderConstantBuffer_.projMatrix = m;
    writeBufferReserved_(
        vertexConstantBuffer_.get(), &paintVertexShaderConstantBuffer_,
        sizeof(PaintVertexShaderConstantBuffer));
}

void D3d11Engine::setViewMatrix_(const geometry::Mat4f& m)
{
    paintVertexShaderConstantBuffer_.viewMatrix = m;
    writeBufferReserved_(
        vertexConstantBuffer_.get(), &paintVertexShaderConstantBuffer_,
        sizeof(PaintVertexShaderConstantBuffer));
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

void D3d11Engine::updatePaintVertexShaderConstantBuffer_()
{
    PaintVertexShaderConstantBuffer buffer = {};
    memcpy(buffer.projMatrix.data(), projMatrix_.data(), 16 * sizeof(float));
    memcpy(buffer.viewMatrix.data(), viewMatrix_.data(), 16 * sizeof(float));
    writeBufferReserved_(
        vertexConstantBuffer_.get(), &buffer,
        sizeof(PaintVertexShaderConstantBuffer));
}





} // namespace vgc::graphics

#endif // VGC_CORE_COMPILER_MSVC
