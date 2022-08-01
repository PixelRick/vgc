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

#include <vgc/ui/internal/qopenglengine.h>

#include <chrono>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QOpenGLVersionFunctionsFactory>
#endif
#include <QWindow>

#include <vgc/core/exceptions.h>
#include <vgc/core/paths.h>

namespace vgc::ui::internal::qopengl {

namespace {

// Returns the file path of a shader file as a QString
QString shaderPath_(const std::string& name)
{
    std::string path = core::resourcePath("graphics/opengl/" + name);
    return toQt(path);
}

struct XYRGBVertex {
    float x, y, r, g, b;
};

} // namespace

class QglImageView;
class QglFramebuffer;

class QglBuffer : public Buffer {
protected:
    friend QglEngine;
    using Buffer::Buffer;

public:
    GLuint object() const
    {
        return object_;
    }

protected:
    void release_(Engine* engine) override
    {
        Buffer::release_(engine);
        static_cast<QglEngine*>(engine)->api()->glDeleteBuffers(1, &object_);
    }

private:
    GLuint object_ = static_cast<GLuint>(-1);
    GLenum target_ = 0;
};
using QglBufferPtr = ResourcePtr<QglBuffer>;

class QglImage : public Image {
protected:
    friend QglEngine;
    using Image::Image;

public:
    GLuint object() const
    {
        return object_;
    }

    GLint internalFormat() const
    {
        return internalFormat_;
    }

protected:
    void release_(Engine* engine) override
    {
        Image::release_(engine);
        static_cast<QglEngine*>(engine)->api()->glDeleteTextures(1, &object_);
    }

private:
    GLuint object_ = static_cast<GLuint>(-1);
    GLenum target_ = 0;
    GLint internalFormat_ = 0;
};
using QglImagePtr = ResourcePtr<QglImage>;

class QglImageView : public ImageView {
protected:
    friend QglEngine;

    QglImageView(ResourceRegistry* registry,
                 const ImageViewCreateInfo& createInfo,
                 const ImagePtr& image)
        : ImageView(registry, createInfo, image) {
    }

    QglImageView(ResourceRegistry* registry,
                 const ImageViewCreateInfo& createInfo,
                 const BufferPtr& buffer,
                 ImageFormat format,
                 UInt32 numBufferElements)
        : ImageView(registry, createInfo, buffer, format, numBufferElements) {

    }

public:
    GLuint internalFormat() const
    {
        return internalFormat_;
    }

protected:
    void release_(Engine* engine) override
    {
        ImageView::release_(engine);
        // ..
    }

private:
    GLuint internalFormat_ = 0;
};
using QglImageViewPtr = ResourcePtr<QglImageView>;

class QglSamplerState : public SamplerState {
    friend QglEngine;
    using SamplerState::SamplerState;
};
using QglSamplerStatePtr = ResourcePtr<QglSamplerState>;

class QglGeometryView : public GeometryView {
protected:
    friend QglEngine;
    using GeometryView::GeometryView;

public:
    GLenum topology() const {
        return topology_;
    }

protected:
    void release_(Engine* engine) override
    {
        GeometryView::release_(engine);
        // XXX release cached vaos
    }

private:
    GLenum topology_;

    // XXX VAO cache ?
    //std::array<ComPtr<ID3D11InputLayout>, core::toUnderlying(BuiltinGeometryLayout::Max_) + 1> builtinLayouts_;
};
using QglGeometryViewPtr = ResourcePtr<QglGeometryView>;

class QglProgram : public Program {
protected:
    friend QglEngine;
    using Program::Program;

public:
    // ..

protected:
    void release_(Engine* engine) override
    {
        Program::release_(engine);
        prog_->release();
        prog_.reset();
    }

private:
    std::unique_ptr<QOpenGLShaderProgram> prog_;
};
using QglProgramPtr = ResourcePtr<QglProgram>;

class QglBlendState : public BlendState {
protected:
    friend QglEngine;
    using BlendState::BlendState;

public:
    // ..

protected:
    void release_(Engine* engine) override
    {
        BlendState::release_(engine);
        // ..
    }

private:
    // ..
};
using QglBlendStatePtr = ResourcePtr<QglBlendState>;

class QglRasterizerState : public RasterizerState {
protected:
    friend QglEngine;
    using RasterizerState::RasterizerState;

public:
    // ..

protected:
    void release_(Engine* engine) override
    {
        RasterizerState::release_(engine);
        // ..
    }

private:
    // ..
};
using QglRasterizerStatePtr = ResourcePtr<QglRasterizerState>;

// -------- WIP ----------

// no equivalent in D3D11, see OMSetRenderTargets
class QglFramebuffer : public Framebuffer {
protected:
    friend QglEngine;

    QglFramebuffer(
        ResourceRegistry* registry,
        const QglImageViewPtr& colorView,
        const QglImageViewPtr& depthStencilView,
        bool isDefault)
        : Framebuffer(registry)
        , colorView_(colorView)
        , depthStencilView_(depthStencilView)
        , isDefault_(isDefault)
    {
        if (colorView_) {
            colorView_->dependentD3dFramebuffers_.append(this);
            d3dColorView_ = colorView_.get();
        }
        if (depthStencilView_) {
            depthStencilView_->dependentD3dFramebuffers_.append(this);
            d3dDepthStencilView_ = depthStencilView_.get();
        }
    }

public:
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
    void forceReleaseColorViewD3dObject()
    {
        colorView_->forceReleaseD3dObject();
    }

protected:
    void releaseSubResources_() override
    {
        colorView_.reset();
        depthStencilView_.reset();
    }

    void release_(Engine* engine) override;

private:
    QglImageViewPtr colorView_;
    QglImageViewPtr depthStencilView_;

    bool isDefault_ = false;

    bool isBoundToD3D_ = false;

    friend QglImageView;
    QglImageView* d3dColorView_ = nullptr; // used to clear backpointer at release time
    QglImageView* d3dDepthStencilView_ = nullptr; // used to clear backpointer at release time
};
using QglFramebufferPtr = ResourcePtr<QglFramebuffer>;

class QglSwapChain : public SwapChain {
public:
    QglSwapChain(ResourceRegistry* registry,
                   const SwapChainCreateInfo& desc,
                   IDXGISwapChain* dxgiSwapChain)
        : SwapChain(registry, desc)
        , dxgiSwapChain_(dxgiSwapChain) {
    }

    IDXGISwapChain* dxgiSwapChain() const
    {
        return dxgiSwapChain_.get();
    }

    // can't be called from render thread
    void setDefaultFramebuffer(const QglFramebufferPtr& defaultFrameBuffer)
    {
        defaultFrameBuffer_ = defaultFrameBuffer;
    }

    // can't be called from render thread
    void clearDefaultFramebuffer()
    {
        if (defaultFrameBuffer_) {
            static_cast<QglFramebuffer*>(defaultFrameBuffer_.get())->forceReleaseColorViewD3dObject();
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
    throw core::LogicError("QglEngine: unsupported usage");
}

UINT resourceMiscFlagsToD3DResourceMiscFlags(ResourceMiscFlags resourceMiscFlags)
{
    UINT x = 0;
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
    throw core::LogicError("QglEngine: unknown image wrap mode");
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
    throw core::LogicError("QglEngine: unknown comparison func");
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
    throw core::LogicError("QglEngine: unknown blend factor");
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
    throw core::LogicError("QglEngine: unknown blend op");
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
    throw core::LogicError("QglEngine: unknown fill mode");
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
    throw core::LogicError("QglEngine: unknown cull mode");
}

// ENGINE FUNCTIONS


//namespace {
//
//class OpenglBuffer : public Buffer {
//public:
//    OpenglBuffer(
//        ResourceRegistry* registry,
//        Usage usage,
//        BindFlags bindFlags,
//        ResourceMiscFlags resourceMiscFlags,
//        CpuAccessFlags cpuAccessFlags);
//
//    void init(const void* data, Int length)
//    {
//        if (!vao_) {
//            QOpenGLBuffer::Type t = {};
//            switch (bindFlags()) {
//            case BindFlags::IndexBuffer:
//                t = QOpenGLBuffer::Type::IndexBuffer; break;
//            case BindFlags::VertexBuffer:
//                t = QOpenGLBuffer::Type::VertexBuffer; break;
//            default:
//                throw core::LogicError("QOpenglBuffer: unsupported bind flags");
//            }
//
//            bool cpuWrites = !!(cpuAccessFlags() & CpuAccessFlags::Write);
//            QOpenGLBuffer::UsagePattern u = QOpenGLBuffer::StaticDraw;
//            switch (usage()) {
//            case Usage::Immutable: // no equivalent
//                u = QOpenGLBuffer::UsagePattern::StaticDraw; break;
//            case Usage::Dynamic:
//                u = QOpenGLBuffer::UsagePattern::DynamicDraw; break;
//            case Usage::Staging:
//                u = cpuWrites
//                    ? QOpenGLBuffer::UsagePattern::DynamicCopy
//                    : QOpenGLBuffer::UsagePattern::StaticCopy; break;
//            default:
//                break;
//            }
//
//            vbo_ = new QOpenGLBuffer(t);
//            vbo_->setUsagePattern(u);
//            vbo_->create();
//
//            vao_ = new QOpenGLVertexArrayObject();
//            vao_->create();
//
//            numVertices_ = length / sizeof(XYRGBVertex);
//            Int dataSize = numVertices_ * sizeof(XYRGBVertex);
//            static_assert(sizeof(XYRGBVertex) == 5 * sizeof(float));
//
//            if (dataSize) {
//                vbo_->bind();
//                if (data) {
//                    vbo_->allocate(data, length);
//                }
//                else {
//                    vbo_->allocate(length);
//                }
//                allocSize_ = length;
//                vbo_->release();
//            }
//        }
//    }
//
//    void bind()
//    {
//        if (vao_) {
//            vao_->bind();
//            vbo_->bind();
//        }
//    }
//
//    void unbind()
//    {
//        if (vao_) {
//            vbo_->release();
//            vao_->release();
//        }
//    }
//
//    void load(const void* data, Int length)
//    {
//        if (!vao_) return;
//        if (length < 0) {
//            throw core::NegativeIntegerError(core::format(
//                "Negative length ({}) provided to QOpenglBuffer::load()", length));
//        }
//
//        numVertices_ = length / sizeof(XYRGBVertex);
//        Int dataSize = numVertices_ * sizeof(XYRGBVertex);
//        static_assert(sizeof(XYRGBVertex) == 5 * sizeof(float));
//
//        vbo_->bind();
//        if (dataSize > allocSize_) {
//            vbo_->allocate(data, dataSize);
//            allocSize_ = dataSize;
//        }
//        else if (dataSize * 2 < allocSize_) {
//            vbo_->allocate(data, dataSize);
//            allocSize_ = dataSize;
//        }
//        else {
//            vbo_->write(0, data, dataSize);
//        }
//        vbo_->release();
//    }
//
//    void draw(QglEngine* engine, GLenum mode)
//    {
//        if (!allocated_) return;
//        vao_->bind();
//        auto api = engine->api();
//        api->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//        api->glDrawArrays(mode, 0, numVertices_);
//        vao_->release();
//    }
//
//protected:
//    void release_(Engine* engine) override
//    {
//        if (!allocated_) return;
//        auto oglApi = static_cast<QglEngine*>(engine)->api();
//        oglApi->glDeleteVertexArrays(1, &cacheVao_);
//        oglApi->glDeleteBuffers(1, &vbo_);
//
//        inited_ = false;
//        vao_->destroy();
//        delete vao_;
//        vao_ = nullptr;
//        vbo_->destroy();
//        delete vbo_;
//        vbo_ = nullptr;
//    }
//
//private:
//    friend QglEngine;
//
//    bool allocated_ = false;
//    GLuint vbo_;
//    GLuint cachedVao_;
//    void* cachedVaoLayoutId_ = nullptr;
//};
//
//class QOpenglFramebuffer : public Framebuffer {
//public:
//    QOpenglFramebuffer(ResourceRegistry* registry, bool isDefault = false)
//        : Framebuffer(registry)
//        , isDefault_(isDefault)
//    {
//    }
//
//    bool isDefault() const
//    {
//        return isDefault_;
//    }
//
//protected:
//    void release_(Engine* engine) override
//    {
//        if (!isDefault_) {
//            static_cast<QglEngine*>(engine)->api()->glDeleteFramebuffers(1, &object_);
//        }
//    }
//
//private:
//    friend QglEngine;
//
//    GLuint object_ = 0;
//    bool isDefault_ = false;
//};
//
//class QOpenglSwapChain : public SwapChain {
//public:
//    QOpenglSwapChain(ResourceRegistry* registry, const SwapChainCreateInfo& desc, QSurface* surface)
//        : SwapChain(registry, desc)
//        , surface_(surface)
//    {
//        defaultFrameBuffer_.reset(new QOpenglFramebuffer(registry, true));
//    }
//
//    QSurface* surface() const
//    {
//        return surface_;
//    }
//
//protected:
//    void release_(Engine* /*engine*/) override
//    {
//        // no-op..
//    }
//
//private:
//    QSurface* surface_;
//};
//
//} // namespace

QglEngine::QglEngine() :
    QglEngine(nullptr, false)
{
}

QglEngine::QglEngine(QOpenGLContext* ctx, bool isExternalCtx) :
    Engine(),
    ctx_(ctx),
    isExternalCtx_(isExternalCtx)
{
}

void QglEngine::onDestroyed()
{
    Engine::onDestroyed();
    if (!isExternalCtx_) {
        delete ctx_;
    }
}

/* static */
QglEnginePtr QglEngine::create()
{
    return QglEnginePtr(new QglEngine());
}

/* static */
QglEnginePtr QglEngine::create(QOpenGLContext* ctx)
{
    return QglEnginePtr(new QglEngine(ctx));
}

void QglEngine::setupContext()
{
    // Initialize shader program
    //paintShaderProgram_.reset(new QOpenGLShaderProgram());
    //paintShaderProgram_->addShaderFromSourceFile(QOpenGLShader::Vertex, shaderPath_("iv4pos_iv4col_um4proj_um4view_ov4fcol.v.glsl"));
    //paintShaderProgram_->addShaderFromSourceFile(QOpenGLShader::Fragment, shaderPath_("iv4fcol.f.glsl"));
    //paintShaderProgram_->link();

    // Get shader locations
    //paintShaderProgram_->bind();
    //posLoc_  = paintShaderProgram_->attributeLocation("pos");
    //colLoc_  = paintShaderProgram_->attributeLocation("col");
    //projLoc_ = paintShaderProgram_->uniformLocation("proj");
    //viewLoc_ = paintShaderProgram_->uniformLocation("view");
    //paintShaderProgram_->release();

    // Get API 3.2
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    api_ = ctx_->versionFunctions<QOpenGLFunctions_3_3_Core>();
#else
    api_ = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(ctx_);
#endif
    //

    api_->initializeOpenGLFunctions();
}

// -- USER THREAD implementation functions --

void QglEngine::createBuiltinShaders_()
{

}

SwapChainPtr QglEngine::constructSwapChain_(const SwapChainCreateInfo& /*createInfo*/) { return nullptr; }
FramebufferPtr QglEngine::constructFramebuffer_(const ImageViewPtr& /*colorImageView*/) { return nullptr; }
BufferPtr QglEngine::constructBuffer_(const BufferCreateInfo& /*createInfo*/) { return nullptr; }
ImagePtr QglEngine::constructImage_(const ImageCreateInfo& /*createInfo*/) { return nullptr; }
ImageViewPtr QglEngine::constructImageView_(const ImageViewCreateInfo& /*createInfo*/, const ImagePtr& /*image*/) { return nullptr; }
ImageViewPtr QglEngine::constructImageView_(const ImageViewCreateInfo& /*createInfo*/, const BufferPtr& /*buffer*/, ImageFormat /*format*/, UInt32 /*numElements*/) { return nullptr; }
SamplerStatePtr QglEngine::constructSamplerState_(const SamplerStateCreateInfo& /*createInfo*/) { return nullptr; }
GeometryViewPtr QglEngine::constructGeometryView_(const GeometryViewCreateInfo& /*createInfo*/) { return nullptr; }
BlendStatePtr QglEngine::constructBlendState_(const BlendStateCreateInfo& /*createInfo*/) { return nullptr; }
RasterizerStatePtr QglEngine::constructRasterizerState_(const RasterizerStateCreateInfo& /*createInfo*/) { return nullptr; }

void QglEngine::resizeSwapChain_(SwapChain* /*swapChain*/, UInt32 /*width*/, UInt32 /*height*/) {}

//--  RENDER THREAD implementation functions --

void QglEngine::initFramebuffer_(Framebuffer* /*framebuffer*/) {}
void QglEngine::initBuffer_(Buffer* /*buffer*/, const char* /*data*/, Int /*lengthInBytes*/) {}
void QglEngine::initImage_(Image* /*image*/, const Span<const Span<const char>>* /*dataSpanSpan*/) {}
void QglEngine::initImageView_(ImageView* /*view*/) {}
void QglEngine::initSamplerState_(SamplerState* /*state*/) {}
void QglEngine::initGeometryView_(GeometryView* /*view*/) {}
void QglEngine::initBlendState_(BlendState* /*state*/) {}
void QglEngine::initRasterizerState_(RasterizerState* /*state*/) {}

void QglEngine::setSwapChain_(const SwapChainPtr& /*swapChain*/) {}
void QglEngine::setFramebuffer_(const FramebufferPtr& /*framebuffer*/) {}
void QglEngine::setViewport_(Int /*x*/, Int /*y*/, Int /*width*/, Int /*height*/) {}
void QglEngine::setProgram_(const ProgramPtr& /*program*/) {}
void QglEngine::setBlendState_(const BlendStatePtr& /*state*/, const geometry::Vec4f& /*blendFactor*/) {}
void QglEngine::setRasterizerState_(const RasterizerStatePtr& /*state*/) {}
void QglEngine::setStageConstantBuffers_(BufferPtr const* /*buffers*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/) {}
void QglEngine::setStageImageViews_(ImageViewPtr const* /*views*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/) {}
void QglEngine::setStageSamplers_(SamplerStatePtr const* /*states*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/) {}

void QglEngine::updateBufferData_(Buffer* /*buffer*/, const void* /*data*/, Int /*lengthInBytes*/) {}

void QglEngine::draw_(GeometryView* /*view*/, UInt /*numIndices*/, UInt /*numInstances*/) {}
void QglEngine::clear_(const core::Color& /*color*/) {}

UInt64 QglEngine::present_(SwapChain* /*swapChain*/, UInt32 /*syncInterval*/, PresentFlags /*flags*/) { return 0; }

//// USER THREAD pimpl functions
//
//SwapChain* QglEngine::createSwapChain_(const SwapChainCreateInfo& desc)
//{
//    //if (ctx_ == nullptr) {
//    //    throw core::LogicError("ctx_ is null.");
//    //}
//
//    if (desc.windowNativeHandleType() != WindowNativeHandleType::QOpenGLWindow) {
//        return nullptr;
//    }
//
//    format_.setDepthBufferSize(24);
//    format_.setStencilBufferSize(8);
//    format_.setVersion(3, 2);
//    format_.setProfile(QSurfaceFormat::CoreProfile);
//    format_.setSamples(desc.numSamples());
//    format_.setSwapInterval(0);
//
//    QWindow* wnd = static_cast<QWindow*>(desc.windowNativeHandle());
//    wnd->setFormat(format_);
//    wnd->create();
//
//    return new QOpenglSwapChain(resourceRegistry_, desc, wnd);
//}
//
//void QglEngine::resizeSwapChain_(SwapChain* /*swapChain*/, UInt32 /*width*/, UInt32 /*height*/)
//{
//    // no-op
//}
//
//Buffer* QglEngine::createBuffer_(
//    Usage usage, BindFlags bindFlags,
//    ResourceMiscFlags resourceMiscFlags, CpuAccessFlags cpuAccessFlags)
//{
//    return new QOpenglBuffer(resourceRegistry_, usage, bindFlags, resourceMiscFlags, cpuAccessFlags);
//}
//
//// RENDER THREAD functions
//
//void QglEngine::bindSwapChain_(SwapChain* swapChain)
//{
//    QOpenglSwapChain* oglChain = static_cast<QOpenglSwapChain*>(swapChain);
//    surface_ = oglChain->surface();
//
//    if (!ctx_) {
//        ctx_ = new QOpenGLContext();
//        ctx_->setFormat(format_);
//        ctx_->create();
//    }
//
//    ctx_->makeCurrent(surface_);
//    if (!api_) {
//        setupContext();
//    }
//}
//
//UInt64 QglEngine::present_(SwapChain* swapChain, UInt32 /*syncInterval*/, PresentFlags /*flags*/)
//{
//    // XXX check valid ?
//    auto oglChain = static_cast<QOpenglSwapChain*>(swapChain);
//    ctx_->swapBuffers(oglChain->surface());
//    return std::chrono::nanoseconds(std::chrono::steady_clock::now() - startTime_).count();
//}
//
//void QglEngine::bindFramebuffer_(Framebuffer* framebuffer)
//{
//    QOpenglFramebuffer* fb = static_cast<QOpenglFramebuffer*>(framebuffer);
//    GLuint fbo = fb->isDefault_ ? ctx_->defaultFramebufferObject() : fb->object_;
//    api_->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
//}
//
//void QglEngine::setViewport_(Int x, Int y, Int width, Int height)
//{
//    api_->glViewport(x, y, width, height);
//}
//
//void QglEngine::clear_(const core::Color& color)
//{
//    api_->glClearColor(
//        static_cast<float>(color.r()),
//        static_cast<float>(color.g()),
//        static_cast<float>(color.b()),
//        static_cast<float>(color.a()));
//    api_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//}
//
//void QglEngine::setProjectionMatrix_(const geometry::Mat4f& m)
//{
//    paintShaderProgram_->setUniformValue(projLoc_, toQtMatrix(m));
//}
//
//void QglEngine::setViewMatrix_(const geometry::Mat4f& m)
//{
//    paintShaderProgram_->setUniformValue(viewLoc_, toQtMatrix(m));
//}
//
//void QglEngine::initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes)
//{
//    QOpenglBuffer* oglBuffer = static_cast<QOpenglBuffer*>(buffer);
//    oglBuffer->init(data, initialLengthInBytes);
//}
//
//void QglEngine::updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes)
//{
//    QOpenglBuffer* oglBuffer = static_cast<QOpenglBuffer*>(buffer);
//    oglBuffer->load(data, lengthInBytes);
//}
//
//void QglEngine::setupVertexBufferForPaintShader_(Buffer* buffer)
//{
//    QOpenglBuffer* oglBuffer = static_cast<QOpenglBuffer*>(buffer);
//    GLsizei stride = sizeof(XYRGBVertex);
//    GLvoid* posPointer = reinterpret_cast<void*>(offsetof(XYRGBVertex, x));
//    GLvoid* colPointer = reinterpret_cast<void*>(offsetof(XYRGBVertex, r));
//    GLboolean normalized = GL_FALSE;
//    oglBuffer->bind();
//    api_->glEnableVertexAttribArray(posLoc_);
//    api_->glEnableVertexAttribArray(colLoc_);
//    api_->glVertexAttribPointer(posLoc_, 2, GL_FLOAT, normalized, stride, posPointer);
//    api_->glVertexAttribPointer(colLoc_, 3, GL_FLOAT, normalized, stride, colPointer);
//    oglBuffer->unbind();
//}
//
//void QglEngine::drawPrimitives_(Buffer* buffer, PrimitiveType type)
//{
//    QOpenglBuffer* oglBuffer = static_cast<QOpenglBuffer*>(buffer);
//    GLenum mode = 0;
//    switch (type) {
//    case PrimitiveType::LineList: mode = GL_LINES; break;
//    case PrimitiveType::LineStrip: mode = GL_LINE_STRIP; break;
//    case PrimitiveType::TriangleList: mode = GL_TRIANGLES; break;
//    case PrimitiveType::TriangleStrip: mode = GL_TRIANGLE_STRIP; break;
//    default:
//        mode = GL_POINTS;
//        break;
//    }
//    oglBuffer->draw(this, mode);
//}
//
//void QglEngine::bindPaintShader_()
//{
//    paintShaderProgram_->bind();
//}
//
//void QglEngine::releasePaintShader_()
//{
//    paintShaderProgram_->release();
//}

} // namespace vgc::ui::internal::qopengl
