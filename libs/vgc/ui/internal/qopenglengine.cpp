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
#include <limits>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QOpenGLVersionFunctionsFactory>
#endif
#include <QWindow>

#include <vgc/core/exceptions.h>
#include <vgc/core/paths.h>
#include <vgc/ui/logcategories.h>
#include <vgc/ui/qtutil.h>

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

inline constexpr GLuint nullGLObject = 0;
inline constexpr GLuint badGLObject = std::numeric_limits<GLuint>::max();
inline constexpr GLenum badGLenum = std::numeric_limits<GLenum>::max();

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

    GLenum usageGL() const
    {
        return usageGL_;
    }

protected:
    void release_(Engine* engine) override
    {
        Buffer::release_(engine);
        static_cast<QglEngine*>(engine)->api()->glDeleteBuffers(1, &object_);
    }

private:
    GLuint object_ = badGLObject;
    GLenum usageGL_ = badGLenum;
};
using QglBufferPtr = ResourcePtr<QglBuffer>;

class QglSamplerState : public SamplerState {
protected:
    friend QglEngine;
    using SamplerState::SamplerState;

    bool isEquivalentTo(const QglSamplerState& other) const {
        if (maxAnisotropyGL_ > 1.f && other.maxAnisotropyGL_ > 1.f) {
            if (maxAnisotropyGL_ != other.maxAnisotropyGL_) return false;
        } else {
            if (magFilterGL_ != other.magFilterGL_) return false;
            if (minFilterGL_ != other.minFilterGL_) return false;
            if (mipFilterGL_ != other.mipFilterGL_) return false;
        }
        if (wrapS_ != other.wrapS_) return false;
        if (wrapT_ != other.wrapT_) return false;
        if (wrapR_ != other.wrapR_) return false;
        if (comparisonFunctionGL_ != other.comparisonFunctionGL_) return false;
        return true;
    }

private:
    float maxAnisotropyGL_ = 0.f;
    GLenum magFilterGL_ = badGLenum;
    GLenum minFilterGL_ = badGLenum;
    GLenum mipFilterGL_ = badGLenum;
    GLenum wrapS_ = badGLenum;
    GLenum wrapT_ = badGLenum;
    GLenum wrapR_ = badGLenum;
    GLenum comparisonFunctionGL_ = badGLenum;
};
using QglSamplerStatePtr = ResourcePtr<QglSamplerState>;

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
    void releaseSubResources_() override
    {
        Image:releaseSubResources_();
        samplerState_.reset();
    }

    void release_(Engine* engine) override
    {
        Image::release_(engine);
        static_cast<QglEngine*>(engine)->api()->glDeleteTextures(1, &object_);
    }

private:
    GLuint object_ = badGLObject;
    GLint internalFormat_ = 0;

    friend QglImageView;
    QglSamplerStatePtr samplerState_;
};
using QglImagePtr = ResourcePtr<QglImage>;

class QglImageView : public ImageView {
protected:
    friend QglEngine;

    QglImageView(ResourceRegistry* registry,
                 const ImageViewCreateInfo& createInfo,
                 const ImagePtr& image)
        : ImageView(registry, createInfo, image) {

        samplerStatePtrAddress_ = &image.get_static_cast<QglImage>()->samplerState_;
    }

    QglImageView(ResourceRegistry* registry,
                 const ImageViewCreateInfo& createInfo,
                 const BufferPtr& buffer,
                 ImageFormat format,
                 UInt32 numBufferElements)
        : ImageView(registry, createInfo, buffer, format, numBufferElements) {

        samplerStatePtrAddress_ = &viewSamplerState_;
    }

public:
    GLuint internalFormat() const
    {
        return internalFormat_;
    }

    GLuint object() const
    {
        QglImage* image = viewedImage().get_static_cast<QglImage>();
        return image ? image->object() : bufferTextureObject_;
    }

protected:
    void releaseSubResources_() override
    {
        ImageView::releaseSubResources_();
        viewSamplerState_.reset();
    }

    void release_(Engine* engine) override
    {
        ImageView::release_(engine);
        viewSamplerState_.reset();
    }

private:
    GLuint bufferTextureObject_ = badGLObject;
    GLuint internalFormat_ = 0;
    QglSamplerStatePtr viewSamplerState_;
    QglSamplerStatePtr* samplerStatePtrAddress_ = nullptr;
};
using QglImageViewPtr = ResourcePtr<QglImageView>;

class QglGeometryView : public GeometryView {
protected:
    friend QglEngine;

    QglGeometryView(ResourceRegistry* registry, const GeometryViewCreateInfo& info)
        : GeometryView(registry, info) {
        std::fill(builtinLayoutVAOs_.begin(), builtinLayoutVAOs_.end(), nullGLObject);
    }

public:
    GLenum topology() const {
        return topology_;
    }

protected:
    void release_(Engine* engine) override
    {
        GeometryView::release_(engine);
        static_cast<QglEngine*>(engine)->api()->glDeleteVertexArrays(
            numBuiltinGeometryLayouts, builtinLayoutVAOs_.data());
    }

private:
    GLenum topology_;

    std::array<GLuint, numBuiltinGeometryLayouts> builtinLayoutVAOs_;
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

struct BlendEquationGL {
    GLenum operation = badGLenum;
    GLenum sourceFactor = badGLenum;
    GLenum targetFactor = badGLenum;
};

struct TargetBlendStateGL {
    bool isEnabled = false;
    BlendEquationGL equationRGB = {};
    BlendEquationGL equationAlpha = {};
    BlendWriteMask writeMask = BlendWriteMaskBit::All;
};

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
    std::array<TargetBlendStateGL, maxColorTargets> targetBlendStatesGL_ = {};
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
    GLenum fillModeGL_ = badGLenum;
    GLenum cullModeGL_ = badGLenum;
};
using QglRasterizerStatePtr = ResourcePtr<QglRasterizerState>;


// no equivalent in D3D11, see OMSetRenderTargets
class QglFramebuffer : public Framebuffer {
protected:
    friend QglEngine;

    using Framebuffer::Framebuffer;

public:
    bool isDefault() const
    {
        return isDefault_;
    }

    GLuint object() const
    {
        return object_;
    }

protected:
    void releaseSubResources_() override
    {
        colorView_.reset();
        depthStencilView_.reset();
    }

    void release_(Engine* engine) override
    {
        Framebuffer::release_(engine);
        static_cast<QglEngine*>(engine)->api()->glDeleteFramebuffers(1, &object_);
    }

private:
    GLuint object_ = badGLObject;
    QglImageViewPtr colorView_;
    QglImageViewPtr depthStencilView_;

    bool isDefault_ = false;
};
using QglFramebufferPtr = ResourcePtr<QglFramebuffer>;

class QglSwapChain : public SwapChain {
protected:
    friend QglEngine;

    QglSwapChain(ResourceRegistry* registry,
                 const SwapChainCreateInfo& desc,
                 const FramebufferPtr& framebuffer)
        : SwapChain(registry, desc, framebuffer) {
    }

protected:
    void release_(Engine* engine) override
    {
        SwapChain::release_(engine);
    }

private:
    bool isExternal_ = false;
    QSurface* surface_ = nullptr;
    QWindow* window_ = nullptr;
};

// ENUM CONVERSIONS

GLenum imageFormatToGLenum(ImageFormat format)
{
    switch (format) {
        // Depth
    case ImageFormat::D_16_UNORM:               return GL_DEPTH_COMPONENT16;
    case ImageFormat::D_32_FLOAT:               return GL_DEPTH_COMPONENT32F;
        // Depth + Stencil
    case ImageFormat::DS_24_UNORM_8_UINT:       return GL_DEPTH24_STENCIL8;
    case ImageFormat::DS_32_FLOAT_8_UINT_24_X:  return GL_DEPTH32F_STENCIL8;
        // Red
    case ImageFormat::R_8_UNORM:                return GL_R8;
    case ImageFormat::R_8_SNORM:                return GL_R8_SNORM;
    case ImageFormat::R_8_UINT:                 return GL_R8UI;
    case ImageFormat::R_8_SINT:                 return GL_R8I;
    case ImageFormat::R_16_UNORM:               return GL_R16;
    case ImageFormat::R_16_SNORM:               return GL_R16_SNORM;
    case ImageFormat::R_16_UINT:                return GL_R16UI;
    case ImageFormat::R_16_SINT:                return GL_R16I;
    case ImageFormat::R_16_FLOAT:               return GL_R16F;
    case ImageFormat::R_32_UINT:                return GL_R32UI;
    case ImageFormat::R_32_SINT:                return GL_R32I;
    case ImageFormat::R_32_FLOAT:               return GL_R32F;
        // RG
    case ImageFormat::RG_8_UNORM:               return GL_RG8;
    case ImageFormat::RG_8_SNORM:               return GL_RG8_SNORM;
    case ImageFormat::RG_8_UINT:                return GL_RG8UI;
    case ImageFormat::RG_8_SINT:                return GL_RG8I;
    case ImageFormat::RG_16_UNORM:              return GL_RG16;
    case ImageFormat::RG_16_SNORM:              return GL_RG16_SNORM;
    case ImageFormat::RG_16_UINT:               return GL_RG16UI;
    case ImageFormat::RG_16_SINT:               return GL_RG16I;
    case ImageFormat::RG_16_FLOAT:              return GL_RG16F;
    case ImageFormat::RG_32_UINT:               return GL_RG32UI;
    case ImageFormat::RG_32_SINT:               return GL_RG32I;
    case ImageFormat::RG_32_FLOAT:              return GL_RG32F;
        // RGB
    case ImageFormat::RGB_11_11_10_FLOAT:       return GL_R11F_G11F_B10F;
    case ImageFormat::RGB_32_UINT:              return GL_RGB32UI;
    case ImageFormat::RGB_32_SINT:              return GL_RGB32I;
    case ImageFormat::RGB_32_FLOAT:             return GL_RGB32F;
        // RGBA
    case ImageFormat::RGBA_8_UNORM:             return GL_RGBA8;
    case ImageFormat::RGBA_8_UNORM_SRGB:        return GL_SRGB8_ALPHA8;
    case ImageFormat::RGBA_8_SNORM:             return GL_RGBA8_SNORM;
    case ImageFormat::RGBA_8_UINT:              return GL_RGBA8UI;
    case ImageFormat::RGBA_8_SINT:              return GL_RGBA8I;
    case ImageFormat::RGBA_10_10_10_2_UNORM:    return GL_RGB10_A2;
    case ImageFormat::RGBA_10_10_10_2_UINT:     return GL_RGB10_A2UI;
    case ImageFormat::RGBA_16_UNORM:            return GL_RGBA16;
    case ImageFormat::RGBA_16_UINT:             return GL_RGBA16UI;
    case ImageFormat::RGBA_16_SINT:             return GL_RGBA16I;
    case ImageFormat::RGBA_16_FLOAT:            return GL_RGBA16F;
    case ImageFormat::RGBA_32_UINT:             return GL_RGBA32UI;
    case ImageFormat::RGBA_32_SINT:             return GL_RGBA32I;
    case ImageFormat::RGBA_32_FLOAT:            return GL_RGBA32F;
    default:
        break;
    }
    return 0;
}

GLenum primitiveTypeToGLenum(PrimitiveType type)
{
    static_assert(numPrimitiveTypes == 6);
    static constexpr std::array<GLenum, numPrimitiveTypes> map = {
        badGLenum,                      // Undefined,
        GL_POINTS,                      // Point,
        GL_LINES,                       // LineList,
        GL_LINE_STRIP,                  // LineStrip,
        GL_TRIANGLES,                   // TriangleList,
        GL_TRIANGLE_STRIP,              // TriangleStrip,
    };

    const UInt index = core::toUnderlying(type);
    if (index == 0 || index >= numPrimitiveTypes) {
        throw core::LogicError("QglEngine: invalid PrimitiveType enum value");
    }

    return map[index];
}

GLenum usageToGLenum(Usage usage, CpuAccessFlags cpuAccessFlags)
{
    switch (usage) {
    case Usage::Default:
        return GL_DYNAMIC_DRAW;
    case Usage::Immutable:
        return GL_STATIC_DRAW;
    case Usage::Dynamic:
        return GL_STREAM_DRAW;
    case Usage::Staging: {
        if (cpuAccessFlags & CpuAccessFlag::Read) {
            if (cpuAccessFlags & CpuAccessFlag::Write) {
                throw core::LogicError("Qgl: staging buffer cannot habe both read and write cpu access.");
            }
            return GL_STATIC_READ;
        } else if (cpuAccessFlags & CpuAccessFlag::Write) {
            return GL_STATIC_COPY;
        }
        throw core::LogicError("Qgl: staging buffer needs either read and write cpu access");
    }
    default:
        break;
    }
    throw core::LogicError("QglEngine: unsupported usage");
}

UINT processResourceMiscFlags(ResourceMiscFlags resourceMiscFlags)
{
    UINT x = 0;
    if (resourceMiscFlags & ResourceMiscFlag::Shared) {
        throw core::LogicError("QglEngine: ResourceMiscFlags::Shared is not supported at the moment");
    }
    //if (resourceMiscFlags & ResourceMiscFlag::TextureCube) {
    //    throw core::LogicError("QglEngine: ResourceMiscFlags::TextureCube is not supported at the moment");
    //}
    //if (resourceMiscFlags & ResourceMiscFlag::ResourceClamp) {
    //    throw core::LogicError("QglEngine: ResourceMiscFlags::ResourceClamp is not supported at the moment");
    //}
    return x;
}

GLenum imageWrapModeToGLenum(ImageWrapMode mode)
{
    static_assert(numImageWrapModes == 5);
    static constexpr std::array<GLenum, numImageWrapModes> map = {
        badGLenum,                      // Undefined
        GL_REPEAT,                      // Repeat
        GL_MIRRORED_REPEAT,             // MirrorRepeat
        GL_CLAMP_TO_EDGE,               // Clamp
        GL_CLAMP_TO_BORDER,             // ClampConstantColor
    };

    const UInt index = core::toUnderlying(mode);
    if (index == 0 || index >= numImageWrapModes) {
        throw core::LogicError("QglEngine: invalid ImageWrapMode enum value");
    }

    return map[index];
}

GLenum comparisonFunctionToGLenum(ComparisonFunction func)
{
    static_assert(numComparisonFunctions == 10);
    static constexpr std::array<GLenum, numComparisonFunctions> map = {
        badGLenum,                      // Undefined
        GL_NEVER,                       // Disabled
        GL_ALWAYS,                      // Always
        GL_NEVER,                       // Never
        GL_EQUAL,                       // Equal
        GL_NOTEQUAL,                    // NotEqual
        GL_LESS,                        // Less
        GL_LEQUAL,                      // LessEqual
        GL_GREATER,                     // Greater
        GL_GEQUAL,                      // GreaterEqual
    };

    const UInt index = core::toUnderlying(func);
    if (index == 0 || index >= numComparisonFunctions) {
        throw core::LogicError("QglEngine: invalid ComparisonFunction enum value");
    }

    return map[index];
}

GLenum blendFactorToGLenum(BlendFactor factor)
{
    static_assert(numBlendFactors == 18);
    static constexpr std::array<GLenum, numBlendFactors> map = {
        badGLenum,                      // Undefined
        GL_ONE,                         // One
        GL_ZERO,                        // Zero
        GL_SRC_COLOR,                   // SourceColor
        GL_ONE_MINUS_SRC_COLOR,         // OneMinusSourceColor
        GL_SRC_ALPHA,                   // SourceAlpha
        GL_ONE_MINUS_SRC_ALPHA,         // OneMinusSourceAlpha
        GL_DST_COLOR,                   // TargetColor
        GL_ONE_MINUS_DST_COLOR,         // OneMinusTargetColor
        GL_DST_ALPHA,                   // TargetAlpha
        GL_ONE_MINUS_DST_ALPHA,         // OneMinusTargetAlpha
        GL_SRC_ALPHA_SATURATE,          // SourceAlphaSaturated
        GL_CONSTANT_COLOR,              // Constant
        GL_ONE_MINUS_CONSTANT_COLOR,    // OneMinusConstant
        GL_SRC1_COLOR,                  // SecondSourceColor
        GL_ONE_MINUS_SRC1_COLOR,        // OneMinusSecondSourceColor
        GL_SRC1_ALPHA,                  // SecondSourceAlpha
        GL_ONE_MINUS_SRC1_ALPHA,        // OneMinusSecondSourceAlpha
    };

    const UInt index = core::toUnderlying(factor);
    if (index == 0 || index >= numBlendFactors) {
        throw core::LogicError("QglEngine: invalid BlendFactor enum value");
    }

    return map[index];
}

GLenum blendOpToGLenum(BlendOp op)
{
    static_assert(numBlendOps == 6);
    static constexpr std::array<GLenum, numBlendOps> map = {
        badGLenum,                      // Undefined
        GL_FUNC_ADD,                    // Add
        GL_FUNC_SUBTRACT,               // SourceMinusTarget
        GL_FUNC_REVERSE_SUBTRACT,       // TargetMinusSource
        GL_MIN,                         // Min
        GL_MAX,                         // Max
    };

    const UInt index = core::toUnderlying(op);
    if (index == 0 || index >= numBlendOps) {
        throw core::LogicError("QglEngine: invalid BlendOp enum value");
    }

    return map[index];
}

GLenum fillModeToGLenum(FillMode mode)
{
    static_assert(numFillModes == 3);
    static constexpr std::array<GLenum, numFillModes> map = {
        badGLenum,                      // Undefined
        GL_FILL,                        // Solid
        GL_LINE,                        // Wireframe
    };

    const UInt index = core::toUnderlying(mode);
    if (index == 0 || index >= numFillModes) {
        throw core::LogicError("QglEngine: invalid FillMode enum value");
    }

    return map[index];
}

GLenum cullModeToGLenum(CullMode mode)
{
    static_assert(numCullModes == 4);
    static constexpr std::array<GLenum, numCullModes> map = {
        badGLenum,              // Undefined
        GL_FRONT_AND_BACK,      // None -> must disable culling
        GL_FRONT,               // Front
        GL_BACK,                // Back
    };

    const UInt index = core::toUnderlying(mode);
    if (index == 0 || index >= numCullModes) {
        throw core::LogicError("QglEngine: invalid CullMode enum value");
    }

    return map[index];
}

GLenum filterModeToGLenum(FilterMode mode)
{
    static_assert(numFilterModes == 3);
    static constexpr std::array<GLenum, numFilterModes> map = {
        badGLenum,              // Undefined
        GL_NEAREST,             // Point
        GL_LINEAR,              // Linear
    };

    const UInt index = core::toUnderlying(mode);
    if (index == 0 || index >= numFilterModes) {
        throw core::LogicError("QglEngine: invalid FilterMode enum value");
    }

    return map[index];
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
//            bool cpuWrites = cpuAccessFlags() & CpuAccessFlag::Write;
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

QglEngine::QglEngine(QOpenGLContext* ctx, bool isExternalCtx) :
    Engine(),
    ctx_(ctx),
    isExternalCtx_(isExternalCtx)
{
    offscreenSurface_ = new QOffscreenSurface();
    QSurface* surface = ctx_->surface();
    surface = surface ? surface : offscreenSurface_;
    ctx_->makeCurrent(surface);
    hasAnisotropicFilteringSupport_ = ctx_->hasExtension("EXT_texture_filter_anisotropic");

    createBuiltinResources_();
}

void QglEngine::onDestroyed()
{
    Engine::onDestroyed();
    if (!isExternalCtx_) {
        delete ctx_;
    }
    ctx_ = nullptr;
    delete offscreenSurface_;
    offscreenSurface_ = nullptr;
    surface_ = nullptr;
}

/* static */
QglEnginePtr QglEngine::create()
{
    QOpenGLContext* ctx = new QOpenGLContext();
    ctx->create();
    return QglEnginePtr(new QglEngine(ctx, false));
}

/* static */
QglEnginePtr QglEngine::create(QOpenGLContext* ctx)
{
    return QglEnginePtr(new QglEngine(ctx, true));
}

SwapChainPtr QglEngine::createSwapChainFromSurface(QSurface* surface)
{
    SwapChainCreateInfo createInfo = {};
    // XXX fill from surface format

    QglFramebufferPtr framebuffer(new QglFramebuffer(resourceRegistry_));
    framebuffer->isDefault_ = true;
    framebuffer->object_ = 0;

    auto swapChain = makeUnique<QglSwapChain>(resourceRegistry_, createInfo, framebuffer);
    swapChain->window_ = nullptr;
    swapChain->surface_ = surface;
    swapChain->isExternal_ = true;

    return SwapChainPtr(swapChain.release());
}

void QglEngine::makeCurrent()
{
    queueLambdaCommand_(
        "makeCurrent",
        [=](Engine* engine) {
            static_cast<QglEngine*>(engine)->makeCurrent_();
        });
}

// -- USER THREAD implementation functions --

void QglEngine::createBuiltinShaders_()
{
    queueLambdaCommand_(
        "initBuiltinShaders",
        [=](Engine* engine) {
            static_cast<QglEngine*>(engine)->initBuiltinShaders_();
        });
}

SwapChainPtr QglEngine::constructSwapChain_(const SwapChainCreateInfo& createInfo)
{
    if (createInfo.windowNativeHandleType() != WindowNativeHandleType::QOpenGLWindow) {
        throw core::LogicError("QglEngine: unsupported WindowNativeHandleType value.");
    }

    if (ctx_ == nullptr) {
        throw core::LogicError("ctx_ is null.");
    }
    // XXX can it be an external context ??

    // XXX only allow D24_S8 for now..
    format_.setDepthBufferSize(24);
    format_.setStencilBufferSize(8);
    format_.setVersion(3, 3);
    format_.setProfile(QSurfaceFormat::CoreProfile);
    format_.setSamples(createInfo.numSamples());
    format_.setSwapInterval(0);

    QWindow* wnd = static_cast<QWindow*>(createInfo.windowNativeHandle());
    wnd->setFormat(format_);
    wnd->create();

    QglFramebufferPtr framebuffer(new QglFramebuffer(resourceRegistry_));
    framebuffer->isDefault_ = true;
    framebuffer->object_ = 0;

    auto swapChain = makeUnique<QglSwapChain>(resourceRegistry_, createInfo, framebuffer);
    swapChain->window_ = wnd;
    swapChain->surface_ = wnd;
    swapChain->isExternal_ = false;

    return SwapChainPtr(swapChain.release());
}

FramebufferPtr QglEngine::constructFramebuffer_(const ImageViewPtr& colorImageView)
{
    auto framebuffer = makeUnique<QglFramebuffer>(resourceRegistry_);
    framebuffer->colorView_ = static_pointer_cast<QglImageView>(colorImageView);

    return FramebufferPtr(framebuffer.release());
}

BufferPtr QglEngine::constructBuffer_(const BufferCreateInfo& createInfo)
{
    auto buffer = makeUnique<QglBuffer>(resourceRegistry_, createInfo);
    buffer->usageGL_ = usageToGLenum(createInfo.usage(), createInfo.cpuAccessFlags());

    return BufferPtr(buffer.release());
}

ImagePtr QglEngine::constructImage_(const ImageCreateInfo& createInfo)
{
    auto image = makeUnique<QglImage>(resourceRegistry_, createInfo);
    image->internalFormat_ = imageFormatToGLenum(createInfo.format());

    return ImagePtr(image.release());
}

ImageViewPtr QglEngine::constructImageView_(const ImageViewCreateInfo& createInfo, const ImagePtr& image)
{
    auto view = makeUnique<QglImageView>(resourceRegistry_, createInfo, image);
    view->internalFormat_ = image.get_static_cast<QglImage>()->internalFormat();

    return ImageViewPtr(view.release());
}

ImageViewPtr QglEngine::constructImageView_(const ImageViewCreateInfo& createInfo, const BufferPtr& buffer, ImageFormat format, UInt32 numElements)
{
    auto view = makeUnique<QglImageView>(resourceRegistry_, createInfo, buffer, format, numElements);
    view->internalFormat_ = imageFormatToGLenum(format);

    return ImageViewPtr(view.release());
}

SamplerStatePtr QglEngine::constructSamplerState_(const SamplerStateCreateInfo& createInfo)
{
    auto state = makeUnique<QglSamplerState>(resourceRegistry_, createInfo);
    state->magFilterGL_ = filterModeToGLenum(createInfo.magFilter());
    state->minFilterGL_ = filterModeToGLenum(createInfo.minFilter());
    state->mipFilterGL_ = filterModeToGLenum(createInfo.mipFilter());
    if (createInfo.maxAnisotropy() >= 1) {
        if (hasAnisotropicFilteringSupport_) {
            state->maxAnisotropyGL_ = static_cast<float>(createInfo.maxAnisotropy());
        } else {
            VGC_WARNING(LogVgcUi, "Anisotropic filtering is not supported.");
        }
    }
    state->wrapS_ = imageWrapModeToGLenum(createInfo.wrapModeU());
    state->wrapT_ = imageWrapModeToGLenum(createInfo.wrapModeV());
    state->wrapR_ = imageWrapModeToGLenum(createInfo.wrapModeW());
    state->comparisonFunctionGL_ = comparisonFunctionToGLenum(createInfo.comparisonFunction());

    return SamplerStatePtr(state.release());
}

GeometryViewPtr QglEngine::constructGeometryView_(const GeometryViewCreateInfo& createInfo)
{
    auto view = makeUnique<QglGeometryView>(resourceRegistry_, createInfo);
    view->topology_ = primitiveTypeToGLenum(createInfo.primitiveType());

    return GeometryViewPtr(view.release());
}

BlendStatePtr QglEngine::constructBlendState_(const BlendStateCreateInfo& createInfo)
{
    auto state = makeUnique<QglBlendState>(resourceRegistry_, createInfo);
    Int n = state->isIndependentBlendEnabled() ? maxColorTargets : 1;
    for (Int i = 0; i < n; ++i) {
        const TargetBlendState& blendState = state->targetBlendState(i);
        TargetBlendStateGL& blendStateGL = state->targetBlendStatesGL_[i];
        blendStateGL.isEnabled = blendState.isEnabled();
        if (blendStateGL.isEnabled) {
            blendStateGL.writeMask = blendState.writeMask();
            const BlendEquation& equationRGB = blendState.equationRGB();
            blendStateGL.equationRGB.operation = blendOpToGLenum(equationRGB.operation());
            blendStateGL.equationRGB.sourceFactor = blendFactorToGLenum(equationRGB.sourceFactor());
            blendStateGL.equationRGB.targetFactor = blendFactorToGLenum(equationRGB.targetFactor());
            const BlendEquation& equationAlpha = blendState.equationAlpha();
            blendStateGL.equationAlpha.operation = blendOpToGLenum(equationAlpha.operation());
            blendStateGL.equationAlpha.sourceFactor = blendFactorToGLenum(equationAlpha.sourceFactor());
            blendStateGL.equationAlpha.targetFactor = blendFactorToGLenum(equationAlpha.targetFactor());
        }
    }

    return BlendStatePtr(state.release());
}

RasterizerStatePtr QglEngine::constructRasterizerState_(const RasterizerStateCreateInfo& createInfo)
{
    auto state = makeUnique<QglRasterizerState>(resourceRegistry_, createInfo);
    state->fillModeGL_ = fillModeToGLenum(createInfo.fillMode());
    state->cullModeGL_ = cullModeToGLenum(createInfo.cullMode());

    return RasterizerStatePtr(state.release());
}

void QglEngine::resizeSwapChain_(SwapChain* /*swapChain*/, UInt32 /*width*/, UInt32 /*height*/)
{
    // XXX anything to do ?
}

//--  RENDER THREAD implementation functions --

void QglEngine::onStart_()
{
    ctx_->makeCurrent(offscreenSurface_);

    // Get API 3.3
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    api_ = ctx_->versionFunctions<QOpenGLFunctions_3_3_Core>();
#else
    api_ = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(ctx_);
#endif
    //

    api_->initializeOpenGLFunctions();

    initBuiltinShaders_();
}

void QglEngine::initFramebuffer_(Framebuffer* framebuffer)
{
    QglFramebuffer* glFramebuffer = static_cast<QglFramebuffer*>(framebuffer);
    api_->glGenFramebuffers(1, &glFramebuffer->object_);
    api_->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, glFramebuffer->object_);
    // XXX handle the different textargets + ranks + layer !!
    api_->glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, glFramebuffer->colorView_->object(), 0);
    api_->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer_);
}

void QglEngine::initBuffer_(Buffer* buffer, const char* /*data*/, Int /*lengthInBytes*/)
{
    QglBuffer* glBuffer = static_cast<QglBuffer*>(buffer);
    api_->glGenBuffers(1, &glBuffer->object_);
    // ...
}

void QglEngine::initImage_(Image* /*image*/, const Span<const Span<const char>>* /*dataSpanSpan*/)
{
}

void QglEngine::initImageView_(ImageView* /*view*/)
{
}

void QglEngine::initSamplerState_(SamplerState* /*state*/)
{
}

void QglEngine::initGeometryView_(GeometryView* /*view*/)
{
}

void QglEngine::initBlendState_(BlendState* /*state*/)
{
}

void QglEngine::initRasterizerState_(RasterizerState* /*state*/)
{
}

void QglEngine::setSwapChain_(const SwapChainPtr& swapChain)
{
    if (swapChain) {
        surface_ = swapChain.get_static_cast<QglSwapChain>()->surface_;
    } else {
        surface_ = offscreenSurface_;
    }
    ctx_->makeCurrent(surface_);
}

void QglEngine::setFramebuffer_(const FramebufferPtr& /*framebuffer*/)
{
    //framebuffer_
}

void QglEngine::setViewport_(Int /*x*/, Int /*y*/, Int /*width*/, Int /*height*/)
{
}

void QglEngine::setProgram_(const ProgramPtr& /*program*/)
{
}

void QglEngine::setBlendState_(const BlendStatePtr& /*state*/, const geometry::Vec4f& /*blendFactor*/)
{
}

void QglEngine::setRasterizerState_(const RasterizerStatePtr& /*state*/)
{
}

void QglEngine::setStageConstantBuffers_(BufferPtr const* /*buffers*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/)
{
}

void QglEngine::setStageImageViews_(ImageViewPtr const* /*views*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/)
{
}

void QglEngine::setStageSamplers_(SamplerStatePtr const* /*states*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/)
{
}

void QglEngine::updateBufferData_(Buffer* /*buffer*/, const void* /*data*/, Int /*lengthInBytes*/)
{
}

// should do init at beginFrame if needed..

void QglEngine::draw_(GeometryView* /*view*/, UInt /*numIndices*/, UInt /*numInstances*/)
{
}

void QglEngine::clear_(const core::Color& /*color*/)
{
}

UInt64 QglEngine::present_(SwapChain* /*swapChain*/, UInt32 /*syncInterval*/, PresentFlags /*flags*/)
{
    return 0;
}

// Private methods

void QglEngine::initBuiltinShaders_()
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
}

void QglEngine::makeCurrent_()
{
    ctx_->makeCurrent(ctx_->surface());
}

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
