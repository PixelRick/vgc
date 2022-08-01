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
//    void draw(QOpenglEngine* engine, GLenum mode)
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
//        auto oglApi = static_cast<QOpenglEngine*>(engine)->api();
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
//    friend QOpenglEngine;
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
//            static_cast<QOpenglEngine*>(engine)->api()->glDeleteFramebuffers(1, &object_);
//        }
//    }
//
//private:
//    friend QOpenglEngine;
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

QOpenglEngine::QOpenglEngine() :
    QOpenglEngine(nullptr, false)
{
}

QOpenglEngine::QOpenglEngine(QOpenGLContext* ctx, bool isExternalCtx) :
    Engine(),
    ctx_(ctx),
    isExternalCtx_(isExternalCtx)
{
    startTime_ = std::chrono::steady_clock::now();
}

void QOpenglEngine::onDestroyed()
{
    Engine::onDestroyed();
    if (!isExternalCtx_) {
        delete ctx_;
    }
}

/* static */
QOpenglEnginePtr QOpenglEngine::create()
{
    return QOpenglEnginePtr(new QOpenglEngine());
}

/* static */
QOpenglEnginePtr QOpenglEngine::create(QOpenGLContext* ctx)
{
    return QOpenglEnginePtr(new QOpenglEngine(ctx));
}

void QOpenglEngine::setupContext()
{
    // Initialize shader program
    paintShaderProgram_.reset(new QOpenGLShaderProgram());
    paintShaderProgram_->addShaderFromSourceFile(QOpenGLShader::Vertex, shaderPath_("iv4pos_iv4col_um4proj_um4view_ov4fcol.v.glsl"));
    paintShaderProgram_->addShaderFromSourceFile(QOpenGLShader::Fragment, shaderPath_("iv4fcol.f.glsl"));
    paintShaderProgram_->link();

    // Get shader locations
    paintShaderProgram_->bind();
    posLoc_  = paintShaderProgram_->attributeLocation("pos");
    colLoc_  = paintShaderProgram_->attributeLocation("col");
    projLoc_ = paintShaderProgram_->uniformLocation("proj");
    viewLoc_ = paintShaderProgram_->uniformLocation("view");
    paintShaderProgram_->release();

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

void QOpenglEngine::createBuiltinShaders_() {}

SwapChainPtr QOpenglEngine::constructSwapChain_(const SwapChainCreateInfo& /*createInfo*/) { return nullptr; }
FramebufferPtr QOpenglEngine::constructFramebuffer_(const ImageViewPtr& /*colorImageView*/) { return nullptr; }
BufferPtr QOpenglEngine::constructBuffer_(const BufferCreateInfo& /*createInfo*/) { return nullptr; }
ImagePtr QOpenglEngine::constructImage_(const ImageCreateInfo& /*createInfo*/) { return nullptr; }
ImageViewPtr QOpenglEngine::constructImageView_(const ImageViewCreateInfo& /*createInfo*/, const ImagePtr& /*image*/) { return nullptr; }
ImageViewPtr QOpenglEngine::constructImageView_(const ImageViewCreateInfo& /*createInfo*/, const BufferPtr& /*buffer*/, ImageFormat /*format*/, UInt32 /*numElements*/) { return nullptr; }
SamplerStatePtr QOpenglEngine::constructSamplerState_(const SamplerStateCreateInfo& /*createInfo*/) { return nullptr; }
GeometryViewPtr QOpenglEngine::constructGeometryView_(const GeometryViewCreateInfo& /*createInfo*/) { return nullptr; }
BlendStatePtr QOpenglEngine::constructBlendState_(const BlendStateCreateInfo& /*createInfo*/) { return nullptr; }
RasterizerStatePtr QOpenglEngine::constructRasterizerState_(const RasterizerStateCreateInfo& /*createInfo*/) { return nullptr; }

void QOpenglEngine::resizeSwapChain_(SwapChain* /*swapChain*/, UInt32 /*width*/, UInt32 /*height*/) {}

//--  RENDER THREAD implementation functions --

void QOpenglEngine::initFramebuffer_(Framebuffer* /*framebuffer*/) {}
void QOpenglEngine::initBuffer_(Buffer* /*buffer*/, const char* /*data*/, Int /*lengthInBytes*/) {}
void QOpenglEngine::initImage_(Image* /*image*/, const Span<const Span<const char>>* /*dataSpanSpan*/) {}
void QOpenglEngine::initImageView_(ImageView* /*view*/) {}
void QOpenglEngine::initSamplerState_(SamplerState* /*state*/) {}
void QOpenglEngine::initGeometryView_(GeometryView* /*view*/) {}
void QOpenglEngine::initBlendState_(BlendState* /*state*/) {}
void QOpenglEngine::initRasterizerState_(RasterizerState* /*state*/) {}

void QOpenglEngine::setSwapChain_(const SwapChainPtr& /*swapChain*/) {}
void QOpenglEngine::setFramebuffer_(const FramebufferPtr& /*framebuffer*/) {}
void QOpenglEngine::setViewport_(Int /*x*/, Int /*y*/, Int /*width*/, Int /*height*/) {}
void QOpenglEngine::setProgram_(const ProgramPtr& /*program*/) {}
void QOpenglEngine::setBlendState_(const BlendStatePtr& /*state*/, const geometry::Vec4f& /*blendFactor*/) {}
void QOpenglEngine::setRasterizerState_(const RasterizerStatePtr& /*state*/) {}
void QOpenglEngine::setStageConstantBuffers_(BufferPtr const* /*buffers*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/) {}
void QOpenglEngine::setStageImageViews_(ImageViewPtr const* /*views*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/) {}
void QOpenglEngine::setStageSamplers_(SamplerStatePtr const* /*states*/, Int /*startIndex*/, Int /*count*/, ShaderStage /*shaderStage*/) {}

void QOpenglEngine::updateBufferData_(Buffer* /*buffer*/, const void* /*data*/, Int /*lengthInBytes*/) {}

void QOpenglEngine::draw_(GeometryView* /*view*/, UInt /*numIndices*/, UInt /*numInstances*/) {}
void QOpenglEngine::clear_(const core::Color& /*color*/) {}

UInt64 QOpenglEngine::present_(SwapChain* /*swapChain*/, UInt32 /*syncInterval*/, PresentFlags /*flags*/) { return 0; }

//// USER THREAD pimpl functions
//
//SwapChain* QOpenglEngine::createSwapChain_(const SwapChainCreateInfo& desc)
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
//void QOpenglEngine::resizeSwapChain_(SwapChain* /*swapChain*/, UInt32 /*width*/, UInt32 /*height*/)
//{
//    // no-op
//}
//
//Buffer* QOpenglEngine::createBuffer_(
//    Usage usage, BindFlags bindFlags,
//    ResourceMiscFlags resourceMiscFlags, CpuAccessFlags cpuAccessFlags)
//{
//    return new QOpenglBuffer(resourceRegistry_, usage, bindFlags, resourceMiscFlags, cpuAccessFlags);
//}
//
//// RENDER THREAD functions
//
//void QOpenglEngine::bindSwapChain_(SwapChain* swapChain)
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
//UInt64 QOpenglEngine::present_(SwapChain* swapChain, UInt32 /*syncInterval*/, PresentFlags /*flags*/)
//{
//    // XXX check valid ?
//    auto oglChain = static_cast<QOpenglSwapChain*>(swapChain);
//    ctx_->swapBuffers(oglChain->surface());
//    return std::chrono::nanoseconds(std::chrono::steady_clock::now() - startTime_).count();
//}
//
//void QOpenglEngine::bindFramebuffer_(Framebuffer* framebuffer)
//{
//    QOpenglFramebuffer* fb = static_cast<QOpenglFramebuffer*>(framebuffer);
//    GLuint fbo = fb->isDefault_ ? ctx_->defaultFramebufferObject() : fb->object_;
//    api_->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
//}
//
//void QOpenglEngine::setViewport_(Int x, Int y, Int width, Int height)
//{
//    api_->glViewport(x, y, width, height);
//}
//
//void QOpenglEngine::clear_(const core::Color& color)
//{
//    api_->glClearColor(
//        static_cast<float>(color.r()),
//        static_cast<float>(color.g()),
//        static_cast<float>(color.b()),
//        static_cast<float>(color.a()));
//    api_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//}
//
//void QOpenglEngine::setProjectionMatrix_(const geometry::Mat4f& m)
//{
//    paintShaderProgram_->setUniformValue(projLoc_, toQtMatrix(m));
//}
//
//void QOpenglEngine::setViewMatrix_(const geometry::Mat4f& m)
//{
//    paintShaderProgram_->setUniformValue(viewLoc_, toQtMatrix(m));
//}
//
//void QOpenglEngine::initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes)
//{
//    QOpenglBuffer* oglBuffer = static_cast<QOpenglBuffer*>(buffer);
//    oglBuffer->init(data, initialLengthInBytes);
//}
//
//void QOpenglEngine::updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes)
//{
//    QOpenglBuffer* oglBuffer = static_cast<QOpenglBuffer*>(buffer);
//    oglBuffer->load(data, lengthInBytes);
//}
//
//void QOpenglEngine::setupVertexBufferForPaintShader_(Buffer* buffer)
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
//void QOpenglEngine::drawPrimitives_(Buffer* buffer, PrimitiveType type)
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
//void QOpenglEngine::bindPaintShader_()
//{
//    paintShaderProgram_->bind();
//}
//
//void QOpenglEngine::releasePaintShader_()
//{
//    paintShaderProgram_->release();
//}

} // namespace vgc::ui::internal::qopengl
