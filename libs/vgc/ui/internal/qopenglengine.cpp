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

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QOpenGLVersionFunctionsFactory>
#endif
#include <QWindow>

#include <vgc/core/exceptions.h>
#include <vgc/core/paths.h>

namespace vgc::ui::internal {

/*
DESIGN PSEUDOCODE:
Context {
int id;
}
Engine {
Context* ctx;
}

Foo = Engine or Context

Widget {
Foo ref;
rid resourceId;

paintDrawInternal(Engine* drawingEngine) {
if (drawingEngine.ref != ref) {
onPaintDestroy(ref);
ctx = drawingEngine->ctx;
onPaintCreate(drawingEngine);
ctx->onDestroy().connect(this->onCtxDestroyed());
}

paintDraw(drawingEngine);
}

void onCtxDestroyed_(Context*) {
onPaintDestroy(ctx);
}
VGC_SLOT(onCtxDestroyed, onCtxDestroyed_);

virtual onPaintCreate(Engine*);
virtual onPaintDraw(Engine*);
virtual onPaintDestroy(Foo*);
}

*/

namespace {

// Returns the file path of a shader file as a QString
QString shaderPath_(const std::string& name) {
    std::string path = core::resourcePath("graphics/opengl/" + name);
    return toQt(path);
}

struct XYRGBVertex {
    float x, y, r, g, b;
};

} // namespace

namespace {

class QOpenglBuffer : public graphics::Buffer {
public:
    using Buffer::Buffer;

    void init() {
        if (!vao_) {
            vbo_.create();
            vao_ = new QOpenGLVertexArrayObject();
            vao_->create();
        }
    }

    void bind() {
        if (vao_) {
            vao_->bind();
            vbo_.bind();
        }
    }

    void unbind() {
        if (vao_) {
            vbo_.release();
            vao_->release();
        }
    }

    void load(const float* data, Int length) {
        if (!vao_) return;
        if (length < 0) {
            throw core::NegativeIntegerError(core::format(
                "Negative length ({}) provided to loadTriangles()", length));
        }
        numVertices_ = length / 5;
        Int dataSize = numVertices_ * sizeof(XYRGBVertex);
        static_assert(sizeof(XYRGBVertex) == 5 * sizeof(float));

        vbo_.bind();
        if (dataSize > allocSize_) {
            vbo_.allocate(data, dataSize);
            allocSize_ = dataSize;
        }
        else if (dataSize * 2 < allocSize_) {
            vbo_.allocate(data, dataSize);
            allocSize_ = dataSize;
        }
        else {
            vbo_.write(0, data, dataSize);
        }
        vbo_.release();
    }

    void draw(QOpenglEngine* engine, GLenum mode) {
        if (!vao_) return;
        vao_->bind();
        auto api = engine->api();
        api->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        api->glDrawArrays(mode, 0, numVertices_);
        vao_->release();
    }

protected:
    void release_(graphics::Engine* engine) override {
        if (!vao_) return;
        vao_->destroy();
        delete vao_;
        vao_ = nullptr;
        vbo_.destroy();
    }

private:
    QOpenGLBuffer vbo_;
    QOpenGLVertexArrayObject* vao_ = nullptr;
    Int numVertices_ = 0;
    Int allocSize_ = 0;
};

class QOpenglSwapChain : public graphics::SwapChain {
public:
    QOpenglSwapChain(ResourceList* owningList, const graphics::SwapChainDesc& desc, QSurface* surface)
        : SwapChain(owningList, desc), surface_(surface) {}

    QSurface* surface() const {
        return surface_;
    }

protected:
    void release_(graphics::Engine* engine) override {
        // no-op..
    }

private:
    QSurface* surface_;
};

class QOpenglRenderTargetView : public graphics::RenderTargetView {
public:
    QOpenglRenderTargetView(ResourceList* owningList, QSurface* surface)
        : RenderTargetView(owningList), surface_(surface) {}

    QSurface* surface() const {
        return surface_;
    }

protected:
    void release_(graphics::Engine* engine) override {
        // no-op..
    }

private:
    QSurface* surface_;
};

} // namespace

QOpenglEngine::QOpenglEngine() :
    QOpenglEngine(new QOpenGLContext(), false)
{
}

QOpenglEngine::QOpenglEngine(QOpenGLContext* ctx, bool isExternalCtx) :
    graphics::Engine(),
    ctx_(ctx),
    isExternalCtx_(isExternalCtx),
    projectionMatrices_({geometry::Mat4f::identity}),
    viewMatrices_({geometry::Mat4f::identity})
{
}

void QOpenglEngine::onDestroyed()
{
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

void QOpenglEngine::makeCurrent()
{
    if (ctx_ == nullptr) {
        throw core::LogicError("ctx_ is null.");
    }
    if (api_ != nullptr) {
        throw core::LogicError("already initialized.");
    }

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(8);
    format.setSwapInterval(0);
    ctx_->setFormat(format);
    ctx_->create();
}

void QOpenglEngine::setupContext()
{
    ctx_->makeCurrent(qw);

    // Initialize shader program
    shaderProgram_.addShaderFromSourceFile(QOpenGLShader::Vertex, shaderPath_("iv4pos_iv4col_um4proj_um4view_ov4fcol.v.glsl"));
    shaderProgram_.addShaderFromSourceFile(QOpenGLShader::Fragment, shaderPath_("iv4fcol.f.glsl"));
    shaderProgram_.link();

    // Get shader locations
    shaderProgram_.bind();
    posLoc_  = shaderProgram_.attributeLocation("pos");
    colLoc_  = shaderProgram_.attributeLocation("col");
    projLoc_ = shaderProgram_.uniformLocation("proj");
    viewLoc_ = shaderProgram_.uniformLocation("view");
    shaderProgram_.release();

    // Initialize engine
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    api_ = ctx_->versionFunctions<QOpenGLFunctions_3_2_Core>();
#else
    api_ = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_2_Core>(ctx_);
#endif
    //

    api_->initializeOpenGLFunctions();

    // Initialize widget for painting.
    // Note that initializedGL() is never called if the widget is never visible.
    // Therefore it's important to keep track whether it has been called, so that
    // we don't call onPaintDestroy() without first calling onPaintCreate()

    /*ctx_->makeCurrent(this);

    oglf_->glViewport(0, 0, width(), height());

    oglf_->glClearColor(1.f, 0, 0, 1.f);
    oglf_->glClear(GL_COLOR_BUFFER_BIT);

    m_context->swapBuffers(this);

    widget_->onPaintCreate(engine_.get());*/
}


graphics::RenderTargetViewPtr QOpenglEngine::getDefaultTarget(QSurface* surface)
{


}


// USER THREAD pimpl functions

graphics::SwapChainPtr QOpenglEngine::createSwapChain_(const graphics::SwapChainDesc& desc, void* windowNativeHandle, graphics::WindowNativeHandleType handleType, UInt32 flags)
{
    if (ctx_ == nullptr) {
        throw core::LogicError("ctx_ is null.");
    }

    if (handleType != graphics::WindowNativeHandleType::QOpenGLWindow) {
        return nullptr;
    }

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(desc.sampleCount());
    format.setSwapInterval(0);

    QWindow* wnd = static_cast<QWindow*>(windowNativeHandle);
    wnd->setFormat(format);

    ctx_->setFormat(format);
    ctx_->create();

    return new QOpenglSwapChain(resourceList_, desc, wnd);
}

graphics::RenderTargetViewPtr QOpenglEngine::constructRenderTargetView_(const graphics::SwapChainPtr& swapChain)
{


}

graphics::BufferPtr QOpenglEngine::constructBuffer_(graphics::Usage usage, graphics::BindFlags bindFlags, graphics::CpuAccessFlags cpuAccessFlags)
{


}

// RENDER THREAD functions

void QOpenglEngine::present_(const graphics::SwapChainPtr& swapChain, UInt32 /*syncInterval*/)
{
    // XXX check valid
    auto oglChain = static_cast<QOpenglSwapChain*>(swapChain.get());
    ctx_->swapBuffers(oglChain->surface());
}

void QOpenglEngine::setTarget_(const graphics::RenderTargetViewPtr& rtv = nullptr)
{
    if (!api_) {
        initContext();
    }
    auto oglRTV = static_cast<QOpenglRenderTargetView*>(rtv.get());
    ctx_->makeCurrent(oglRTV->surface());

#ifdef VGC_QOPENGL_EXPERIMENT
    auto fmt = ctx_->format();
    OutputDebugString(core::format("Ctx swap behavior: {}\n", (int)fmt.swapBehavior()).c_str());
    OutputDebugString(core::format("Ctx swap interval: {}\n", fmt.swapInterval()).c_str());
#endif
}

void QOpenglEngine::setViewport_(Int x, Int y, Int width, Int height)
{
    api_->glViewport(x, y, width, height);
}

void QOpenglEngine::clear_(const core::Color& color)
{
    api_->glClearColor(
        static_cast<float>(color.r()),
        static_cast<float>(color.g()),
        static_cast<float>(color.b()),
        static_cast<float>(color.a()));
    api_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void QOpenglEngine::setProjectionMatrix_(const geometry::Mat4f& m)
{
    paintShaderProgram_->setUniformValue(projLoc_, toQtMatrix(m));
}

void QOpenglEngine::setViewMatrix_(const geometry::Mat4f& m)
{
    paintShaderProgram_->setUniformValue(viewLoc_, toQtMatrix(m));
}

void QOpenglEngine::bindPaintShader_()
{
    paintShaderProgram_->bind();
}

void QOpenglEngine::releasePaintShader_()
{
    paintShaderProgram_->release();
}

void QOpenglEngine::initBuffer_(const graphics::BufferPtr& buffer, const float* data, Int initialLengthInBytes)
{


}

void QOpenglEngine::updateBufferData_(const graphics::BufferPtr& buffer, const float* data, Int lengthInBytes)
{


}

void QOpenglEngine::initVertexBufferForPaintShader_(const graphics::BufferPtr& buffer)
{
    auto r = QOpenglPrimitiveBuffer::create(this);
    GLsizei stride = sizeof(XYRGBVertex);
    GLvoid* posPointer = reinterpret_cast<void*>(offsetof(XYRGBVertex, x));
    GLvoid* colPointer = reinterpret_cast<void*>(offsetof(XYRGBVertex, r));
    GLboolean normalized = GL_FALSE;
    r->bind();
    api_->glEnableVertexAttribArray(posLoc_);
    api_->glEnableVertexAttribArray(colLoc_);
    api_->glVertexAttribPointer(posLoc_, 2, GL_FLOAT, normalized, stride, posPointer);
    api_->glVertexAttribPointer(colLoc_, 3, GL_FLOAT, normalized, stride, colPointer);
    r->unbind();
    return r;

}

void QOpenglEngine::drawPrimitives_(const graphics::BufferPtr& buffer, graphics::PrimitiveType type)
{
    GLenum mode = 0;
    switch (type) {
    case graphics::PrimitiveType::LineList: mode = GL_LINES; break;
    case graphics::PrimitiveType::LineStrip: mode = GL_LINE_STRIP; break;
    case graphics::PrimitiveType::TriangleList: mode = GL_TRIANGLES; break;
    case graphics::PrimitiveType::TriangleStrip: mode = GL_TRIANGLE_STRIP; break;
    default:
        mode = GL_POINTS;
        break;
    }
    auto oglBuffer = static_cast<QOpenglBuffer*>(buffer.get());
    oglBuffer->draw(this, mode);
}

graphics::PrimitiveBufferPtr QOpenglEngine::createPrimitivesBuffer(graphics::PrimitiveType type)
{
    
}






} // namespace vgc::ui::internal
