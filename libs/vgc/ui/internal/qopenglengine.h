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

#ifndef VGC_UI_QOPENGLENGINE_H
#define VGC_UI_QOPENGLENGINE_H

#include <chrono>
#include <memory>

#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_2_Core>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QPointF>
#include <QString>

#include <vgc/core/color.h>
#include <vgc/core/paths.h>
#include <vgc/geometry/mat4d.h>
#include <vgc/geometry/mat4f.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/geometry/vec2f.h>
#include <vgc/graphics/engine.h>
#include <vgc/ui/api.h>

namespace vgc::ui::internal {

inline QString toQt(const std::string& s)
{
    int size = vgc::core::int_cast<int>(s.size());
    return QString::fromUtf8(s.data(), size);
}

inline std::string fromQt(const QString& s)
{
    QByteArray a = s.toUtf8();
    size_t size = vgc::core::int_cast<size_t>(s.size());
    return std::string(a.data(), size);
}

inline QPointF toQt(const geometry::Vec2d& v)
{
    return QPointF(v[0], v[1]);
}

inline QPointF toQt(const geometry::Vec2f& v)
{
    return QPointF(v[0], v[1]);
}

inline geometry::Vec2d fromQtd(const QPointF& v)
{
    return geometry::Vec2d(v.x(), v.y());
}

inline geometry::Vec2f fromQtf(const QPointF& v)
{
    return geometry::Vec2f(v.x(), v.y());
}

inline QMatrix4x4 toQtMatrix(const geometry::Mat4f& m) {
    return QMatrix4x4(
        m(0,0), m(0,1), m(0,2), m(0,3),
        m(1,0), m(1,1), m(1,2), m(1,3),
        m(2,0), m(2,1), m(2,2), m(2,3),
        m(3,0), m(3,1), m(3,2), m(3,3));
}

inline geometry::Mat4f toMat4f(const geometry::Mat4d& m) {
    // TODO: implement Mat4d to Mat4f conversion directly in Mat4x classes
    return geometry::Mat4f(
        (float)m(0,0), (float)m(0,1), (float)m(0,2), (float)m(0,3),
        (float)m(1,0), (float)m(1,1), (float)m(1,2), (float)m(1,3),
        (float)m(2,0), (float)m(2,1), (float)m(2,2), (float)m(2,3),
        (float)m(3,0), (float)m(3,1), (float)m(3,2), (float)m(3,3));
}

VGC_DECLARE_OBJECT(QOpenglEngine);

/// \class vgc::widget::QOpenglEngine
/// \brief The QtOpenGL-based graphics::Engine.
///
/// This class is an implementation of graphics::Engine using QOpenGLContext and
/// OpenGL calls.
///
class VGC_UI_API QOpenglEngine : public graphics::Engine {
private:
    VGC_OBJECT(QOpenglEngine, graphics::Engine)

protected:
    QOpenglEngine();
    QOpenglEngine(QOpenGLContext* ctx, bool isExternalCtx = true);

    void onDestroyed() override;

public:
    using OpenGLFunctions = QOpenGLFunctions_3_2_Core;

    /// Creates a new OpenglEngine.
    ///
    static QOpenglEnginePtr create();
    static QOpenglEnginePtr create(QOpenGLContext* ctx);

    // not part of the common interface

    void setupContext();

    OpenGLFunctions* api() const {
        return api_;
    }

public:
    // Implementation of graphics::Engine API

    // USER THREAD pimpl functions

    graphics::SwapChain* createSwapChain_(const graphics::SwapChainCreateInfo& desc) override;
    void resizeSwapChain_(graphics::SwapChain* swapChain, UInt32 width, UInt32 height) override;
    graphics::Buffer* createBuffer_(
        graphics::Usage usage, graphics::BindFlags bindFlags,
        graphics::ResourceMiscFlags resourceMiscFlags, graphics::CpuAccessFlags cpuAccessFlags) override;

    // RENDER THREAD functions

    void bindSwapChain_(graphics::SwapChain* swapChain) override;
    UInt64 present_(graphics::SwapChain* swapChain, UInt32 syncInterval, graphics::PresentFlags flags) override;
    void bindFramebuffer_(graphics::Framebuffer* framebuffer) override;
    void setViewport_(Int x, Int y, Int width, Int height) override;
    void clear_(const core::Color& color) override;
    void setProjectionMatrix_(const geometry::Mat4f& m) override;
    void setViewMatrix_(const geometry::Mat4f& m) override;

    void initBuffer_(graphics::Buffer* buffer, const void* data, Int initialLengthInBytes) override;
    void updateBufferData_(graphics::Buffer* buffer, const void* data, Int lengthInBytes) override;
    void setupVertexBufferForPaintShader_(graphics::Buffer* buffer) override;
    void drawPrimitives_(graphics::Buffer* buffer, graphics::PrimitiveType type) override;
    GL_DEPTH_COMPONENT16
        GL_RGBA32I
    //void bindPaintShader_() override;
    //void releasePaintShader_() override;
    //void bindAtlasGlyphShader_() override;
    //void releaseAtlasGlyphShader_() override;
    //void bindRoundedRectangleShader_() override;
    //void releaseRoundedRectangleShader_() override;

private:
    // XXX keep only format of first chain and compare against new windows ?
    QSurfaceFormat format_;
    QOpenGLContext* ctx_ = nullptr;
    bool isExternalCtx_ = false;
    QOpenGLFunctions_3_2_Core* api_ = nullptr;

    QSurface* surface_ = nullptr;

    std::chrono::steady_clock::time_point startTime_;

    // Shader
    std::unique_ptr<QOpenGLShaderProgram> paintShaderProgram_;
    int posLoc_ = -1;
    int colLoc_ = -1;
    int projLoc_ = -1;
    int viewLoc_ = -1;
};

} // namespace vgc::ui::internal

#endif // VGC_UI_QOPENGLENGINE_H
