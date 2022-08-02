//// Copyright 2022 The VGC Developers
//// See the COPYRIGHT file at the top-level directory of this distribution
//// and at https://github.com/vgc/vgc/blob/master/COPYRIGHT
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VGC_UI_QOPENGLENGINE_H
#define VGC_UI_QOPENGLENGINE_H

#include <chrono>
#include <memory>

#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_2_Core>
#include <QOpenGLFunctions_3_3_Core>
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

namespace qopengl {

VGC_DECLARE_OBJECT(QglEngine);

using namespace ::vgc::graphics;

/// \class vgc::widget::QglEngine
/// \brief The QtOpenGL-based graphics::Engine.
///
/// This class is an implementation of graphics::Engine using QOpenGLContext and
/// OpenGL calls.
///
class VGC_UI_API QglEngine : public Engine {
private:
    VGC_OBJECT(QglEngine, Engine)

protected:
    QglEngine();
    QglEngine(QOpenGLContext* ctx, bool isExternalCtx = true);

    void onDestroyed() override;

public:
    using OpenGLFunctions = QOpenGLFunctions_3_3_Core;

    /// Creates a new OpenglEngine.
    ///
    static QglEnginePtr create();
    static QglEnginePtr create(QOpenGLContext* ctx);

    // not part of the common interface

    void setupContext();

    OpenGLFunctions* api() const {
        return api_;
    }

protected:
    // Implementation of Engine API

    // -- USER THREAD implementation functions --

    void createBuiltinShaders_() override;

    SwapChainPtr constructSwapChain_(const SwapChainCreateInfo& createInfo) override;
    FramebufferPtr constructFramebuffer_(const ImageViewPtr& colorImageView) override;
    BufferPtr constructBuffer_(const BufferCreateInfo& createInfo) override;
    ImagePtr constructImage_(const ImageCreateInfo& createInfo) override;
    ImageViewPtr constructImageView_(const ImageViewCreateInfo& createInfo, const ImagePtr& image) override;
    ImageViewPtr constructImageView_(const ImageViewCreateInfo& createInfo, const BufferPtr& buffer, ImageFormat format, UInt32 numElements) override;
    SamplerStatePtr constructSamplerState_(const SamplerStateCreateInfo& createInfo) override;
    GeometryViewPtr constructGeometryView_(const GeometryViewCreateInfo& createInfo) override;
    BlendStatePtr constructBlendState_(const BlendStateCreateInfo& createInfo) override;
    RasterizerStatePtr constructRasterizerState_(const RasterizerStateCreateInfo& createInfo) override;

    void resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height) override;

    //--  RENDER THREAD implementation functions --

    void initFramebuffer_(Framebuffer* framebuffer) override;
    void initBuffer_(Buffer* buffer, const char* data, Int lengthInBytes) override;
    void initImage_(Image* image, const Span<const Span<const char>>* dataSpanSpan) override;
    void initImageView_(ImageView* view) override;
    void initSamplerState_(SamplerState* state) override;
    void initGeometryView_(GeometryView* view) override;
    void initBlendState_(BlendState* state) override;
    void initRasterizerState_(RasterizerState* state) override;

    void setSwapChain_(const SwapChainPtr& swapChain) override;
    void setFramebuffer_(const FramebufferPtr& framebuffer) override;
    void setViewport_(Int x, Int y, Int width, Int height) override;
    void setProgram_(const ProgramPtr& program) override;
    void setBlendState_(const BlendStatePtr& state, const geometry::Vec4f& blendFactor) override;
    void setRasterizerState_(const RasterizerStatePtr& state) override;
    void setStageConstantBuffers_(BufferPtr const* buffers, Int startIndex, Int count, ShaderStage shaderStage) override;
    void setStageImageViews_(ImageViewPtr const* views, Int startIndex, Int count, ShaderStage shaderStage) override;
    void setStageSamplers_(SamplerStatePtr const* states, Int startIndex, Int count, ShaderStage shaderStage) override;

    void updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes) override;

    void draw_(GeometryView* view, UInt numIndices, UInt numInstances) override;
    void clear_(const core::Color& color) override;

    UInt64 present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags flags) override;

private:
    // XXX keep only format of first chain and compare against new windows ?
    QSurfaceFormat format_;
    QOpenGLContext* ctx_ = nullptr;
    bool isExternalCtx_ = false;
    QOpenGLFunctions_3_3_Core* api_ = nullptr;
    QSurface* surface_ = nullptr;

    template<typename T, typename... Args>
    _NODISCARD std::unique_ptr<T> makeUnique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

    void initBuiltinShaders_();

    // Shader
    //std::unique_ptr<QOpenGLShaderProgram> paintShaderProgram_;
    //int posLoc_ = -1;
    //int colLoc_ = -1;
    //int projLoc_ = -1;
    //int viewLoc_ = -1;
};

} // namespace qopengl

using QglEngine = qopengl::QglEngine;
using QglEnginePtr = qopengl::QglEnginePtr;

} // namespace vgc::ui::internal

#endif // VGC_UI_QOPENGLENGINE_H
