// Copyright 2021 The VGC Developers
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

#ifndef VGC_GRAPHICS_ENGINE_H
#define VGC_GRAPHICS_ENGINE_H

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>

#include <vgc/core/assert.h>
#include <vgc/core/color.h>
#include <vgc/core/enum.h>
#include <vgc/core/innercore.h>
#include <vgc/core/templateutil.h>
#include <vgc/geometry/mat4f.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/buffers.h>
#include <vgc/graphics/detail/commands.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>
#include <vgc/graphics/targets.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

/// \class vgc::graphics::Engine
/// \brief Abstract interface for graphics rendering.
///
/// This class is an abstract base class defining a common shared API for
/// graphics rendering. Implementations of this abstraction may provide
/// backends such as OpenGL, Vulkan, Direct3D, Metal, or software rendering.
///
/// The graphics engine is responsible for managing two matrix stacks: the
/// projection matrix stack, and the view matrix stack. When the engine is
/// constructed, each of these stacks is initialized with the identity matrix
/// as the only matrix in the stack. It is undefined behavior for clients to
/// call "pop" more times than "push": some implementations may emit an
/// exception (instantly or later), others may cause a crash (instantly or
/// later), or the drawing operations may fail.
///
class VGC_GRAPHICS_API Engine : public core::Object {
private:
    VGC_OBJECT(Engine, core::Object)
    VGC_PRIVATIZE_OBJECT_TREE_MUTATORS

    using Command = detail::Command;
    using CommandUPtr = std::unique_ptr<Command>;

protected:
    /// Constructs an Engine. This constructor is an implementation detail only
    /// available to derived classes.
    ///
    Engine();

public:
    // MODEL THREAD functions

    /// Submits the current command list for execution by the render thread.
    /// Returns the index assigned to the submitted command list.
    ///
    /// If the current command list is empty, `flush()` does nothing and
    /// returns the index of the previous list.
    ///
    UInt flush() {
        return flushPendingCommandList_();
    }

    /// Submits the current command list if present then waits for all submitted
    /// command lists to finish executing on the CPU.
    ///
    void finish() {
        UInt id = flushPendingCommandList_();
        waitCommandListExecutionFinished_(id);
    }

    void present(SwapChainPtr swapChain, UInt32 syncInterval) {
        // XXX command + finish if syncInterval > 0
    }

    void setOutputStage(RenderTargetViewPtr rtv = nullptr) {
        // XXX command
    }

    void setViewport(Int x, Int y, Int width, Int height) {
        // XXX command
    }

    /// Creates a swap chain for the given window.
    ///
    virtual SwapChainPtr createSwapChain(void* windowNativeHandle, UInt32 flags);

    /// Creates a float buffer for storing primitives data, and returns a shared
    /// pointer to the new resource.
    ///
    /// Once created, you can load triangles data using
    /// resource->load(...), and you can draw the loaded triangles
    /// using resource->draw().
    ///
    /// When you don't need the buffer anymore (e.g., in the
    /// vgc::ui::Widget::cleanup() function), you must reset the resource pointer.
    ///
    // Note: in the future, we may add overloads to this function to allow
    // specifying the vertex format (i.e., XYRGB, XYZRGBA, etc.), but for now
    // the format is fixed (XYRGB).
    //
    virtual PrimitiveBufferPtr createPrimitivesBuffer(PrimitiveType type) = 0;

    /// Clears the whole render area with the given color.
    ///
    void clear(const core::Color& color) {
        queueCommand_<detail::ClearCommand>(color);
    }

    /// Returns the current projection matrix (top-most on the stack).
    ///
    geometry::Mat4f projectionMatrix() const {
        return projectionMatrixStack_.last();
    }

    /// Assigns `m` to the top-most matrix of the projection matrix stack.
    /// `m` becomes the current projection matrix.
    ///
    void setProjectionMatrix(const geometry::Mat4f& m) {
        queueCommand_<detail::SetProjectionMatrixCommand>(m);
    }

    /// Duplicates the top-most matrix on the projection matrix stack.
    ///
    void pushProjectionMatrix() {
        projectionMatrixStack_.emplaceLast(projectionMatrixStack_.last());
        setViewMatrix(projectionMatrixStack_.last());
    }

    /// Removes the top-most matrix of the projection matrix stack.
    /// The new top-most matrix becomes the current projection matrix.
    ///
    /// The behavior is undefined if there is only one matrix in the stack
    /// before calling this function.
    ///
    void popProjectionMatrix() {
        projectionMatrixStack_.removeLast();
        setViewMatrix(projectionMatrixStack_.last());
    }

    /// Returns the current view matrix (top-most on the stack).
    ///
    geometry::Mat4f viewMatrix() const {
        return viewMatrixStack_.last();
    }

    /// Assigns `m` to the top-most matrix of the view matrix stack.
    /// `m` becomes the current view matrix.
    ///
    void setViewMatrix(const geometry::Mat4f& m) {
        queueCommand_<detail::SetViewMatrixCommand>(m);
    }

    /// Duplicates the top-most matrix on the view matrix stack.
    ///
    void pushViewMatrix() {
        viewMatrixStack_.emplaceLast(viewMatrixStack_.last());
        setViewMatrix(viewMatrixStack_.last());
    }

    /// Removes the top-most matrix of the view matrix stack.
    /// The new top-most matrix becomes the current view matrix.
    ///
    /// The behavior is undefined if there is only one matrix in the stack
    /// before calling this function.
    ///
    void popViewMatrix() {
        viewMatrixStack_.removeLast();
        setViewMatrix(viewMatrixStack_.last());
    }

    void bindPaintShader() {
        // XXX command
    }

    void releasePaintShader() {
        // XXX command
    }

    void drawPrimitives(const PrimitiveBufferPtr& buffer) {
        // XXX command
    }

protected:
    detail::ResourceList* resourceList_;
    std::list<CommandUPtr> commandList_;

    // RENDER THREAD functions

    virtual void present_(const SwapChainPtr& swapChain, UInt32 syncInterval) = 0;
    virtual void setOutputStage_(const RenderTargetViewPtr& rtv = nullptr) = 0;
    virtual void setViewport_(Int x, Int y, Int width, Int height) = 0;
    virtual void setProjectionMatrix_(const geometry::Mat4f& m) = 0;
    virtual void setViewMatrix_(const geometry::Mat4f& m) = 0;
    virtual void bindPaintShader_();
    virtual void releasePaintShader_();
    virtual void drawPrimitives_(const PrimitiveBufferPtr& buffer) = 0;

    template<typename TCommand, typename... Args>
    void queueCommand_(Args&&... args) {
        commandList_.emplace_back(new TCommand(std::forward<Args>(args)...));
    };

private:
    core::Array<geometry::Mat4f> projectionMatrixStack_;
    core::Array<geometry::Mat4f> viewMatrixStack_;

    std::thread renderThread_;

    std::mutex mutex_;
    std::condition_variable wakeRenderThreadConditionVariable_;
    std::condition_variable renderThreadEventConditionVariable_;

    core::Array<std::list<CommandUPtr>> commandQueue_;
    UInt lastExecutedCommandListId_ = 0;
    UInt lastSubmittedCommandListId_ = 0;

    bool running_ = false;
    bool stopRequested_ = false;

    void renderThreadProc_();
    void startRenderThread_();
    void stopRenderThread_(); // blocking

    UInt flushPendingCommandList_();

    // returns false if execution was cancelled by a stop request.
    void waitCommandListExecutionFinished_(UInt commandListId = 0);
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_ENGINE_H
