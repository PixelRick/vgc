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

#include <atomic>
#include <condition_variable>
#include <functional>
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
#include <vgc/graphics/logcategories.h>
#include <vgc/graphics/resource.h>
#include <vgc/graphics/targets.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

// XXX add something to limit the number of pending frames for each swapchain..

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

    void onDestroyed() override {
        stopRenderThread_();
    }

public:

    void start() {
        startRenderThread_();
    }

    // MODEL THREAD functions

    /// Submits the current command list for execution by the render thread.
    /// Returns the index assigned to the submitted command list.
    ///
    /// If the current command list is empty, `flush()` does nothing and
    /// returns the index of the previous list.
    ///
    UInt flush() {
        return submitPendingCommandList_();
    }

    /// Submits the current command list if present then waits for all submitted
    /// command lists to finish being translated to GPU commands.
    ///
    void finish() {
        UInt id = submitPendingCommandList_();
        waitCommandListTranslationFinished_(id);
    }

    /// Creates a swap chain for the given window.
    ///
    SwapChainPtr createSwapChain(const SwapChainDesc& desc) {
        return SwapChainPtr(createSwapChain_(desc));
    }

    void resizeSwapChain(SwapChain* swapChain, UInt32 width, UInt32 height) {
        if (swapChain->owningList_ != resourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }
        finish();
        resizeSwapChain_(swapChain, width, height);
    }

    void bindSwapChain(const SwapChainPtr& swapChain) {
        if (swapChain->owningList_ != resourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }
        lastBoundSwapChain_ = swapChain;
        queueLambdaCommandWithParameters_<SwapChain*>(
            "bindSwapChain",
            [](Engine* engine, SwapChain* swapChain) {
                engine->bindSwapChain_(swapChain);
            },
            swapChain.get());
    }

    // presentedCallback is called from an unspecified thread.
    void present(
        UInt32 syncInterval,
        std::function<void(UInt64 /*timestamp*/)>&& presentedCallback,
        PresentFlags flags = PresentFlags::None)
    {
        ++lastBoundSwapChain_->pendingPresentCount_;
        bool shouldSync = syncInterval > 0;
        if (shouldSync && shouldPresentWaitFromSyncedUserThread_()) {
            // Preventing dead-locks
            // See https://docs.microsoft.com/en-us/windows/win32/api/DXGI1_2/nf-dxgi1_2-idxgiswapchain1-present1#remarks
            finish();
            UInt64 timestamp = present_(lastBoundSwapChain_.get(), syncInterval, flags);
            --lastBoundSwapChain_->pendingPresentCount_;
            presentedCallback(timestamp);
        }
        else {
            struct CommandParameters {
                SwapChain* swapChain;
                UInt32 syncInterval;
                PresentFlags flags;
                std::function<void(UInt64 /*timestamp*/)> presentedCallback;
            };
            queueLambdaCommandWithParameters_<CommandParameters>(
                "present",
                [](Engine* engine, const CommandParameters& p) {
                    UInt64 timestamp = engine->present_(p.swapChain, p.syncInterval, p.flags);
                    --p.swapChain->pendingPresentCount_;
                    p.presentedCallback(timestamp);
                },
                lastBoundSwapChain_.get(), syncInterval, flags, std::move(presentedCallback));

            if (shouldSync) {
                finish();
            }
            else {
                submitPendingCommandList_();
            }
        }
    }

    FramebufferPtr getDefaultFramebuffer() {
        SwapChain* swapChain = lastBoundSwapChain_.get();
        return swapChain ? swapChain->defaultFrameBuffer_ : FramebufferPtr();
    }

    void bindFramebuffer(Framebuffer* framebuffer = nullptr) {
        if (!framebuffer) {
            framebuffer = getDefaultFramebuffer().get();
        }
        else if (framebuffer->owningList_ != resourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }
        queueLambdaCommandWithParameters_<Framebuffer*>(
            "bindFramebuffer",
            [](Engine* engine, Framebuffer* framebuffer) {
                engine->bindFramebuffer_(framebuffer);
            },
            framebuffer);
    }

    void setViewport(Int x, Int y, Int width, Int height) {
        struct CommandParameters {
            Int x, y, width, height;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "setViewport",
            [](Engine* engine, const CommandParameters& p) {
                engine->setViewport_(p.x, p.y, p.width, p.height);
            },
            x, y, width, height);
    }

    /// Clears the whole render area with the given color.
    ///
    void clear(const core::Color& color) {
        queueLambdaCommandWithParameters_<core::Color>(
            "clear",
            [](Engine* engine, const core::Color& c) {
                engine->clear_(c);
            },
            color);
    }

    /// Returns the current projection matrix (top-most on the stack).
    ///
    geometry::Mat4f projectionMatrix() const {
        return projectionMatrixStack_.last();
    }

    /// Assigns `m` to the top-most matrix of the projection matrix stack.
    /// `m` becomes the current projection matrix.
    ///
    void setProjectionMatrix(const geometry::Mat4f& projectionMatrix) {
        projectionMatrixStack_.last() = projectionMatrix;
        queueLambdaCommandWithParameters_<geometry::Mat4f>(
            "setProjectionMatrix",
            [](Engine* engine, const geometry::Mat4f& m) {
                engine->setProjectionMatrix_(m);
            },
            projectionMatrix);
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
    void setViewMatrix(const geometry::Mat4f& viewMatrix) {
        viewMatrixStack_.last() = viewMatrix;
        queueLambdaCommandWithParameters_<geometry::Mat4f>(
            "setViewMatrix",
            [](Engine* engine, const geometry::Mat4f& m) {
                engine->setViewMatrix_(m);
            },
            viewMatrix);
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
        queueLambdaCommand_(
            "bindPaintShader",
            [](Engine* engine) {
                engine->bindPaintShader_();
            });
    }

    void releasePaintShader() {
        queueLambdaCommand_(
            "releasePaintShader",
            [](Engine* engine) {
                engine->releasePaintShader_();
            });
    }

    // XXX fix comment
    /// Creates a resource for storing primitives data, and returns a shared
    /// pointer to it.
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
    template<typename DataGetter>
    BufferPtr createPrimitiveBuffer(DataGetter&& initialDataGetter, Int initialLengthInBytes, bool dynamic) {
        if (initialLengthInBytes < 0) {
            throw core::NegativeIntegerError(core::format(
                "Negative initialLengthInBytes ({}) provided to vgc::graphics::Engine::createPrimitiveBuffer()", initialLengthInBytes));
        }

        BufferPtr buffer = BufferPtr(createBuffer_(
            dynamic ? Usage::Dynamic : Usage::Immutable,
            BindFlags::VertexBuffer,
            dynamic ? CpuAccessFlags::Write : CpuAccessFlags::None));

        struct CommandParameters {
            Buffer* buffer;
            std::decay_t<DataGetter> initialDataGetter;
            Int initialLengthInBytes;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "initPrimitiveBuffer",
            [](Engine* engine, const CommandParameters& p) {
                engine->initBuffer_(p.buffer, p.initialDataGetter(), p.initialLengthInBytes);
                engine->initVertexBufferForPaintShader_(buf);
            },
            buffer.get(), std::move(initialDataGetter), initialLengthInBytes);
        return buffer;
    }

    BufferPtr createDynamicPrimitiveBuffer() {
        BufferPtr buffer = BufferPtr(createBuffer_(
            Usage::Dynamic,
            BindFlags::VertexBuffer,
            CpuAccessFlags::Write));

        queueLambdaCommandWithParameters_<Buffer*>(
            "initPrimitiveBuffer",
            [](Engine* engine, Buffer* buf) {
                engine->initBuffer_(buf, nullptr, 0);
                engine->setupVertexBufferForPaintShader_(buf);
            },
            buffer.get());
        return buffer;
    }

    template<typename DataGetter>
    void updateBufferData(Buffer* buffer, DataGetter&& initialDataGetter, Int lengthInBytes) {
        if (lengthInBytes < 0) {
            throw core::NegativeIntegerError(core::format(
                "Negative lengthInBytes ({}) provided to vgc::graphics::Engine::updateBufferData()", lengthInBytes));
        }

        if (buffer->owningList_ != resourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }

        if (!(buffer->cpuAccessFlags() & CpuAccessFlags::Write)) {
            VGC_ERROR(LogVgcGraphics, "cpu does not have write access on buffer");
        }

        struct CommandParameters {
            Buffer* buffer;
            std::decay_t<DataGetter> initialDataGetter;
            Int lengthInBytes;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "updateBufferData",
            [](Engine* engine, const CommandParameters& p) {
                engine->updateBufferData_(p.buffer, p.initialDataGetter(), p.lengthInBytes);
            },
            buffer, std::move(initialDataGetter), lengthInBytes);
    }

    void drawPrimitives(Buffer* buffer, PrimitiveType type) {
        if (buffer->owningList_ != resourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }

        struct CommandParameters {
            Buffer* buffer;
            PrimitiveType type;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "drawPrimitives",
            [](Engine* engine, const CommandParameters& p) {
                engine->drawPrimitives_(p.buffer, p.type);
            },
            buffer, type);
    }

protected:
    detail::ResourceList* resourceList_ = nullptr;
    std::list<CommandUPtr> pendingCommands_;

    // USER THREAD pimpl functions

    virtual SwapChain* createSwapChain_(const SwapChainDesc& desc) = 0;
    virtual void resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height) = 0;
    // XXX virtual FramebufferPtr createFramebuffer_(const RenderTargetViewPtr& colorRtv) = 0;
    // XXX virtual void setDefaultFramebuffer_(const SwapChainPtr& swapChain, const FramebufferPtr& fb) = 0;
    virtual Buffer* createBuffer_(Usage usage, BindFlags bindFlags, CpuAccessFlags cpuAccessFlags) = 0;

    virtual bool shouldPresentWaitFromSyncedUserThread_() { return false; }

    // RENDER THREAD functions

    virtual void bindSwapChain_(SwapChain* swapChain) = 0;
    virtual UInt64 present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags flags) = 0;
    // XXX virtual void initFramebuffer_(Framebuffer* framebuffer) = 0;
    virtual void bindFramebuffer_(Framebuffer* framebuffer) = 0;
    virtual void setViewport_(Int x, Int y, Int width, Int height) = 0;
    virtual void clear_(const core::Color& color) = 0;
    virtual void setProjectionMatrix_(const geometry::Mat4f& m) = 0;
    virtual void setViewMatrix_(const geometry::Mat4f& m) = 0;
    virtual void bindPaintShader_() = 0;
    virtual void releasePaintShader_() = 0;
    virtual void initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes) = 0;
    virtual void updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes) = 0;
    virtual void setupVertexBufferForPaintShader_(Buffer* buffer) = 0;
    virtual void drawPrimitives_(Buffer* buffer, PrimitiveType type) = 0;

    // QUEUING

    template<typename TCommand, typename... Args>
    void queueCommand_(Args&&... args) {
        pendingCommands_.emplace_back(
            new TCommand(std::forward<Args>(args)...));
    };

    template<typename Lambda>
    void queueLambdaCommand_(std::string_view name, Lambda&& lambda) {
        pendingCommands_.emplace_back(
            new detail::LambdaCommand(name, std::forward<Lambda>(lambda)));
    };

    template<typename Data, typename Lambda, typename... Args>
    void queueLambdaCommandWithParameters_(std::string_view name, Lambda&& lambda, Args&&... args) {
        pendingCommands_.emplace_back(
            new detail::LambdaCommandWithParameters<Data, std::decay_t<Lambda>>(name, std::forward<Lambda>(lambda), std::forward<Args>(args)...));
    };

private:
    SwapChainPtr lastBoundSwapChain_;
    core::Array<geometry::Mat4f> projectionMatrixStack_;
    core::Array<geometry::Mat4f> viewMatrixStack_;
    std::thread renderThread_;

    std::mutex mutex_;
    std::condition_variable wakeRenderThreadConditionVariable_;
    std::condition_variable renderThreadEventConditionVariable_;
    std::atomic_uint64_t lastExecutedCommandListId_ = 0;
    UInt lastSubmittedCommandListId_ = 0;
    bool running_ = false;
    bool stopRequested_ = false;

    struct CommandList {
        CommandList(
            std::list<CommandUPtr>&& commands,
            core::Array<Resource*>&& garbagedResources)
            : commands(std::move(commands))
            , garbagedResources(std::move(garbagedResources))
        {}

        std::list<CommandUPtr> commands;
        core::Array<Resource*> garbagedResources;
    };
    core::Array<CommandList> commandQueue_;

    void renderThreadProc_();
    void startRenderThread_();
    void stopRenderThread_(); // blocking

    UInt submitPendingCommandList_();

    // returns false if translation was cancelled by a stop request.
    void waitCommandListTranslationFinished_(UInt commandListId = 0);
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_ENGINE_H
