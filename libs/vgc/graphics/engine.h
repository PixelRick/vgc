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

#include <array>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <chrono>

#include <vgc/core/assert.h>
#include <vgc/core/color.h>
#include <vgc/core/enum.h>
#include <vgc/core/innercore.h>
#include <vgc/core/templateutil.h>
#include <vgc/geometry/mat4f.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/batch.h>
#include <vgc/graphics/blendstate.h>
#include <vgc/graphics/buffer.h>
#include <vgc/graphics/constants.h>
#include <vgc/graphics/detail/command.h>
#include <vgc/graphics/detail/pipelinestate.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/framebuffer.h>
#include <vgc/graphics/geometryview.h>
#include <vgc/graphics/image.h>
#include <vgc/graphics/imageview.h>
#include <vgc/graphics/logcategories.h>
#include <vgc/graphics/program.h>
#include <vgc/graphics/rasterizerstate.h>
#include <vgc/graphics/resource.h>
#include <vgc/graphics/samplerstate.h>
#include <vgc/graphics/swapchain.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

struct BuiltinConstants {
    geometry::Mat4f projMatrix;
    geometry::Mat4f viewMatrix;
    UInt32 engineTimeInMs_ = 0;
};

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

    void onDestroyed() override
    {
        swapChain_.reset();
        programStack_.clear();
        framebufferStack_.clear();

        colorGradientsBuffer_; // 1D buffer
        colorGradientsBufferImageView_;

        glyphAtlasProgram_.reset();
        glyphAtlasBuffer_.reset();
        glyphAtlasBufferImageView_.reset();

        iconAtlasProgram_.reset();
        iconAtlasImage_.reset();
        iconAtlasImageView_.reset();

        roundedRectangleProgram_.reset();

        // XXX also clear state stacks !

        stopRenderThread_();
    }

public:
    void start()
    {
        createBuiltinResources_();
        startRenderThread_();
    }

    // USER THREAD functions

    /// Submits the current command list for execution by the render thread.
    /// Returns the index assigned to the submitted command list.
    ///
    /// If the current command list is empty, `flush()` does nothing and
    /// returns the index of the previous list.
    ///
    UInt flush()
    {
        return submitPendingCommandList_(true);
    }

    /// Submits the current command list if present then waits for all submitted
    /// command lists to finish being translated to GPU commands.
    ///
    void finish()
    {
        UInt id = submitPendingCommandList_(true);
        waitCommandListTranslationFinished_(id);
    }

    /// Creates a swap chain for the given window.
    ///
    SwapChainPtr createSwapChain(const SwapChainCreateInfo& desc)
    {
        return SwapChainPtr(createSwapChain_(desc));
    }

    void resizeSwapChain(SwapChain* swapChain, UInt32 width, UInt32 height)
    {
        if (swapChain->gcList_ != gcResourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }
        finish();
        resizeSwapChain_(swapChain, width, height);
    }

    void setSwapChain(const SwapChainPtr& swapChain)
    {
        if (swapChain->gcList_ != gcResourceList_) {
            // XXX error, using a resource from another engine..
            return;
        }
        swapChain_ = swapChain;
        queueLambdaCommandWithParameters_<SwapChain*>(
            "bindSwapChain",
            [](Engine* engine, SwapChain* swapChain) {
                engine->setSwapChain_(swapChain);
            },
            swapChain.get());
        if (!areBuiltinResourcesInited_) {
            queueLambdaCommand_(
                "initBuiltinResources",
                [](Engine* engine) {
                    engine->initBuiltinResources_();
                });
            areBuiltinResourcesInited_ = true;
        }
    }

    // presentedCallback is called from an unspecified thread.
    void present(UInt32 syncInterval,
                 std::function<void(UInt64 /*timestamp*/)>&& presentedCallback,
                 PresentFlags flags = PresentFlags::None)
    {
        ++swapChain_->pendingPresentCount_;
        bool shouldSync = syncInterval > 0;
        if (shouldSync && shouldPresentWaitFromSyncedUserThread_()) {
            // Preventing dead-locks
            // See https://docs.microsoft.com/en-us/windows/win32/api/DXGI1_2/nf-dxgi1_2-idxgiswapchain1-present1#remarks
            finish();
            UInt64 timestamp = present_(swapChain_.get(), syncInterval, flags);
            --swapChain_->pendingPresentCount_;
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
                swapChain_.get(), syncInterval, flags, std::move(presentedCallback));

            if (shouldSync) {
                finish();
            }
            else {
                submitPendingCommandList_(true);
            }
        }
    }

    FramebufferPtr getDefaultFramebuffer()
    {
        SwapChain* swapChain = swapChain_.get();
        return swapChain ? swapChain->defaultFrameBuffer_ : FramebufferPtr();
    }

    void setFramebuffer(const FramebufferPtr& framebuffer = nullptr)
    {
        if (framebuffer && !checkResourceIsValid(framebuffer.get())) {
            return;
        }
        framebufferStack_.emplaceLast(framebuffer);
        dirtyPipelineParameters_ |= PipelineParameters::Framebuffer;
    }

    void pushPipelineParameters(PipelineParameters parameters);
    void popPipelineParameters(PipelineParameters parameters);

    void setViewport(Int x, Int y, Int width, Int height)
    {
        viewportStack_.emplaceLast(x, y, width, height);
        dirtyPipelineParameters_ |= PipelineParameters::Viewport;
    }

    /// Returns the current projection matrix (top-most on the stack).
    ///
    geometry::Mat4f projectionMatrix() const
    {
        return projectionMatrixStack_.last();
    }

    /// Assigns `m` to the top-most matrix of the projection matrix stack.
    /// `m` becomes the current projection matrix.
    ///
    void setProjectionMatrix(const geometry::Mat4f& projectionMatrix)
    {
        projectionMatrixStack_.last() = projectionMatrix;
        dirtyBuiltinConstantBuffer_ = true;
    }

    /// Duplicates the top-most matrix on the projection matrix stack.
    ///
    void pushProjectionMatrix()
    {
        projectionMatrixStack_.emplaceLast(projectionMatrixStack_.last());
    }

    /// Removes the top-most matrix of the projection matrix stack.
    /// The new top-most matrix becomes the current projection matrix.
    ///
    /// The behavior is undefined if there is only one matrix in the stack
    /// before calling this function.
    ///
    void popProjectionMatrix()
    {
        projectionMatrixStack_.removeLast();
        dirtyBuiltinConstantBuffer_ = true;
    }

    /// Returns the current view matrix (top-most on the stack).
    ///
    geometry::Mat4f viewMatrix() const
    {
        return viewMatrixStack_.last();
    }

    /// Assigns `m` to the top-most matrix of the view matrix stack.
    /// `m` becomes the current view matrix.
    ///
    void setViewMatrix(const geometry::Mat4f& viewMatrix)
    {
        viewMatrixStack_.last() = viewMatrix;
        dirtyBuiltinConstantBuffer_ = true;
    }

    /// Duplicates the top-most matrix on the view matrix stack.
    ///
    void pushViewMatrix()
    {
        viewMatrixStack_.emplaceLast(viewMatrixStack_.last());
    }

    /// Removes the top-most matrix of the view matrix stack.
    /// The new top-most matrix becomes the current view matrix.
    ///
    /// The behavior is undefined if there is only one matrix in the stack
    /// before calling this function.
    ///
    void popViewMatrix()
    {
        viewMatrixStack_.removeLast();
        dirtyBuiltinConstantBuffer_ = true;
    }

    void bindProgram(BuiltinProgram builtinProgram)
    {
        ProgramPtr program = {};
        switch (builtinProgram) {
        case BuiltinProgram::Simple: {
            program = simpleProgram_; break;
        }
        }
        programStack_.emplaceLast(program);
        dirtyPipelineParameters_ |= PipelineParameters::Program;
    }

    template<typename DataGetter>
    BufferPtr createBuffer(const BufferCreateInfo& createInfo, DataGetter&& initialDataGetter, Int initialLengthInBytes)
    {
        if (initialLengthInBytes < 0) {
            throw core::NegativeIntegerError(core::format(
                "Negative initialLengthInBytes ({}) provided to Engine::createBuffer()", initialLengthInBytes));
        }

        BufferPtr buffer(createBuffer_(createInfo));

        struct CommandParameters {
            Buffer* buffer;
            std::decay_t<DataGetter> initialDataGetter;
            Int initialLengthInBytes;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "initBuffer",
            [](Engine* engine, const CommandParameters& p) {
                engine->initBuffer_(p.buffer, p.initialDataGetter(), p.initialLengthInBytes);
            },
            buffer.get(), std::move(initialDataGetter), initialLengthInBytes);
        return buffer;
    }

    template<typename DataGetter>
    void updateBufferData(Buffer* buffer, DataGetter&& initialDataGetter, Int lengthInBytes)
    {
        if (lengthInBytes < 0) {
            throw core::NegativeIntegerError(core::format(
                "Negative lengthInBytes ({}) provided to vgc::graphics::Engine::updateBufferData()", lengthInBytes));
        }

        if (!checkResourceIsValid(buffer)) {
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
    BufferPtr createVertexBuffer(DataGetter&& initialDataGetter, Int initialLengthInBytes, bool dynamic)
    {
        BufferCreateInfo createInfo = {};
        createInfo.setUsage(dynamic ? Usage::Dynamic : Usage::Immutable);
        createInfo.setBindFlags(BindFlags::VertexBuffer);
        createInfo.setCpuAccessFlags(dynamic ? CpuAccessFlags::Write : CpuAccessFlags::None);
        createInfo.setResourceMiscFlags(ResourceMiscFlags::None);
        return createBuffer(createInfo, std::move(initialDataGetter), initialLengthInBytes);
    }

    template<typename DataGetter>
    ImagePtr createImage(const ImageCreateInfo& createInfo, DataGetter&& initialDataGetter)
    {
        ImagePtr image(createImage_(createInfo));

        struct CommandParameters {
            Image* image;
            std::decay_t<DataGetter> initialDataGetter;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "initImage",
            [](Engine* engine, const CommandParameters& p) {
                engine->initImage_(p.image, p.initialDataGetter());
            },
            image.get(), std::move(initialDataGetter));
        return image;
    }

    GeometryViewPtr createGeometryView(const GeometryViewCreateInfo& createInfo)
    {
        GeometryViewPtr geometryView(createGeometryView_(createInfo));

        queueLambdaCommandWithParameters_<GeometryView*>(
            "initImage",
            [](Engine* engine, GeometryView* gv) {
                engine->initGeometryView_(gv);
            },
            geometryView.get());

        return geometryView;
    }

    void draw(const GeometryViewPtr& geometryView)
    {
        if (!checkResourceIsValid(geometryView.get())) {
            return;
        }
        syncState_();
        queueLambdaCommandWithParameters_<GeometryView*>(
            "draw",
            [](Engine* engine, GeometryView* gv) {
                engine->draw_(gv);
            },
            geometryView.get());
    }

    /// Clears the whole render area with the given color.
    ///
    void clear(const core::Color& color)
    {
        syncState_();
        queueLambdaCommandWithParameters_<core::Color>(
            "clear",
            [](Engine* engine, const core::Color& c) {
                engine->clear_(c);
            },
            color);
    }

protected:
    // USER THREAD implementation functions

    virtual SwapChain* createSwapChain_(const SwapChainCreateInfo& createInfo) = 0;
    virtual void resizeSwapChain_(SwapChain* swapChain, UInt32 width, UInt32 height) = 0;

    // XXX virtual void setDefaultFramebuffer_(const SwapChainPtr& swapChain, const FramebufferPtr& fb) = 0;

    virtual Buffer* createBuffer_(const BufferCreateInfo& createInfo) = 0;
    virtual Image* createImage_(const ImageCreateInfo& createInfo) = 0;
    virtual ImageView* createImageView_(const ImageViewCreateInfo& createInfo, const ImagePtr& image) = 0;
    virtual ImageView* createImageView_(const ImageViewCreateInfo& createInfo, const BufferPtr& buffer, ImageFormat format, UInt32 elementsCount) = 0;
    virtual GeometryView* createGeometryView_(const GeometryViewCreateInfo& createInfo) = 0;
    virtual BlendState* createBlendState_(const BlendStateCreateInfo& createInfo) = 0;
    virtual RasterizerState* createRasterizerState_(const RasterizerStateCreateInfo& createInfo) = 0;
    virtual Framebuffer* createFramebuffer_(const ImageViewPtr& colorImageView) = 0;

    virtual bool shouldPresentWaitFromSyncedUserThread_() { return false; }

    // RENDER THREAD implementation functions

    virtual void initBuiltinResources_() = 0;

    virtual void setSwapChain_(SwapChain* swapChain) = 0;
    virtual UInt64 present_(SwapChain* swapChain, UInt32 syncInterval, PresentFlags flags) = 0;

    virtual void initBuffer_(Buffer* buffer, const void* data, Int initialLengthInBytes) = 0;
    virtual void initImage_(Image* image, const void* data) = 0;
    virtual void initImageView_(ImageView* view) = 0;
    virtual void initGeometryView_(GeometryView* view) = 0;
    virtual void initBlendState_(BlendState* state) = 0;
    virtual void initRasterizerState_(RasterizerState* state) = 0;
    virtual void initFramebuffer_(Framebuffer* framebuffer) = 0;

    virtual void setViewport_(Int x, Int y, Int width, Int height) = 0;
    virtual void setProgram_(Program* program) = 0;
    virtual void setBlendState_(BlendState* state) = 0;
    virtual void setRasterizerState_(RasterizerState* state) = 0;
    virtual void setStageConstantBuffers_(Buffer* const* buffers, Int startIndex, Int count, ShaderStage shaderStage) = 0;
    virtual void setStageImageViews_(ImageView* const* views, Int startIndex, Int count, ShaderStage shaderStage) = 0;
    virtual void setStageSamplers_(SamplerState* const* states, Int startIndex, Int count, ShaderStage shaderStage) = 0;
    virtual void setFramebuffer_(Framebuffer* framebuffer) = 0;

    virtual void updateBufferData_(Buffer* buffer, const void* data, Int lengthInBytes) = 0;

    virtual void draw_(GeometryView* view) = 0;
    virtual void clear_(const core::Color& color) = 0;

protected:
    bool areBuiltinResourcesInited_ = false;
    detail::ResourceList* gcResourceList_ = nullptr;
    // cannot be flushed in out-of-order chunks unless garbagedResources is only sent with the last
    std::list<CommandUPtr> pendingCommands_;

    // QUEUING

    template<typename TCommand, typename... Args>
    void queueCommand_(Args&&... args)
    {
        pendingCommands_.emplace_back(
            new TCommand(std::forward<Args>(args)...));
    };

    template<typename Lambda>
    void queueLambdaCommand_(std::string_view name, Lambda&& lambda)
    {
        pendingCommands_.emplace_back(
            new detail::LambdaCommand(name, std::forward<Lambda>(lambda)));
    };

    template<typename Data, typename Lambda, typename... Args>
    void queueLambdaCommandWithParameters_(std::string_view name, Lambda&& lambda, Args&&... args)
    {
        pendingCommands_.emplace_back(
            new detail::LambdaCommandWithParameters<Data, std::decay_t<Lambda>>(name, std::forward<Lambda>(lambda), std::forward<Args>(args)...));
    };

private:
    // -- pipeline state on the user thread --

    SwapChainPtr swapChain_;
    core::Array<FramebufferPtr> framebufferStack_;

    // pushable pipeline parameters
    core::Array<Viewport> viewportStack_;
    core::Array<ProgramPtr> programStack_;
    core::Array<BlendStatePtr> blendStateStack_;
    core::Array<RasterizerStatePtr> rasterizerStateStack_;

    static constexpr size_t shaderStageToIndex_(ShaderStage stage)
    {
        return core::toUnderlying(stage);
    }

    static constexpr Int stageEndIndex = core::toUnderlying(ShaderStage::Max_) + 1;

    using StageConstantBuffers = std::array<BufferPtr, maxConstantBufferCountPerStage>;
    using StageConstantBuffersStack = core::Array<StageConstantBuffers>;
    std::array<StageConstantBuffersStack, stageEndIndex> constantBuffersStacks_;

    using StageImageViews = std::array<ImageViewPtr, maxImageViewCountPerStage>;
    using StageImageViewsStack = core::Array<StageImageViews>;
    std::array<StageImageViewsStack, stageEndIndex> imageViewsStacks_;

    using StageSamplers = std::array<SamplerStatePtr, maxSamplerCountPerStage>;
    using StageSamplersStack = core::Array<StageSamplers>;
    std::array<StageSamplersStack, stageEndIndex> samplersStacks_;

    PipelineParameters dirtyPipelineParameters_ = PipelineParameters::None;

    // called in user thread
    void syncState_();
    void syncStageConstantBuffers_(ShaderStage shaderStage);
    void syncStageImageViews_(ShaderStage shaderStage);
    void syncStageSamplers_(ShaderStage shaderStage);

    // -- builtin shaders -- (create by api-specific engine implementations)

    void createBuiltinResources_();

    ProgramPtr simpleProgram_;

    // -- builtin constants + dirty bool --

    BufferPtr builtinConstantsBuffer_;
    core::Array<geometry::Mat4f> projectionMatrixStack_;
    core::Array<geometry::Mat4f> viewMatrixStack_;
    UInt32 engineTimeInMs_ = 0;
    bool dirtyBuiltinConstantBuffer_ = false;

    // -- builtin batching early impl --

    BufferPtr unitQuad_;

    BufferPtr colorGradientsBuffer_; // 1D buffer
    ImageViewPtr colorGradientsBufferImageView_;

    ProgramPtr glyphAtlasProgram_;
    BufferPtr glyphAtlasBuffer_; // 1D layered
    ImageViewPtr glyphAtlasBufferImageView_;
    BufferPtr textBatch_;

    ProgramPtr iconAtlasProgram_;
    ImagePtr iconAtlasImage_; // 2D
    ImageViewPtr iconAtlasImageView_;

    ProgramPtr roundedRectangleProgram_;

    void flushBuiltinBatches_();
    void prependBuiltinBatchesResourceUpdates_();

    // -- render thread + sync --

    std::thread renderThread_;
    std::mutex mutex_;
    std::condition_variable wakeRenderThreadConditionVariable_;
    std::condition_variable renderThreadEventConditionVariable_;
    UInt lastExecutedCommandListId_ = 0;
    UInt lastSubmittedCommandListId_ = 0;
    bool running_ = false;
    bool stopRequested_ = false;

    struct CommandList {
        CommandList(std::list<CommandUPtr>&& commands,
                    core::Array<Resource*>&& garbagedResources)
            : commands(std::move(commands))
            , garbagedResources(std::move(garbagedResources))
        {
        }

        CommandList(std::list<CommandUPtr>&& commands)
            : commands(std::move(commands))
        {
        }

        std::list<CommandUPtr> commands;
        core::Array<Resource*> garbagedResources;
    };
    core::Array<CommandList> commandQueue_;

    void renderThreadProc_();
    void startRenderThread_();
    void stopRenderThread_(); // blocking

    // -- threads sync --

    UInt submitPendingCommandList_(bool withGarbage);

    // returns false if translation was cancelled by a stop request.
    void waitCommandListTranslationFinished_(UInt commandListId = 0);

    // -- checks --

    bool checkResourceIsValid(Resource* resource)
    {
        if (!resource) {
            VGC_ERROR(LogVgcGraphics, "Unexpected null resource");
            return false;
        }
        if (!resource->gcList_) {
            VGC_ERROR(LogVgcGraphics, "Trying to use a resource from a stopped engine");
            return false;
        }
        if (resource->gcList_ != gcResourceList_) {
            VGC_ERROR(LogVgcGraphics, "Trying to use a geometry view from an other engine");
            return false;
        }
        return true;
    }
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_ENGINE_H
