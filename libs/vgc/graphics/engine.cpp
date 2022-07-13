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

#include <vgc/graphics/engine.h>

#include <tuple> // std::tuple_size

namespace vgc {
namespace graphics {

Engine::Engine()
    : Object()
    , gcResourceList_(new detail::ResourceList())
{
    projectionMatrixStack_.emplaceLast(geometry::Mat4f::identity);
    viewMatrixStack_.emplaceLast(geometry::Mat4f::identity);
    framebufferStack_.emplaceLast(nullptr);
}

void Engine::pushPipelineParameters(PipelineParameters parameters)
{
    if (!!(parameters & PipelineParameters::Framebuffer)) {
        framebufferStack_.emplaceLast(framebufferStack_.last());
    }
    if (!!(parameters & PipelineParameters::Viewport)) {
        viewportStack_.emplaceLast(viewportStack_.last());
    }
    if (!!(parameters & PipelineParameters::Program)) {
        programStack_.emplaceLast(programStack_.last());
    }
    if (!!(parameters & PipelineParameters::BlendState)) {
        blendStateStack_.emplaceLast(blendStateStack_.last());
    }
    if (!!(parameters & PipelineParameters::DepthStencilState)) {
        // todo
    }
    if (!!(parameters & PipelineParameters::RasterizerState)) {
        rasterizerStateStack_.emplaceLast(rasterizerStateStack_.last());
    }
    if (!!(parameters & PipelineParameters::AllShadersResources)) {
        if (!!(parameters & PipelineParameters::VertexShaderConstantBuffers)) {
            StageConstantBuffersStack& constantBuffersStack =
                constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Vertex)];
            constantBuffersStack.emplaceLast(constantBuffersStack.last());
        }
        if (!!(parameters & PipelineParameters::VertexShaderImageViews)) {
            StageImageViewsStack& imageViewsStack =
                imageViewsStacks_[shaderStageToIndex_(ShaderStage::Vertex)];
            imageViewsStack.emplaceLast(imageViewsStack.last());
        }
        if (!!(parameters & PipelineParameters::VertexShaderSamplers)) {
            StageSamplersStack& samplersStack =
                samplersStacks_[shaderStageToIndex_(ShaderStage::Vertex)];
            samplersStack.emplaceLast(samplersStack.last());
        }
        if (!!(parameters & PipelineParameters::GeometryShaderConstantBuffers)) {
            StageConstantBuffersStack& constantBuffersStack =
                constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Geometry)];
            constantBuffersStack.emplaceLast(constantBuffersStack.last());
        }
        if (!!(parameters & PipelineParameters::GeometryShaderImageViews)) {
            StageImageViewsStack& imageViewsStack =
                imageViewsStacks_[shaderStageToIndex_(ShaderStage::Geometry)];
            imageViewsStack.emplaceLast(imageViewsStack.last());
        }
        if (!!(parameters & PipelineParameters::GeometryShaderSamplers)) {
            StageSamplersStack& samplersStack =
                samplersStacks_[shaderStageToIndex_(ShaderStage::Geometry)];
            samplersStack.emplaceLast(samplersStack.last());
        }
        if (!!(parameters & PipelineParameters::PixelShaderConstantBuffers)) {
            StageConstantBuffersStack& constantBuffersStack =
                constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Pixel)];
            constantBuffersStack.emplaceLast(constantBuffersStack.last());
        }
        if (!!(parameters & PipelineParameters::PixelShaderImageViews)) {
            StageImageViewsStack& imageViewsStack =
                imageViewsStacks_[shaderStageToIndex_(ShaderStage::Pixel)];
            imageViewsStack.emplaceLast(imageViewsStack.last());
        }
        if (!!(parameters & PipelineParameters::PixelShaderSamplers)) {
            StageSamplersStack& samplersStack =
                samplersStacks_[shaderStageToIndex_(ShaderStage::Pixel)];
            samplersStack.emplaceLast(samplersStack.last());
        }
    }
}

void Engine::popPipelineParameters(PipelineParameters parameters)
{
    if (parameters == PipelineParameters::None) {
        return;
    }

    if (!!(parameters & PipelineParameters::Framebuffer)) {
        framebufferStack_.removeLast();
        dirtyPipelineParameters_ |= PipelineParameters::Framebuffer;
    }
    if (!!(parameters & PipelineParameters::Viewport)) {
        viewportStack_.removeLast();
        dirtyPipelineParameters_ |= PipelineParameters::Viewport;
    }
    if (!!(parameters & PipelineParameters::Program)) {
        programStack_.removeLast();
        dirtyPipelineParameters_ |= PipelineParameters::Program;
    }
    if (!!(parameters & PipelineParameters::BlendState)) {
        blendStateStack_.removeLast();
        dirtyPipelineParameters_ |= PipelineParameters::BlendState;
    }
    if (!!(parameters & PipelineParameters::DepthStencilState)) {
        //dirtyPipelineParameters_ |= PipelineParameters::DepthStencilState;
    }
    if (!!(parameters & PipelineParameters::RasterizerState)) {
        rasterizerStateStack_.removeLast();
        dirtyPipelineParameters_ |= PipelineParameters::RasterizerState;
    }
    if (!!(parameters & PipelineParameters::AllShadersResources)) {
        if (!!(parameters & PipelineParameters::VertexShaderConstantBuffers)) {
            StageConstantBuffersStack& constantBuffersStack =
                constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Vertex)];
            constantBuffersStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::VertexShaderConstantBuffers;
        }
        if (!!(parameters & PipelineParameters::VertexShaderImageViews)) {
            StageImageViewsStack& imageViewsStack =
                imageViewsStacks_[shaderStageToIndex_(ShaderStage::Vertex)];
            imageViewsStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::VertexShaderImageViews;
        }
        if (!!(parameters & PipelineParameters::VertexShaderSamplers)) {
            StageSamplersStack& samplersStack =
                samplersStacks_[shaderStageToIndex_(ShaderStage::Vertex)];
            samplersStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::VertexShaderSamplers;
        }
        if (!!(parameters & PipelineParameters::GeometryShaderConstantBuffers)) {
            StageConstantBuffersStack& constantBuffersStack =
                constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Geometry)];
            constantBuffersStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::GeometryShaderConstantBuffers;
        }
        if (!!(parameters & PipelineParameters::GeometryShaderImageViews)) {
            StageImageViewsStack& imageViewsStack =
                imageViewsStacks_[shaderStageToIndex_(ShaderStage::Geometry)];
            imageViewsStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::GeometryShaderImageViews;
        }
        if (!!(parameters & PipelineParameters::GeometryShaderSamplers)) {
            StageSamplersStack& samplersStack =
                samplersStacks_[shaderStageToIndex_(ShaderStage::Geometry)];
            samplersStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::GeometryShaderSamplers;
        }
        if (!!(parameters & PipelineParameters::PixelShaderConstantBuffers)) {
            StageConstantBuffersStack& constantBuffersStack =
                constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Pixel)];
            constantBuffersStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::PixelShaderConstantBuffers;
        }
        if (!!(parameters & PipelineParameters::PixelShaderImageViews)) {
            StageImageViewsStack& imageViewsStack =
                imageViewsStacks_[shaderStageToIndex_(ShaderStage::Pixel)];
            imageViewsStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::PixelShaderImageViews;
        }
        if (!!(parameters & PipelineParameters::PixelShaderSamplers)) {
            StageSamplersStack& samplersStack =
                samplersStacks_[shaderStageToIndex_(ShaderStage::Pixel)];
            samplersStack.removeLast();
            dirtyPipelineParameters_ |= PipelineParameters::PixelShaderSamplers;
        }
    }
}

void Engine::syncState_()
{
    if (dirtyBuiltinConstantBuffer_) {
        BuiltinConstants constants = {};
        constants.projMatrix = projectionMatrixStack_.last();
        constants.viewMatrix = viewMatrixStack_.last();
        constants.engineTimeInMs_ = 0;
        struct CommandParameters {
            Buffer* buffer;
            BuiltinConstants constants;
        };
        queueLambdaCommandWithParameters_<CommandParameters>(
            "updateBuiltinConstantBufferData",
            [](Engine* engine, const CommandParameters& p) {
                engine->updateBufferData_(p.buffer, &p.constants, sizeof(BuiltinConstants));
            },
            builtinConstantsBuffer_.get(), constants);
        dirtyBuiltinConstantBuffer_ = false;
    }

    const PipelineParameters parameters = dirtyPipelineParameters_;
    if (parameters == PipelineParameters::None) {
        return;
    }

    if (!!(parameters & PipelineParameters::Framebuffer)) {
        Framebuffer* framebuffer = framebufferStack_.last().get();
        if (!framebuffer) {
            framebuffer = getDefaultFramebuffer().get();
        }
        queueLambdaCommandWithParameters_<Framebuffer*>(
            "setFramebuffer",
            [](Engine* engine, Framebuffer* framebuffer) {
                engine->setFramebuffer_(framebuffer);
            },
            framebuffer);
    }
    if (!!(parameters & PipelineParameters::Viewport)) {
        const Viewport& vp = viewportStack_.last();
        queueLambdaCommandWithParameters_<Viewport>(
            "setViewport",
            [](Engine* engine, const Viewport& vp) {
                engine->setViewport_(vp.x(), vp.y(), vp.width(), vp.height());
            },
            vp);
    }
    if (!!(parameters & PipelineParameters::Program)) {
        Program* p = programStack_.last().get();
        queueLambdaCommand_(
            "setProgram",
            [=](Engine* engine) {
                engine->setProgram_(p);
            });
    }
    if (!!(parameters & PipelineParameters::BlendState)) {
        BlendState* p = blendStateStack_.last().get();
        queueLambdaCommand_(
            "setBlendState",
            [=](Engine* engine) {
                engine->setBlendState_(p);
            });
    }
    if (!!(parameters & PipelineParameters::DepthStencilState)) {
        //dirtyPipelineParameters_ |= PipelineParameters::DepthStencilState;
    }
    if (!!(parameters & PipelineParameters::RasterizerState)) {
        RasterizerState* p = rasterizerStateStack_.last().get();
        queueLambdaCommand_(
            "setRasterizerState",
            [=](Engine* engine) {
                engine->setRasterizerState_(p);
            });
    }
    if (!!(parameters & PipelineParameters::AllShadersResources)) {
        if (!!(parameters & PipelineParameters::VertexShaderConstantBuffers)) {
            syncStageConstantBuffers_(ShaderStage::Vertex);
        }
        if (!!(parameters & PipelineParameters::VertexShaderImageViews)) {
            syncStageImageViews_(ShaderStage::Vertex);
        }
        if (!!(parameters & PipelineParameters::VertexShaderSamplers)) {
            syncStageSamplers_(ShaderStage::Vertex);
        }
        if (!!(parameters & PipelineParameters::GeometryShaderConstantBuffers)) {
            syncStageConstantBuffers_(ShaderStage::Geometry);
        }
        if (!!(parameters & PipelineParameters::GeometryShaderImageViews)) {
            syncStageImageViews_(ShaderStage::Geometry);
        }
        if (!!(parameters & PipelineParameters::GeometryShaderSamplers)) {
            syncStageSamplers_(ShaderStage::Geometry);
        }
        if (!!(parameters & PipelineParameters::PixelShaderConstantBuffers)) {
            syncStageConstantBuffers_(ShaderStage::Pixel);
        }
        if (!!(parameters & PipelineParameters::PixelShaderImageViews)) {
            syncStageImageViews_(ShaderStage::Pixel);
        }
        if (!!(parameters & PipelineParameters::PixelShaderSamplers)) {
            syncStageSamplers_(ShaderStage::Pixel);
        }
    }

    dirtyPipelineParameters_ = PipelineParameters::None;
}

void Engine::syncStageConstantBuffers_(ShaderStage shaderStage)
{
    static constexpr Int count = static_cast<Int>(std::tuple_size_v<StageConstantBuffers>);
    using Buffers = std::array<Buffer*, count>;
    struct CommandParameters {
        Buffers buffers;
        ShaderStage shaderStage;
    } parameters = {{}, shaderStage};
    const StageConstantBuffers& constantBuffers =
        constantBuffersStacks_[shaderStageToIndex_(shaderStage)].last();
    std::transform(
        constantBuffers.begin(), constantBuffers.end(),
        parameters.buffers.begin(), [](const BufferPtr& p){ return p.get(); });
    queueLambdaCommandWithParameters_<CommandParameters>(
        "setStageConstantBuffers",
        [](Engine* engine, const CommandParameters& p) {
            engine->setStageConstantBuffers_(p.buffers.data(), 0, count, p.shaderStage);
        },
        parameters);
}

void Engine::syncStageImageViews_(ShaderStage shaderStage)
{
    static constexpr Int count = static_cast<Int>(std::tuple_size_v<StageImageViews>);
    using ImageViews = std::array<ImageView*, count>;
    struct CommandParameters {
        ImageViews views;
        ShaderStage shaderStage;
    } parameters = {{}, shaderStage};
    const StageImageViews& imageViews =
        imageViewsStacks_[shaderStageToIndex_(shaderStage)].last();
    std::transform(
        imageViews.begin(), imageViews.end(),
        parameters.views.begin(), [](const ImageViewPtr& p){ return p.get(); });
    queueLambdaCommandWithParameters_<CommandParameters>(
        "setStageImageViews",
        [](Engine* engine, const CommandParameters& p) {
            engine->setStageImageViews_(p.views.data(), 0, count, p.shaderStage);
        },
        parameters);
}

void Engine::syncStageSamplers_(ShaderStage shaderStage)
{
    static constexpr Int count = static_cast<Int>(std::tuple_size_v<StageSamplers>);
    using Samplers = std::array<SamplerState*, count>;
    struct CommandParameters {
        Samplers samplers;
        ShaderStage shaderStage;
    } parameters = {{}, shaderStage};
    const StageSamplers& samplers =
        samplersStacks_[shaderStageToIndex_(shaderStage)].last();
    std::transform(
        samplers.begin(), samplers.end(),
        parameters.samplers.begin(), [](const SamplerStatePtr& p){ return p.get(); });
    queueLambdaCommandWithParameters_<CommandParameters>(
        "setStageSamplers",
        [](Engine* engine, const CommandParameters& p) {
            engine->setStageSamplers_(p.samplers.data(), 0, count, p.shaderStage);
        },
        parameters);
}

// XXX add try/catch ?
void Engine::renderThreadProc_()
{
    std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
    while (1) {
        lock.lock();

        // wait for work or stop request
        wakeRenderThreadConditionVariable_.wait(lock, [&]{ return !commandQueue_.isEmpty() || stopRequested_; });

        // if requested, stop
        if (stopRequested_) {
            // cancel submitted lists
            for (const CommandList& l : commandQueue_) {
                for (Resource* resource : l.garbagedResources) {
                    resource->release_(this);
                }
            }
            commandQueue_.clear();
            lastExecutedCommandListId_ = lastSubmittedCommandListId_;
            // release all resources..
            for (Resource* r : gcResourceList_->danglingResources_) {
                r->release_(this);
                delete r;
            }
            for (Resource* r : gcResourceList_->resources_) {
                r->release_(this);
                r->gcList_ = nullptr;
            }
            gcResourceList_->resources_.clear();
            gcResourceList_->danglingResources_.clear();
            // notify
            lock.unlock();
            renderThreadEventConditionVariable_.notify_all();
            return;
        }

        // else commandQueue_ is not empty, so prepare some work
        CommandList commandList = std::move(commandQueue_.first());
        commandQueue_.removeFirst();

        lock.unlock();

        // execute commands
        for (const CommandUPtr& command : commandList.commands) {
            command->execute(this);
        }

        lock.lock();
        ++lastExecutedCommandListId_;
        lock.unlock();

        renderThreadEventConditionVariable_.notify_all();

        // release garbaged resources
        for (Resource* r : commandList.garbagedResources) {
            r->release_(this);
            delete r;
        }
    }
}

void Engine::startRenderThread_()
{
    if (stopRequested_) {
        throw core::LogicError("Engine: restarts are not supported.");
    }
    if (!running_) {
        renderThread_ = std::thread([=]{ this->renderThreadProc_(); });
        running_ = true;
    }
}

void Engine::stopRenderThread_()
{
    pendingCommands_.clear();
    swapChain_.reset();
    if (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        stopRequested_ = true;
        lock.unlock();
        wakeRenderThreadConditionVariable_.notify_all();
        VGC_CORE_ASSERT(renderThread_.joinable());
        renderThread_.join();
        running_ = false;
        return;
    }
}

UInt Engine::submitPendingCommandList_(bool withGarbage)
{
    std::unique_lock<std::mutex> lock(mutex_);
    bool notifyRenderThread = commandQueue_.isEmpty();
    if (withGarbage) {
        commandQueue_.emplaceLast(std::move(pendingCommands_), std::move(gcResourceList_->danglingResources_));
        pendingCommands_.clear();
        gcResourceList_->danglingResources_.clear();
    }
    else {
        commandQueue_.emplaceLast(std::move(pendingCommands_));
        pendingCommands_.clear();
    }
    UInt id = ++lastSubmittedCommandListId_;
    lock.unlock();
    if (notifyRenderThread) {
        wakeRenderThreadConditionVariable_.notify_all();
    }
    return id;
}

void Engine::waitCommandListTranslationFinished_(UInt commandListId)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (commandListId == 0) {
        commandListId = lastSubmittedCommandListId_;
    }
    renderThreadEventConditionVariable_.wait(lock, [&]{ return lastExecutedCommandListId_ == commandListId; });
    lock.unlock();
}

} // namespace graphics
} // namespace vgc
