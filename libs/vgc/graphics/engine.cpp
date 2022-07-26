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
    framebufferStack_.emplaceLast();

    viewportStack_.emplaceLast(0, 0, 0, 0);
    programStack_.emplaceLast();
    blendStateStack_.emplaceLast();
    blendConstantFactorStack_.emplaceLast();
    rasterizerStateStack_.emplaceLast();

    for (Int i = 0; i < stageEndIndex_; ++i) {
        constantBuffersStacks_[i].emplaceLast();
        imageViewsStacks_[i].emplaceLast();
        samplersStacks_[i].emplaceLast();
    }

    projectionMatrixStack_.emplaceLast(geometry::Mat4f::identity);
    viewMatrixStack_.emplaceLast(geometry::Mat4f::identity);
}

void Engine::onDestroyed()
{
    swapChain_.reset();

    framebufferStack_.clear();

    viewportStack_.clear();
    programStack_.clear();
    blendStateStack_.clear();
    blendConstantFactorStack_.clear();
    rasterizerStateStack_.clear();

    for (Int i = 0; i < stageEndIndex_; ++i) {
        constantBuffersStacks_[i].clear();
        imageViewsStacks_[i].clear();
        samplersStacks_[i].clear();
    }

    projectionMatrixStack_.clear();
    viewMatrixStack_.clear();

    colorGradientsBuffer_; // 1D buffer
    colorGradientsBufferImageView_;

    glyphAtlasProgram_.reset();
    glyphAtlasBuffer_.reset();
    glyphAtlasBufferImageView_.reset();

    iconAtlasProgram_.reset();
    iconAtlasImage_.reset();
    iconAtlasImageView_.reset();

    roundedRectangleProgram_.reset();

    stopRenderThread_();
}

void Engine::start()
{
    engineStartTime_ = std::chrono::steady_clock::now();
    createBuiltinResources_();
    startRenderThread_();
}

SwapChainPtr Engine::createSwapChain(const SwapChainCreateInfo& desc)
{
    return constructSwapChain_(desc);
}

FramebufferPtr Engine::createFramebuffer(const ImageViewPtr& colorImageView)
{
    // XXX should check bind flags compatibility here

    FramebufferPtr framebuffer = constructFramebuffer_(colorImageView);
    queueLambdaCommandWithParameters_<Framebuffer*>(
        "initFramebuffer",
        [](Engine* engine, Framebuffer* p) {
            engine->initFramebuffer_(p);
        },
        framebuffer.get());
    return framebuffer;
}

BufferPtr Engine::createBuffer(const BufferCreateInfo& createInfo, Int initialLengthInBytes)
{
    if (initialLengthInBytes < 0) {
        throw core::NegativeIntegerError(core::format(
            "Negative initialLengthInBytes ({}) provided to Engine::createBuffer()", initialLengthInBytes));
    }

    BufferPtr buffer(constructBuffer_(createInfo));
    buffer->lengthInBytes_ = initialLengthInBytes;

    struct CommandParameters {
        Buffer* buffer;
        Int initialLengthInBytes;
    };
    queueLambdaCommandWithParameters_<CommandParameters>(
        "initBufferZeroed",
        [](Engine* engine, const CommandParameters& p) {
            engine->initBuffer_(p.buffer, nullptr, p.initialLengthInBytes);
        },
        buffer.get(), initialLengthInBytes);
    return buffer;
}

BufferPtr Engine::createVertexBuffer(Int initialLengthInBytes)
{
    if (initialLengthInBytes < 0) {
        throw core::NegativeIntegerError(core::format(
            "Negative initialLengthInBytes ({}) provided to Engine::createVertexBuffer()", initialLengthInBytes));
    }

    BufferCreateInfo createInfo = {};
    createInfo.setUsage(Usage::Dynamic);
    createInfo.setBindFlags(BindFlags::VertexBuffer);
    createInfo.setCpuAccessFlags(CpuAccessFlags::Write);
    createInfo.setResourceMiscFlags(ResourceMiscFlags::None);
    return createBuffer(createInfo, initialLengthInBytes);
}

GeometryViewPtr Engine::createDynamicTriangleListView(BuiltinGeometryLayout vertexLayout)
{
    BufferPtr vertexBuffer = createVertexBuffer(0);
    GeometryViewCreateInfo createInfo = {};
    createInfo.setBuiltinGeometryLayout(vertexLayout);
    createInfo.setPrimitiveType(PrimitiveType::TriangleList);
    createInfo.setVertexBuffer(0, vertexBuffer);
    return createGeometryView(createInfo);
}

//ImagePtr Engine::createImage(const ImageCreateInfo& createInfo)
//ImagePtr Engine::createImage(const ImageCreateInfo& createInfo, core::Array<char> initialData)

ImagePtr Engine::createImage(const ImageCreateInfo& createInfo)
{
    ImagePtr image(constructImage_(createInfo));

    struct CommandParameters {
        Image* image;
    };
    queueLambdaCommandWithParameters_<CommandParameters>(
        "initImage",
        [](Engine* engine, const CommandParameters& p) {
            engine->initImage_(p.image, nullptr);
        },
        image.get());
    return image;
}

ImagePtr Engine::createImage(const ImageCreateInfo& createInfo, core::Array<char> initialData)
{
    ImagePtr image(constructImage_(createInfo));

    struct CommandParameters {
        Image* image;
        core::Array<char> initialData;
    };
    queueLambdaCommandWithParameters_<CommandParameters>(
        "initImage",
        [](Engine* engine, const CommandParameters& p) {
            Span<const char> l0m0 = { p.initialData.data(), p.initialData.length() };
            Span<const Span<const char>> imgs = { &l0m0, 1 };
            engine->initImage_(p.image, &imgs);
        },
        image.get(), std::move(initialData));
    return image;
}

ImageViewPtr Engine::createImageView(const ImageViewCreateInfo& createInfo, const ImagePtr& image)
{
    // XXX should check bind flags compatibility here

    ImageViewPtr imageView = constructImageView_(createInfo, image);
    queueLambdaCommandWithParameters_<ImageView*>(
        "initImageView",
        [](Engine* engine, ImageView* p) {
            engine->initImageView_(p);
        },
        imageView.get());
    return imageView;
}

ImageViewPtr Engine::createImageView(const ImageViewCreateInfo& createInfo, const BufferPtr& buffer, ImageFormat format, UInt32 elementsCount)
{
    // XXX should check bind flags compatibility here

    ImageViewPtr imageView = constructImageView_(createInfo, buffer, format, elementsCount);
    queueLambdaCommandWithParameters_<ImageView*>(
        "initBufferImageView",
        [](Engine* engine, ImageView* p) {
            engine->initImageView_(p);
        },
        imageView.get());
    return imageView;
}

SamplerStatePtr Engine::createSamplerState(const SamplerStateCreateInfo& createInfo)
{
    SamplerStatePtr samplerState = constructSamplerState_(createInfo);
    queueLambdaCommandWithParameters_<SamplerState*>(
        "initSamplerState",
        [](Engine* engine, SamplerState* p) {
            engine->initSamplerState_(p);
        },
        samplerState.get());
    return samplerState;
}

GeometryViewPtr Engine::createGeometryView(const GeometryViewCreateInfo& createInfo)
{
    GeometryViewPtr geometryView = constructGeometryView_(createInfo);
    queueLambdaCommandWithParameters_<GeometryView*>(
        "initGeometryView",
        [](Engine* engine, GeometryView* p) {
            engine->initGeometryView_(p);
        },
        geometryView.get());
    return geometryView;
}

BlendStatePtr Engine::createBlendState(const BlendStateCreateInfo& createInfo)
{
    BlendStatePtr blendState = constructBlendState_(createInfo);
    queueLambdaCommandWithParameters_<BlendState*>(
        "initBlendState",
        [](Engine* engine, BlendState* p) {
            engine->initBlendState_(p);
        },
        blendState.get());
    return blendState;
}

RasterizerStatePtr Engine::createRasterizerState(const RasterizerStateCreateInfo& createInfo)
{
    RasterizerStatePtr rasterizerState = constructRasterizerState_(createInfo);
    queueLambdaCommandWithParameters_<RasterizerState*>(
        "initRasterizerState",
        [](Engine* engine, RasterizerState* p) {
            engine->initRasterizerState_(p);
        },
        rasterizerState.get());
    return rasterizerState;
}

void Engine::setSwapChain(const SwapChainPtr& swapChain)
{
    if (swapChain->gcList_ != gcResourceList_) {
        // XXX error, using a resource from another engine..
        return;
    }
    swapChain_ = swapChain;
    frameStartTime_ = std::chrono::steady_clock::now();
    dirtyBuiltinConstantBuffer_ = true;
    queueLambdaCommandWithParameters_<SwapChain*>(
        "setSwapChain",
        [](Engine* engine, SwapChain* swapChain) {
            engine->setSwapChain_(swapChain);
        },
        swapChain.get());
}

void Engine::setFramebuffer(const FramebufferPtr& framebuffer)
{
    if (framebuffer && !checkResourceIsValid_(framebuffer.get())) {
        return;
    }
    if (framebufferStack_.last() != framebuffer) {
        framebufferStack_.last() = framebuffer;
        dirtyPipelineParameters_ |= PipelineParameters::Framebuffer;
    }
}

void Engine::setViewport(Int x, Int y, Int width, Int height)
{
    viewportStack_.last() = Viewport(x, y, width, height);
    dirtyPipelineParameters_ |= PipelineParameters::Viewport;
}

void Engine::setProgram(BuiltinProgram builtinProgram)
{
    ProgramPtr program = {};
    switch (builtinProgram) {
    case BuiltinProgram::Simple: {
        program = simpleProgram_;
        break;
    }
    }
    if (programStack_.last() != program) {
        programStack_.last() = program;
        if (true /*program->usesBuiltinConstants()*/) {
            BufferPtr& constantBufferRef = constantBuffersStacks_[shaderStageToIndex_(ShaderStage::Vertex)].last()[0];
            if (constantBufferRef != builtinConstantsBuffer_) {
                constantBufferRef = builtinConstantsBuffer_;
                dirtyPipelineParameters_ |= PipelineParameters::VertexShaderConstantBuffers;
            }
        }
        dirtyPipelineParameters_ |= PipelineParameters::Program;
    }
}

void Engine::setBlendState(const BlendStatePtr& state, const geometry::Vec4f& blendConstantFactor)
{
    if (blendStateStack_.last() != state) {
        blendStateStack_.last() = state;
        dirtyPipelineParameters_ |= PipelineParameters::BlendState;
    }
    if (blendConstantFactorStack_.last() != blendConstantFactor) {
        blendConstantFactorStack_.last() = blendConstantFactor;
        dirtyPipelineParameters_ |= PipelineParameters::BlendState;
    }
}

void Engine::setRasterizerState(const RasterizerStatePtr& state)
{
    if (rasterizerStateStack_.last() != state) {
        rasterizerStateStack_.last() = state;
        dirtyPipelineParameters_ |= PipelineParameters::RasterizerState;
    }
}

void Engine::setStageConstantBuffers(const BufferPtr* buffers, Int startIndex, Int count, ShaderStage shaderStage)
{
    size_t stageIndex = shaderStageToIndex_(shaderStage);
    StageConstantBuffersStack& constantBuffersStack = constantBuffersStacks_[stageIndex];
    StageConstantBuffers& constantBuffers = constantBuffersStack.emplaceLast();
    for (Int i = 0; i < count; ++i) {
        constantBuffers[startIndex + i] = buffers[i];
    }
    dirtyPipelineParameters_ |= std::array{
        PipelineParameters::VertexShaderConstantBuffers,
        PipelineParameters::GeometryShaderConstantBuffers,
        PipelineParameters::PixelShaderConstantBuffers
    }[stageIndex];
}

void Engine::setStageImageViews(const ImageViewPtr* views, Int startIndex, Int count, ShaderStage shaderStage)
{
    size_t stageIndex = shaderStageToIndex_(shaderStage);
    StageImageViewsStack& imageViewsStack = imageViewsStacks_[stageIndex];
    StageImageViews& imageViews = imageViewsStack.emplaceLast();
    for (Int i = 0; i < count; ++i) {
        imageViews[startIndex + i] = views[i];
    }
    dirtyPipelineParameters_ |= std::array{
        PipelineParameters::VertexShaderImageViews,
        PipelineParameters::GeometryShaderImageViews,
        PipelineParameters::PixelShaderImageViews
    }[stageIndex];
}

void Engine::setStageSamplers(const SamplerStatePtr* states, Int startIndex, Int count, ShaderStage shaderStage)
{
    size_t stageIndex = shaderStageToIndex_(shaderStage);
    StageSamplersStack& samplersStack = samplersStacks_[stageIndex];
    StageSamplers& samplers = samplersStack.emplaceLast();
    for (Int i = 0; i < count; ++i) {
        samplers[startIndex + i] = states[i];
    }
    dirtyPipelineParameters_ |= std::array{
        PipelineParameters::VertexShaderSamplers,
        PipelineParameters::GeometryShaderSamplers,
        PipelineParameters::PixelShaderSamplers
    }[stageIndex];
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
        constants.frameStartTimeInMs = core::int_cast<UInt32>(std::chrono::duration_cast<std::chrono::milliseconds>(
            frameStartTime_ - engineStartTime_).count());
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
        const geometry::Vec4f& blendConstantFactor = blendConstantFactorStack_.last();
        queueLambdaCommand_(
            "setBlendState",
            [=](Engine* engine) {
                engine->setBlendState_(p, blendConstantFactor);
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

void Engine::resizeSwapChain(const SwapChainPtr& swapChain, UInt32 width, UInt32 height)
{
    if (swapChain->gcList_ != gcResourceList_) {
        // XXX error, using a resource from another engine..
        return;
    }
    finish();
    resizeSwapChain_(swapChain.get(), width, height);
}

void Engine::draw(const GeometryViewPtr& geometryView, Int indexCount, UInt instanceCount)
{
    if (!checkResourceIsValid_(geometryView.get())) {
        return;
    }
    if (indexCount == 0) {
        return;
    }
    syncState_();
    queueLambdaCommandWithParameters_<GeometryView*>(
        "draw",
        [=](Engine* engine, GeometryView* gv) {
            Int n = (indexCount >= 0) ? indexCount : gv->vertexCount();
            engine->draw_(gv, static_cast<UInt>(n), instanceCount);
        },
        geometryView.get());
}

void Engine::clear(const core::Color& color)
{
    syncState_();
    queueLambdaCommandWithParameters_<core::Color>(
        "clear",
        [](Engine* engine, const core::Color& c) {
            engine->clear_(c);
        },
        color);
}

void Engine::present(UInt32 syncInterval,
                     std::function<void(UInt64 /*timestamp*/)>&& presentedCallback,
                     PresentFlags flags)
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


void Engine::createBuiltinResources_()
{
    {
        BufferCreateInfo createInfo = {};
        createInfo.setUsage(Usage::Dynamic);
        createInfo.setBindFlags(BindFlags::ConstantBuffer);
        createInfo.setCpuAccessFlags(CpuAccessFlags::Write);
        builtinConstantsBuffer_ = createBuffer(createInfo, sizeof(BuiltinConstants));
    }

    createBuiltinShaders_();
}

// -- render thread + sync --

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
