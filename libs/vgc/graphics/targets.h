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

#ifndef VGC_GRAPHICS_TARGETS_H
#define VGC_GRAPHICS_TARGETS_H

#include <unordered_set>
#include <atomic>

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/buffer.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

enum class WindowNativeHandleType : uint8_t {
    None = 0,
    Win32,
    QOpenGLWindow,
};

class VGC_GRAPHICS_API SwapChainDesc {
public:
    SwapChainDesc() = default;

    UInt32 width() const {
        return width_;
    }

    void setWidth(UInt32 width) {
        width_ = width;
    }

    UInt32 height() const {
        return height_;
    }

    void setHeight(UInt32 height) {
        height_ = height;
    }

    SwapChainTargetFormat format() const {
        return format_;
    }

    void setFormat(SwapChainTargetFormat format) {
        format_ = format;
    }

    void* windowNativeHandle() const {
        return windowNativeHandle_;
    }

    WindowNativeHandleType windowNativeHandleType() const {
        return windowNativeHandleType_;
    }

    void setWindowNativeHandle(void* windowNativeHandle, WindowNativeHandleType windowNativeHandleType) {
        windowNativeHandle_ = windowNativeHandle;
        windowNativeHandleType_ = windowNativeHandleType;
    }

    UInt32 windowed() const {
        return windowed_;
    }

    void setWindowed(UInt32 windowed) {
        windowed_ = windowed;
    }

    UInt8 sampleCount() const {
        return sampleCount_;
    }

    void setSampleCount(UInt8 sampleCount) {
        sampleCount_ = sampleCount;
    }

    UInt8 bufferCount() const {
        return bufferCount_;
    }

    void setBufferCount(UInt8 bufferCount) {
        bufferCount_ = bufferCount;
    }

    UInt flags() const {
        return flags_;
    }

    void setFlags(UInt flags) {
        flags_ = flags;
    }

private:
    UInt32 width_;
    UInt32 height_;
    SwapChainTargetFormat format_;
    // XXX add support for sample quality ?
    void* windowNativeHandle_;
    WindowNativeHandleType windowNativeHandleType_;
    bool windowed_;
    UInt8 sampleCount_;
    UInt8 bufferCount_;
    UInt flags_;
};

/// \class vgc::graphics::ImageView
/// \brief Abstract view of an image buffer attachable to some stage of the graphics pipeline.
///
// Since a swap chain's render target view represents different buffers over
// time, a Vulkan implementation should probably cache a view for each
// back buffer.
//
// Concept mapping:
//  D3D11  -> Shader Resource View (SRV)
//  OpenGL -> Texture
//  Vulkan -> Image View
// Looks like all three support buffers as image.
//
class VGC_GRAPHICS_API ImageView : public Resource {
protected:
    ImageView(ResourceList* owningList)
        : Resource(owningList) {}

    using Resource::Resource;
};
using ImageViewPtr = ResourcePtr<ImageView>;

/// \class vgc::graphics::Framebuffer
/// \brief Abstract framebuffer, a collection of graphics pipeline attachments.
///
// Since a swap chain's render target view represents different buffers over
// time, a Vulkan implementation should probably cache a vkFramebuffer for each
// back buffer.
//
class VGC_GRAPHICS_API Framebuffer : public Resource {
protected:
    Framebuffer(ResourceList* owningList)
        : Resource(owningList) {}

    using Resource::Resource;
};
using FramebufferPtr = ResourcePtr<Framebuffer>;

/// \class vgc::graphics::SwapChain
/// \brief Abstract window swap buffers chain.
///
class VGC_GRAPHICS_API SwapChain : public Resource {
protected:
    using Resource::Resource;

    SwapChain(ResourceList* owningList, const SwapChainDesc& desc)
        : Resource(owningList), desc_(desc)
    {}

public:
    const SwapChainDesc& desc() const {
        return desc_;
    }

    UInt32 pendingPresentCount() const {
        return pendingPresentCount_.load();
    }

    const FramebufferPtr& defaultFrameBuffer() const {
        return defaultFrameBuffer_;
    }

protected:
    friend Engine;

    void clearSubResources_() override {
        defaultFrameBuffer_.reset();
    }

    FramebufferPtr defaultFrameBuffer_;

private:
    SwapChainDesc desc_;
    std::atomic_uint32_t pendingPresentCount_ = 0; // to limit queuing in the Engine.
};
using SwapChainPtr = ResourcePtr<SwapChain>;




} // namespace vgc::graphics

#endif // VGC_GRAPHICS_TARGETS_H
