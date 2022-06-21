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

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/buffers.h>
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

    void setWindowNativeHandle(void* windowNativeHandle) {
        windowNativeHandle_ = windowNativeHandle;
    }

    WindowNativeHandleType windowNativeHandleType() const {
        return windowNativeHandleType_;
    }

    void setWindowNativeHandle(WindowNativeHandleType windowNativeHandleType) {
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

/// \class vgc::graphics::RenderTargetView
/// \brief Abstract view of a render target buffer.
///
class VGC_GRAPHICS_API RenderTargetView : public Resource {
protected:
    RenderTargetView(ResourceList* owningList)
        : Resource(owningList) {}

    using Resource::Resource;
};
using RenderTargetViewPtr = ResourcePtr<RenderTargetView>;

/// \class vgc::graphics::SwapChain
/// \brief Abstract swap chain.
///
class VGC_GRAPHICS_API SwapChain : public Resource {
protected:
    using Resource::Resource;

    SwapChain(ResourceList* owningList, const SwapChainDesc& desc)
        : Resource(owningList), desc_(desc) {}

public:
    const SwapChainDesc& desc() const {
        return desc_;
    }

private:
    SwapChainDesc desc_;
};
using SwapChainPtr = ResourcePtr<SwapChain>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_TARGETS_H
