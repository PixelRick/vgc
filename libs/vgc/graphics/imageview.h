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

#ifndef VGC_GRAPHICS_IMAGEVIEW_H
#define VGC_GRAPHICS_IMAGEVIEW_H

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>
#include <vgc/graphics/image.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

/// \class vgc::graphics::ImageViewCreateInfo
/// \brief Parameters for image view creation.
///
class VGC_GRAPHICS_API ImageViewCreateInfo {
public:
    constexpr ImageViewCreateInfo() noexcept = default;

    UInt8 layerStart() const
    {
        return layerStart_;
    }

    void setLayerStart(UInt8 layerStart)
    {
        layerStart_ = layerStart;
    }

    UInt8 layerCount() const
    {
        return layerCount_;
    }

    void setLayerCount(UInt8 layerCount)
    {
        layerCount_ = layerCount;
    }

    UInt8 layerLast() const
    {
        return layerStart_ + layerCount_ - 1;
    }

    UInt8 mipLevelStart() const
    {
        return mipLevelStart_;
    }

    void setMipLevelStart(UInt8 mipLevelStart)
    {
        mipLevelStart_ = mipLevelStart;
    }

    UInt8 mipLevelCount() const
    {
        return mipLevelCount_;
    }

    /// Only effective when binding as a shader resource.
    ///
    void setMipLevelCount(UInt8 mipLevelCount)
    {
        mipLevelCount_ = mipLevelCount;
    }

    UInt8 mipLevelLast() const
    {
        return mipLevelStart_ + mipLevelCount_ - 1;
    }

    ImageBindFlags bindFlags() const
    {
        return bindFlags_;
    }

    void setBindFlags(ImageBindFlags bindFlags)
    {
        bindFlags_ = bindFlags;
    }

private:
    UInt8 layerStart_ = 0;
    UInt8 layerCount_ = 0;
    UInt8 mipLevelStart_ = 0;
    UInt8 mipLevelCount_ = 0;

    ImageBindFlags bindFlags_ = ImageBindFlags::None;
};

/// \class vgc::graphics::ImageView
/// \brief Abstract view of an image buffer attachable to some stage of the graphics pipeline.
///
// Since a swap chain's render target view represents different buffers over
// time, a Vulkan implementation should probably cache a view for each
// back buffer.
//
// Concept mapping:
//  D3D11  -> Shader Resource View (SRV), Render Target View (RTV), Depth Stencil View (DSV)
//  OpenGL -> Texture
//  Vulkan -> Image View
// Looks like all three support buffers as image.
//
class VGC_GRAPHICS_API ImageView : public Resource {
protected:
    ImageView(ResourceList* gcList,
              const ImageViewCreateInfo& createInfo,
              const ResourcePtr<Resource>& viewedResource,
              ImageFormat format,
              UInt32 bufferElementsCount)
        : Resource(gcList)
        , info_(createInfo)
        , viewedResource_(viewedResource)
        , format_(format)
        , bufferElementsCount_(bufferElementsCount)
    {
    }

public:
    UInt8 layerStart() const
    {
        return info_.layerStart();
    }

    UInt8 layerCount() const
    {
        return info_.layerCount();
    }

    UInt8 layerLast() const
    {
        return info_.layerLast();
    }

    UInt8 mipLevelStart() const
    {
        return info_.mipLevelStart();
    }

    UInt8 mipLevelCount() const
    {
        return info_.mipLevelCount();
    }

    UInt8 mipLevelLast() const
    {
        return info_.mipLevelLast();
    }

    ImageBindFlags bindFlags() const
    {
        return info_.bindFlags();
    }

    const ResourcePtr<Resource>& viewedResource() const
    {
        return viewedResource_;
    }

    ImageFormat format() const
    {
        return format_;
    }

    UInt32 bufferElementsCount() const
    {
        return bufferElementsCount_;
    }

    bool isBuffer() const
    {
        return bufferElementsCount_ > 0;
    }

private:
    friend Engine;

    void clearSubResources_() override
    {
        viewedResource_.reset();
    }

    ImageViewCreateInfo info_;
    ResourcePtr<Resource> viewedResource_;
    ImageFormat format_;
    UInt32 bufferElementsCount_ = 0;
};
using ImageViewPtr = ResourcePtr<ImageView>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_IMAGEVIEW_H
