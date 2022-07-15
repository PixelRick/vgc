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

#ifndef VGC_GRAPHICS_IMAGE_H
#define VGC_GRAPHICS_IMAGE_H

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>

namespace vgc::graphics {

// Concept mapping:
//  D3D11  -> Image
//  OpenGL -> Image (within Texture)
//  Vulkan -> Image
//

/// \class vgc::graphics::ImageCreateInfo
/// \brief Parameters for image creation.
///
class VGC_GRAPHICS_API ImageCreateInfo {
public:
    constexpr ImageCreateInfo() noexcept = default;

    UInt32 width() const
    {
        return width_;
    }

    void setWidth(UInt32 width)
    {
        width_ = width;
    }

    UInt32 height() const
    {
        return height_;
    }

    void setHeight(UInt32 height)
    {
        height_ = height;
    }

    ImageRank rank() const
    {
        return rank_;
    }

    void setRank(ImageRank rank)
    {
        rank_ = rank;
    }

    ImageFormat format() const
    {
        return format_;
    }

    void setFormat(ImageFormat format)
    {
        format_ = format;
    }

    UInt8 layerCount() const
    {
        return layerCount_;
    }

    void setLayerCount(UInt8 layerCount)
    {
        layerCount_ = layerCount;
    }

    UInt8 mipLevelCount() const
    {
        return mipLevelCount_;
    }

    void setMipLevelCount(UInt8 mipLevelCount)
    {
        mipLevelCount_ = mipLevelCount;
    }

    UInt8 sampleCount() const
    {
        return sampleCount_;
    }

    void setSampleCount(UInt8 sampleCount)
    {
        sampleCount_ = sampleCount;
    }

    Usage usage() const
    {
        return usage_;
    }

    void setUsage(Usage usage)
    {
        usage_ = usage;
    }

    ImageBindFlags bindFlags() const
    {
        return bindFlags_;
    }

    void setBindFlags(ImageBindFlags bindFlags)
    {
        bindFlags_ = bindFlags;
    }

    ResourceMiscFlags resourceMiscFlags() const
    {
        return resourceMiscFlags_;
    }

    void setResourceMiscFlags(ResourceMiscFlags resourceMiscFlags)
    {
        resourceMiscFlags_ = resourceMiscFlags;
    }

    CpuAccessFlags cpuAccessFlags() const
    {
        return cpuAccessFlags_;
    }

    void setCpuAccessFlags(CpuAccessFlags cpuAccessFlags)
    {
        cpuAccessFlags_ = cpuAccessFlags;
    }

private:
    UInt32 width_ = 0;
    UInt32 height_ = 0;
    ImageRank rank_ = ImageRank::_1D;
    ImageFormat format_ = ImageFormat::Unknown;
    UInt8 layerCount_ = 1;
    UInt8 mipLevelCount_ = 1;
    UInt8 sampleCount_ = 1;
    Usage usage_ = Usage::Default;
    ImageBindFlags bindFlags_ = ImageBindFlags::ShaderResource;
    CpuAccessFlags cpuAccessFlags_ = CpuAccessFlags::None;
    ResourceMiscFlags resourceMiscFlags_ = ResourceMiscFlags::None;
};

/// \class vgc::graphics::Image
/// \brief Abstract image resource.
///
class VGC_GRAPHICS_API Image : public Resource {
protected:
    Image(ResourceList* gcList,
          const ImageCreateInfo& info)
        : Resource(gcList)
        , info_(info)
    {
    }

public:
    UInt32 width() const
    {
        return info_.width();
    }

    UInt32 height() const
    {
        return info_.height();
    }

    ImageRank rank() const
    {
        return info_.rank();
    }

    ImageFormat format() const
    {
        return info_.format();
    }

    UInt8 layerCount() const
    {
        return info_.layerCount();
    }

    UInt8 mipLevelCount() const
    {
        return info_.mipLevelCount();
    }

    UInt8 sampleCount() const
    {
        return info_.sampleCount();
    }

    Usage usage() const
    {
        return info_.usage();
    }

    ImageBindFlags bindFlags() const
    {
        return info_.bindFlags();
    }

    CpuAccessFlags cpuAccessFlags() const
    {
        return info_.cpuAccessFlags();
    }

    ResourceMiscFlags resourceMiscFlags() const
    {
        return info_.resourceMiscFlags();
    }

private:
    ImageCreateInfo info_;
};
using ImagePtr = ResourcePtr<Image>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_IMAGE_H
