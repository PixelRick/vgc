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

#ifndef VGC_GRAPHICS_BUFFERS_H
#define VGC_GRAPHICS_BUFFERS_H

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>

namespace vgc::graphics {

//class BufferDataSpan {
//    BufferDataSpan(const float* data, Int length)
//        : data_(data), length_(length) {}
//
//    const float* data() const {
//        return data_;
//    }
//
//    Int length() const {
//        return length_;
//    }
//
//private:
//    const float* data_;
//    Int length_;
//};

/// \class vgc::graphics::PrimitivesBuffer
/// \brief Abstract primitive data buffer.
///
class VGC_GRAPHICS_API Buffer : public Resource {
protected:
    Buffer(
        ResourceList* owningList,
        Usage usage,
        BindFlags bindFlags,
        CpuAccessFlags cpuAccessFlags)
        : Resource(owningList)
        , lengthInBytes_(0)
        , usage_(usage)
        , bindFlags_(bindFlags)
        , cpuAccessFlags_(cpuAccessFlags)
    {}

public:
    Int lengthInBytes() const {
        return lengthInBytes_;
    }

    Usage usage() const {
        return usage_;
    }

    BindFlags bindFlags() const {
        return bindFlags_;
    }

    CpuAccessFlags cpuAccessFlags() const {
        return cpuAccessFlags_;
    }

protected:
    Int lengthInBytes_;

private:
    Usage usage_;
    BindFlags bindFlags_;
    CpuAccessFlags cpuAccessFlags_;
};
using BufferPtr = ResourcePtr<Buffer>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_BUFFERS_H
