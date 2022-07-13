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

#ifndef VGC_GRAPHICS_GEOMETRYVIEW_H
#define VGC_GRAPHICS_GEOMETRYVIEW_H

#include <array>

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>
#include <vgc/graphics/buffer.h>

namespace vgc::graphics {

class GeometryView;

inline constexpr size_t maxAttachedVertexBufferCount = 4;

using VertexBufferArray = std::array<BufferPtr, maxAttachedVertexBufferCount>;

/// \class vgc::graphics::GeometryViewCreateInfo
/// \brief Parameters for geometry view creation.
///
class VGC_GRAPHICS_API GeometryViewCreateInfo {
public:
    GeometryViewCreateInfo() noexcept = default;

    PrimitiveType primitiveType() const
    {
        return primitiveType_;
    }

    void setPrimitiveType(PrimitiveType primitiveType)
    {
        primitiveType_ = primitiveType;
    }

    BuiltinGeometryLayout builtinGeometryLayout() const
    {
        return builtinGeometryLayout_;
    }

    void setBuiltinGeometryLayout(BuiltinGeometryLayout builtinGeometryLayout)
    {
        builtinGeometryLayout_ = builtinGeometryLayout;
    }

    const BufferPtr& indexBuffer() const
    {
        return indexBuffer_;
    }

    void setIndexBuffer(const BufferPtr& indexBuffer)
    {
        indexBuffer_ = indexBuffer;
    }

    const VertexBufferArray& vertexBuffers() const
    {
        return vertexBuffers_;
    }

    void setVertexBuffer(Int i, const BufferPtr& vertexBuffer)
    {
        size_t idx = core::int_cast<size_t>(i);
        if (idx >= vertexBuffers_.size()) {
            throw core::IndexError(core::format(
                "Vertex buffer index {} is out of range [0, {}]", i, vertexBuffers_.size() - 1));
        }
        vertexBuffers_[idx] = vertexBuffer;
    }

private:
    friend GeometryView; // to reset resource pointers

    PrimitiveType primitiveType_ = PrimitiveType::Point;
    BuiltinGeometryLayout builtinGeometryLayout_ = BuiltinGeometryLayout::None;
    BufferPtr indexBuffer_ = {};
    VertexBufferArray vertexBuffers_ = {};
};

/// \class vgc::graphics::GeometryView
/// \brief View on a sequence of primitives.
///
/// View on a sequence of primitives of type point, line, or triangle.
/// Vertices can have different components layed out in arrays.
/// These arrays can be interleaved and stored in a one or multiple buffers.
/// The layout must be described by a builtin enum or in the future by a generic
/// descriptor structure.
///
class VGC_GRAPHICS_API GeometryView : public Resource {
protected:
    GeometryView(ResourceList* gcList, const GeometryViewCreateInfo& info)
        : Resource(gcList), info_(info)
    {
        // XXX check buffers against layout (slots, alignment, ..)

        for (const BufferPtr& vb : info_.vertexBuffers()) {
            if (vb && !(vb->bindFlags() & BindFlags::VertexBuffer)) {
                throw core::LogicError("Buffer needs BindFlags::VertexBuffer flag to be used as a vertex buffer");
            }
        }
        const BufferPtr& ib = info_.indexBuffer();
        if (ib && !(ib->bindFlags() & BindFlags::IndexBuffer)) {
            throw core::LogicError("Buffer needs BindFlags::IndexBuffer flag to be used as an index buffer");
        }
    }

public:
    PrimitiveType primitiveType() const
    {
        return info_.primitiveType();
    }

    BuiltinGeometryLayout builtinGeometryLayout() const
    {
        return info_.builtinGeometryLayout();
    }

    const BufferPtr& indexBuffer() const
    {
        return info_.indexBuffer();
    }

    const VertexBufferArray& vertexBuffers() const
    {
        return info_.vertexBuffers();
    }

private:
    friend Engine;

    void clearSubResources_() override
    {
        for (BufferPtr& vb : info_.vertexBuffers_) {
            vb.reset();
        }
        info_.indexBuffer_.reset();
    }

    GeometryViewCreateInfo info_;
};
using GeometryViewPtr = ResourcePtr<GeometryView>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_GEOMETRYVIEW_H
