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
    static constexpr size_t maxVertexBuffersCount = 4;

    using VertexBuffersArray = std::array<BufferPtr, maxVertexBuffersCount>;

    GeometryView(ResourceList* owningList,
                 PrimitiveType primitiveType,
                 const BuiltinGeometryLayout& layout,
                 const BufferPtr& indexBuffer,
                 const BufferPtr& vertexBuffer0)
        : Resource(owningList)
        , primitiveType_(primitiveType)
        , vertexBuffers_{vertexBuffer0}
        , indexBuffer_(indexBuffer)
        , layout_(layout)
    {
        // XXX check buffers against layout

        for (const BufferPtr& vb : vertexBuffers_) {
            if (vb) {
                if (!(vb->bindFlags() & BindFlags::VertexBuffer)) {
                    throw core::LogicError("Buffer needs BindFlags::VertexBuffer flag to be used as a vertex buffer");
                }
            }
        }
        if (indexBuffer_) {
            if (!(indexBuffer_->bindFlags() & BindFlags::VertexBuffer)) {
                throw core::LogicError("Buffer needs BindFlags::IndexBuffer flag to be used as an index buffer");
            }
        }
    }

public:
    PrimitiveType primitiveType() const {
        return primitiveType_;
    }

    const BuiltinGeometryLayout& layout() const {
        return layout_;
    }

    const BufferPtr& indexBuffer() const {
        return indexBuffer_;
    }

    VertexBuffersArray vertexBuffers() const {
        return vertexBuffers_;
    }

private:
    friend Engine;

    void clearSubResources_() override {
        for (BufferPtr& vb : vertexBuffers_) {
            vb.reset();
        }
        indexBuffer_.reset();
    }

    PrimitiveType primitiveType_;
    const BuiltinGeometryLayout& layout_;
    BufferPtr indexBuffer_;
    VertexBuffersArray vertexBuffers_;
};
using GeometryViewPtr = ResourcePtr<GeometryView>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_GEOMETRYVIEW_H
