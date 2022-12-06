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

#ifndef VGC_WORKSPACE_EDGE_H
#define VGC_WORKSPACE_EDGE_H

#include <vgc/geometry/vec2d.h>
#include <vgc/geometry/vec4d.h>
#include <vgc/graphics/buffer.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/graphicelement.h>

namespace vgc::workspace {

class VGC_WORKSPACE_API EdgeMesh {
public:
    EdgeMesh() noexcept = default;

private:
    geometry::Vec2dArray meshVertices_;
    geometry::Vec4dArray meshUVSTs_;
    core::Array<UInt32> meshIndices_;
};

class VGC_WORKSPACE_API EdgeGraphics {
public:
private:
    // Stroke
    graphics::BufferPtr meshVertices_;
    graphics::BufferPtr meshUVSTs_;
    graphics::BufferPtr meshIndices_;
    graphics::BufferPtr controlPoints_;
    graphics::BufferPtr centerline_;

    //CurveMesh mesh_;

    // if edge is not animated we want to share the graphics resources for all frames.
    // if edge is animated we probably want to cache the graphics.
};


struct CurveGraphics {
    // Stroke
    graphics::GeometryViewPtr strokeGeometry_;

    // Fill
    //graphics::GeometryViewPtr fillGeometry_;

    // Control Points
    graphics::GeometryViewPtr pointsGeometry_;
    Int numPoints = 0;

    // Line
    graphics::GeometryViewPtr dispLineGeometry_;
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_EDGE_H
