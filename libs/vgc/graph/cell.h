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

#ifndef VGC_GRAPH_CELL_H
#define VGC_GRAPH_CELL_H

#include <vgc/core/object.h>
#include <vgc/dom/path.h>
#include <vgc/geometry/curve.h>
#include <vgc/geometry/mat2d.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/graph/api.h>

namespace vgc::graph {

// Vertex iterator
// Edge iterator
// if we use compact indices we can reuse them for storage indexing..
// u32 should be enough,
// have layer and group in graph too.
// let's use a map ! (same for canvas)

VGC_DECLARE_OBJECT(Graph);

namespace detail {

// Affine 2D transform for Vec2d.
class Transform2d {

    geometry::Vec2d operator*(const geometry::Vec2d& v) const {
        return matrix_ * v + translation_;
    }

private:
    geometry::Vec2d translation_;
    geometry::Mat2d matrix_;
};

class VGC_GRAPH_API GraphElement {
protected:
    virtual ~GraphElement() = default;

public:
    //const dom::Path& shortPath() const {
    //    return domPath_;
    //}

private:
    std::shared_ptr<Transform2d> transform_;
};

} // namespace detail

/// \enum vgc::graph::CellType
/// \brief Specifies the type of a Cell.
enum class CellType {
    KeyVertex,
    KeyEdge,
    KeyFace,
    InbetweenVertex,
    InbetweenEdge,
    InbetweenFace,
};

class VGC_GRAPH_API Cell {
protected:
    virtual ~Cell() = default;

public:
    const dom::Path& shortPath() const {
        return shortPath_;
    }

private:
    dom::Path shortPath_;
};

class VGC_GRAPH_API KeyCell : virtual public Cell {
    // api ?
};

class VGC_GRAPH_API InBetweenCell : virtual public Cell {
    // api ?
};

class VGC_GRAPH_API Vertex : virtual public Cell {
    virtual geometry::Vec2d position() const = 0;
};

class VGC_GRAPH_API Edge : virtual public Cell {};

class VGC_GRAPH_API Face : virtual public Cell {};

class VGC_GRAPH_API KeyVertex : public KeyCell, Vertex {

    geometry::Vec2d position() const override {
        return positionFast();
    }

    const geometry::Vec2d& positionFast() const {
        return position_;
    }

private:
    geometry::Vec2d position_;
};

class VGC_GRAPH_API KeyEdge : virtual public KeyCell, Edge {

private:
    geometry::Vec2dArray meshVertices_;
    core::Array<UInt32> meshIndices_;
    geometry::Vec2d meshOffset_;
    core::Array<geometry::CurveSample> samples_;
    bool isClosed_ = false;
};

class VGC_GRAPH_API KeyFace : virtual public KeyCell, Face {};

class VGC_GRAPH_API InBetweenVertex : public InBetweenCell, Vertex {
    geometry::Vec2d position() const override {
        return {}; // XXX VGC Animation.
    }
};

class VGC_GRAPH_API InBetweenEdge : public InBetweenCell, Edge {};

class VGC_GRAPH_API InBetweenFace : public InBetweenCell, Face {};

} // namespace vgc::graph

#endif // VGC_GRAPH_CELL_H
