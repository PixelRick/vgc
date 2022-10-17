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

#ifndef VGC_GRAPH_GRAPH_H
#define VGC_GRAPH_GRAPH_H

#include <functional>
#include <unordered_map>

#include <vgc/core/array.h>
#include <vgc/core/stringid.h>
#include <vgc/dom/element.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/geometry/vec2f.h>
#include <vgc/graph/api.h>

namespace vgc::graph {

class ElementPath {
public:
    explicit ElementPath(dom::Element* element)
        : rootDirectory_(element) {
    }

    size_t hash() const noexcept {
        size_t res = reinterpret_cast<uintptr_t>(static_cast<void*>(rootDirectory_));
        std::hash<core::StringId> hasher = {};
        for (const core::StringId& sid : relativePath_) {
            res ^= hasher(sid);
        }
    }

private:
    dom::Element* rootDirectory_ = nullptr;
    core::Array<core::StringId> relativePath_;
};

template<typename T>
using ElementMap = std::unordered_map<ElementPath, T>;


// canvas
//struct VGC_GRAPH_API Edge : Cell {
//
//private:
//    // Stroke
//    graphics::BufferPtr strokeStrip_;
//    graphics::BufferPtr controlPoints_;
//    graphics::BufferPtr outline_;
//
//    CurveMesh mesh_;
};

} // namespace vgc::graph

struct std::hash<vgc::graph::ElementPath> {
    std::size_t operator()(const vgc::graph::ElementPath& p) const noexcept {
        return p.hash();
    }
};

#endif // VGC_GRAPH_GRAPH_H
