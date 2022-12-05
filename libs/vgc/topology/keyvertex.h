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

#ifndef VGC_TOPOLOGY_KEYVERTEX_H
#define VGC_TOPOLOGY_KEYVERTEX_H

#include <vgc/core/flags.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/topology/api.h>
#include <vgc/topology/cell.h>

namespace vgc::topology {

enum class KeyVertexGeometryParametersFlag : UInt16 {
    None = 0x00,
    ReadOnly = 0x01,
};
VGC_DEFINE_FLAGS(KeyVertexGeometryParametersFlags, KeyVertexGeometryParametersFlag)

// dev note: position could be a variant<vec2d, func, provider>
//           provider could have a dirty flag to not update data, especially important for
//           big value types like curve geometry in edges.

// Translated from dom or set manually via graph.
class VGC_TOPOLOGY_API KeyVertexGeometryParameters {
private:
    friend detail::Operations;

public:
    geometry::Vec2d position() const {
        return position_;
    }

private:
    geometry::Vec2d position_;
    KeyVertexGeometryParametersFlags flags_ = KeyVertexGeometryParametersFlag::None;
};

class VGC_TOPOLOGY_API KeyVertex : public KeyCell, public VertexCell {
private:
    friend detail::Operations;

    explicit KeyVertex(core::Id id, core::AnimTime t) noexcept
        : VacCell(id, VacCellType::KeyVertex)
        , KeyCell(t) {
    }

public:
    bool existsAt(core::AnimTime t) const override {
        return KeyCell::existsAt(t);
    }

    geometry::Vec2d position(core::AnimTime /*t*/) const override {
        return geometryParameters_.position();
    }

    double junctionWidth() const {
        return cachedJunctionWidth_;
    }

private:
    KeyVertexGeometryParameters geometryParameters_;
    mutable double cachedJunctionWidth_ = 0.0;
};

} // namespace vgc::topology

#endif // VGC_TOPOLOGY_KEYVERTEX_H
