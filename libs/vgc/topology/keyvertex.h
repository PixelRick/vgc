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

// dev note: position could be a variant<vec2d, func, provider>
//           provider could have a dirty flag to not update data, especially important for
//           big value types like curve geometry in edges.

class VGC_TOPOLOGY_API KeyVertex : public KeyCell, public VertexCell {
private:
    friend detail::Operations;

    explicit KeyVertex(core::Id id, core::AnimTime t) noexcept
        : KeyCell(static_cast<VacCell*>(this), t)
        , VertexCell(id, VacCellType::KeyVertex) {
    }

public:
    using KeyCell::existsAt;
    using VacCell::cellType;
    using VacCell::existsAt;
    using VacCell::spatialType;
    using VacCell::vac;

    geometry::Vec2d position(core::AnimTime /*t*/) const override {
        return position_;
    }

private:
    geometry::Vec2d position_;
};

template<>
inline constexpr KeyCell* static_cell_cast<KeyCell, KeyVertex>(KeyVertex* p) {
    return static_cast<KeyCell*>(p);
}

template<>
inline constexpr KeyCell* dynamic_cell_cast<KeyCell, KeyVertex>(KeyVertex* p) {
    return static_cast<KeyCell*>(p);
}

template<>
inline constexpr VacCell* dynamic_cell_cast<VacCell, VacCellProxy>(VacCellProxy* p) {
    return p->cell();
}

} // namespace vgc::topology

#endif // VGC_TOPOLOGY_KEYVERTEX_H
