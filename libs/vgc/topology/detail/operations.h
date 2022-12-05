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

#ifndef VGC_TOPOLOGY_DETAIL_OPERATIONS_H
#define VGC_TOPOLOGY_DETAIL_OPERATIONS_H

#include <vgc/topology/vac.h>

namespace vgc::topology::detail {

class VGC_TOPOLOGY_API Operations {
    using VacGroupChildrenIterator = decltype(VacGroup::children_)::iterator;
    using VacGroupChildrenConstIterator = decltype(VacGroup::children_)::const_iterator;

public:
    static std::unique_ptr<VacGroup> createUnlinkedVacGroup(core::Id id);

    static std::unique_ptr<KeyVertex>
    createUnlinkedKeyVertex(core::Id id, core::AnimTime t = {});
    static std::unique_ptr<KeyEdge>
    createUnlinkedKeyEdge(core::Id id, core::AnimTime t = {});

    // Throws if insertPos is not in range.
    static VacGroup* linkVacGroupUnchecked(
        std::unique_ptr<VacGroup>&& p,
        Vac* vac,
        VacGroup* parentGroup,
        Int insertPos);

    // Throws if insertPos is not in range.
    static KeyVertex*
    linkKeyVertex(std::unique_ptr<KeyVertex>&& p, VacGroup* parentGroup, Int insertPos);

    // Throws if insertPos is not in range.
    static KeyEdge* linkKeyEdgeUnchecked(
        std::unique_ptr<KeyEdge>&& p,
        VacGroup* parentGroup,
        Int insertPos,
        KeyVertex* startVertex,
        KeyVertex* endVertex);

    // Throws if insertPos is not in range.
    static KeyEdge* linkKeyEdge(
        std::unique_ptr<KeyEdge>&& p,
        VacGroup* parentGroup,
        Int insertPos,
        KeyVertex* startVertex,
        KeyVertex* endVertex);

    // Throws if insertPos is not in range.
    static KeyEdge* linkKeyClosedEdge(
        std::unique_ptr<KeyEdge>&& p,
        VacGroup* parentGroup,
        Int insertPos,
        core::AnimTime t = {});

    // Throws if insertPos is not in range.
    static KeyVertex* createKeyVertex(
        core::Id id,
        VacGroup* parentGroup,
        Int insertPos,
        core::AnimTime t = {}) {

        std::unique_ptr<KeyVertex> p = createUnlinkedKeyVertex(id, t);
        return linkKeyVertex(std::move(p), parentGroup, insertPos);
    }

    // Throws if insertPos is not in range.
    static KeyEdge* createKeyEdge(
        core::Id id,
        VacGroup* parentGroup,
        Int insertPos,
        KeyVertex* startVertex,
        KeyVertex* endVertex) {

        std::unique_ptr<KeyEdge> p = createUnlinkedKeyEdge(id);
        return linkKeyEdge(std::move(p), parentGroup, insertPos, startVertex, endVertex);
    }

    // Throws if insertPos is not in range.
    static KeyEdge* createKeyClosedEdge(
        core::Id id,
        VacGroup* parentGroup,
        Int insertPos,
        core::AnimTime t = {}) {

        std::unique_ptr<KeyEdge> p = createUnlinkedKeyEdge(id);
        return linkKeyClosedEdge(std::move(p), parentGroup, insertPos, t);
    }

    static void setKeyVertexPositionUnchecked(KeyVertex* v, const geometry::Vec2d& pos);
};

} // namespace vgc::topology::detail

#endif // VGC_TOPOLOGY_DETAIL_OPERATIONS_H
