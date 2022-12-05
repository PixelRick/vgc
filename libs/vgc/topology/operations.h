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

#ifndef VGC_TOPOLOGY_OPERATIONS_H
#define VGC_TOPOLOGY_OPERATIONS_H

#include <vgc/topology/detail/operations.h>
#include <vgc/topology/vac.h>

namespace vgc::topology::ops {

// VGC_TOPOLOGY_API

inline std::unique_ptr<VacGroup> createUnlinkedVacGroup(core::Id id) {
    return detail::Operations::createUnlinkedVacGroup(id);
}

inline std::unique_ptr<KeyVertex>
createUnlinkedKeyVertex(core::Id id, core::AnimTime t = {}) {
    return detail::Operations::createUnlinkedKeyVertex(id, t);
}

inline std::unique_ptr<KeyEdge>
createUnlinkedKeyEdge(core::Id id, core::AnimTime t = {}) {
    return detail::Operations::createUnlinkedKeyEdge(id, t);
}

// Throws if insertPos is not in range.
inline VacGroup* linkVacGroupUnchecked(
    std::unique_ptr<VacGroup>&& p,
    Vac* vac,
    VacGroup* parentGroup,
    Int insertPos) {

    return detail::Operations::linkVacGroupUnchecked(
        std::move(p), vac, parentGroup, insertPos);
}

// Throws if insertPos is not in range.
inline KeyVertex*
linkKeyVertex(std::unique_ptr<KeyVertex>&& p, VacGroup* parentGroup, Int insertPos) {

    return detail::Operations::linkKeyVertex(std::move(p), parentGroup, insertPos);
}

// Throws if insertPos is not in range.
inline KeyEdge* linkKeyEdgeUnchecked(
    std::unique_ptr<KeyEdge>&& p,
    VacGroup* parentGroup,
    Int insertPos,
    KeyVertex* startVertex,
    KeyVertex* endVertex) {

    return detail::Operations::linkKeyEdgeUnchecked(
        std::move(p), parentGroup, insertPos, startVertex, endVertex);
}

// Throws if insertPos is not in range.
inline KeyEdge* linkKeyEdge(
    std::unique_ptr<KeyEdge>&& p,
    VacGroup* parentGroup,
    Int insertPos,
    KeyVertex* startVertex,
    KeyVertex* endVertex) {

    return detail::Operations::linkKeyEdge(
        std::move(p), parentGroup, insertPos, startVertex, endVertex);
}

// Throws if insertPos is not in range.
inline KeyEdge* linkKeyClosedEdge(
    std::unique_ptr<KeyEdge>&& p,
    VacGroup* parentGroup,
    Int insertPos,
    core::AnimTime t = {}) {

    return detail::Operations::linkKeyClosedEdge(std::move(p), parentGroup, insertPos, t);
}

// Throws if insertPos is not in range.
inline KeyVertex* createKeyVertex(
    core::Id id,
    VacGroup* parentGroup,
    Int insertPos,
    core::AnimTime t = {}) {

    return detail::Operations::createKeyVertex(id, parentGroup, insertPos, t);
}

// Throws if insertPos is not in range.
inline KeyEdge* createKeyEdge(
    core::Id id,
    VacGroup* parentGroup,
    Int insertPos,
    KeyVertex* startVertex,
    KeyVertex* endVertex) {

    return detail::Operations::createKeyEdge(
        id, parentGroup, insertPos, startVertex, endVertex);
}

// Throws if insertPos is not in range.
inline KeyEdge* createKeyClosedEdge(
    core::Id id,
    VacGroup* parentGroup,
    Int insertPos,
    core::AnimTime t = {}) {

    return detail::Operations::createKeyClosedEdge(id, parentGroup, insertPos, t);
}

inline void setKeyVertexPosition(KeyVertex* v, const geometry::Vec2d& pos) {

    return detail::Operations::setKeyVertexPositionUnchecked(v, pos);
}

} // namespace vgc::topology::ops

#endif // VGC_TOPOLOGY_OPERATIONS_H
