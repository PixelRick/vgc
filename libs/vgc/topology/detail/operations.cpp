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

#include <vgc/topology/detail/operations.h>
#include <vgc/topology/exceptions.h>

namespace vgc::topology::detail {

// dev note: update boundary before star

std::unique_ptr<VacGroup> Operations::createUnlinkedVacGroup(core::Id id) {
    return std::unique_ptr<VacGroup>(new VacGroup(id));
}

std::unique_ptr<KeyVertex>
Operations::createUnlinkedKeyVertex(core::Id id, core::AnimTime t) {
    return std::unique_ptr<KeyVertex>(new KeyVertex(id, t));
}

std::unique_ptr<KeyEdge>
Operations::createUnlinkedKeyEdge(core::Id id, core::AnimTime t) {
    return std::unique_ptr<KeyEdge>(new KeyEdge(id, t));
}

VacGroup* Operations::linkVacGroupUnchecked(
    std::unique_ptr<VacGroup>&& u,
    Vac* vac,
    VacGroup* parentGroup,
    Int insertPos) {

    VacGroup* p = u.get();

    // add cell to vac
    vac->nodes_[p->id_] = std::move(u);
    // add cell to parent group
    parentGroup->children_.emplace(insertPos, p);
    // update diff
    vac->version_ = core::genId();
    if (vac->diffEnabled_) {
        vac->diff_.onNodeDiff(p, VacNodeDiffFlag::Created);
        vac->diff_.onNodeDiff(parentGroup, VacNodeDiffFlag::ChildrenChanged);
    }
    return p;
}

KeyVertex* Operations::linkKeyVertex(
    std::unique_ptr<KeyVertex>&& u,
    VacGroup* parentGroup,
    Int insertPos) {

    Vac* vac = parentGroup->vac();
    KeyVertex* p = u.get();

    // add cell to vac
    vac->nodes_[p->id_] = std::move(u);
    // add cell to parent group
    parentGroup->children_.emplace(insertPos, p);
    // update diff
    vac->version_ = core::genId();
    if (vac->diffEnabled_) {
        vac->diff_.onNodeDiff(p, VacNodeDiffFlag::Created);
        vac->diff_.onNodeDiff(parentGroup, VacNodeDiffFlag::ChildrenChanged);
    }
    return p;
}

KeyEdge* Operations::linkKeyEdgeUnchecked(
    std::unique_ptr<KeyEdge>&& u,
    VacGroup* parentGroup,
    Int insertPos,
    KeyVertex* startVertex,
    KeyVertex* endVertex) {

    Vac* vac = parentGroup->vac();
    KeyEdge* p = u.get();

    // init cell
    p->startVertex_ = startVertex;
    p->endVertex_ = endVertex;

    // add cell to parent group
    parentGroup->children_.emplace(insertPos, p);
    // add edge to new vertices star
    startVertex->star_.emplaceLast(p);
    if (endVertex != startVertex) {
        endVertex->star_.emplaceLast(p);
    }
    // update diff
    vac->version_ = core::genId();
    if (vac->diffEnabled_) {
        vac->diff_.onNodeDiff(p, VacNodeDiffFlag::Created);
        vac->diff_.onNodeDiff(parentGroup, VacNodeDiffFlag::ChildrenChanged);
        vac->diff_.onNodeDiff(startVertex, VacNodeDiffFlag::StarChanged);
        vac->diff_.onNodeDiff(endVertex, VacNodeDiffFlag::StarChanged);
    }
    return p;
}

KeyEdge* Operations::linkKeyEdge(
    std::unique_ptr<KeyEdge>&& u,
    VacGroup* parentGroup,
    Int insertPos,
    KeyVertex* startVertex,
    KeyVertex* endVertex) {

    Vac* vac = parentGroup->vac();
    KeyEdge* p = u.get();
    // checks
    if (vac != startVertex->vac()) {
        throw LogicError("Given parentGroup and startVertex are not from the same Vac.");
    }
    if (vac != endVertex->vac()) {
        throw LogicError("Given parentGroup and endVertex are not from the same Vac.");
    }
    if (p->time() != startVertex->time()) {
        throw LogicError("Given startVertex has different time than this edge.");
    }
    if (p->time() != endVertex->time()) {
        throw LogicError("Given endVertex has different time than this edge.");
    }

    return linkKeyEdgeUnchecked(
        std::move(u), parentGroup, insertPos, startVertex, endVertex);
}

KeyEdge* Operations::linkKeyClosedEdge(
    std::unique_ptr<KeyEdge>&& u,
    VacGroup* parentGroup,
    Int insertPos,
    core::AnimTime t) {

    Vac* vac = parentGroup->vac();
    KeyEdge* p = u.get();

    // init cell
    p->time_ = t;
    // add cell to vac
    vac->nodes_[p->id_] = std::move(u);
    // add cell to parent group
    parentGroup->children_.emplace(insertPos, p);
    // update diff
    vac->version_ = core::genId();
    if (vac->diffEnabled_) {
        vac->diff_.onNodeDiff(p, VacNodeDiffFlag::Created);
        vac->diff_.onNodeDiff(parentGroup, VacNodeDiffFlag::ChildrenChanged);
    }
    return p;
}

void Operations::setKeyVertexPositionUnchecked(KeyVertex* v, const geometry::Vec2d& pos) {
    v->geometryParameters_.position_ = pos;
    Vac* vac = v->vac();
    if (vac) {
        vac->version_ = core::genId();
        if (vac->diffEnabled_) {
            vac->diff_.onNodeDiff(v, VacNodeDiffFlag::ParametersChanged);
        }
    }
}

} // namespace vgc::topology::detail
