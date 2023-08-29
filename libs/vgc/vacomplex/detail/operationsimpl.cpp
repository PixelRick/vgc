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

#include <vgc/vacomplex/detail/operationsimpl.h>

#include <unordered_set>

#include <vgc/vacomplex/exceptions.h>
#include <vgc/vacomplex/keyedgedata.h>
#include <vgc/vacomplex/logcategories.h>

namespace vgc::vacomplex::detail {

Operations::Operations(Complex* complex)
    : complex_(complex) {

    // Ensure complex is non-null
    if (!complex) {
        throw LogicError("Cannot instantiate a VAC `Operations` with a null complex.");
    }

    if (++complex->numOperationsInProgress_ == 1) {
        // Increment version
        complex->version_ += 1;
    }
}

Operations::~Operations() {
    Complex* complex = this->complex();
    // TODO: try/catch
    if (--complex->numOperationsInProgress_ == 0) {
        // Do geometric updates
        for (const ModifiedNodeInfo& info : complex->opDiff_.modifiedNodes_) {
            if (info.flags().has(NodeModificationFlag::BoundaryMeshChanged)
                && !info.flags().has(NodeModificationFlag::GeometryChanged)) {
                //
                // Let the cell snap to its boundary..
                Cell* cell = info.node()->toCell();
                if (cell && cell->updateGeometryFromBoundary_()) {
                    onNodeModified_(cell, NodeModificationFlag::GeometryChanged);
                }
            }
        }
        complex->nodesChanged().emit(complex->opDiff_);
        complex->opDiff_.clear();
    }
}

Group* Operations::createRootGroup() {
    Group* group = createNode_<Group>(complex());
    return group;
}

Group* Operations::createGroup(Group* parentGroup, Node* nextSibling) {
    Group* group = createNodeAt_<Group>(parentGroup, nextSibling, complex());
    return group;
}

KeyVertex* Operations::createKeyVertex(
    const geometry::Vec2d& position,
    Group* parentGroup,
    Node* nextSibling,
    core::AnimTime t) {

    KeyVertex* kv = createNodeAt_<KeyVertex>(parentGroup, nextSibling, t);

    // Topological attributes
    // -> None

    // Geometric attributes
    kv->position_ = position;

    return kv;
}

// TODO: replace points & widths with `std::unique_ptr<EdgeGeometry>&& geometry`.

KeyEdge* Operations::createKeyOpenEdge(
    KeyVertex* startVertex,
    KeyVertex* endVertex,
    const std::shared_ptr<KeyEdgeData>& data,
    Group* parentGroup,
    Node* nextSibling) {

    KeyEdge* ke = createNodeAt_<KeyEdge>(parentGroup, nextSibling, startVertex->time());

    // Topological attributes
    ke->startVertex_ = startVertex;
    ke->endVertex_ = endVertex;
    addToBoundary_(ke, startVertex);
    addToBoundary_(ke, endVertex);

    // Geometric attributes
    ke->data_ = data;
    detail::CellPropertiesPrivateInterface::setOwningCell(&data->properties(), ke);

    return ke;
}

KeyEdge* Operations::createKeyClosedEdge(
    const std::shared_ptr<KeyEdgeData>& data,
    Group* parentGroup,
    Node* nextSibling,
    core::AnimTime t) {

    KeyEdge* ke = createNodeAt_<KeyEdge>(parentGroup, nextSibling, t);

    // Topological attributes
    // -> None

    // Geometric attributes
    ke->data_ = data;
    detail::CellPropertiesPrivateInterface::setOwningCell(&data->properties(), ke);

    return ke;
}

// Assumes `cycles` are valid.
// Assumes `nextSibling` is either `nullptr` or a child of `parentGroup`.
KeyFace* Operations::createKeyFace(
    core::Array<KeyCycle> cycles,
    Group* parentGroup,
    Node* nextSibling,
    core::AnimTime t) {

    KeyFace* kf = createNodeAt_<KeyFace>(parentGroup, nextSibling, t);

    // Topological attributes
    kf->cycles_ = std::move(cycles);
    for (const KeyCycle& cycle : kf->cycles()) {
        addToBoundary_(kf, cycle);
    }

    // Geometric attributes
    // -> None

    return kf;
}

void Operations::hardDelete(Node* node, bool deleteIsolatedVertices) {

    std::unordered_set<Node*> nodesToDestroy;

    // When hard-deleting the root, we delete all nodes below the root, but
    // preserve the root itself since we have the invariant that there is
    // always a root.
    //
    const bool isRoot = (complex()->rootGroup() == node);
    if (!isRoot) {
        nodesToDestroy.insert(node);
    }

    // Recursively collect all dependent nodes:
    // - children of groups
    // - star cells of cells
    //
    collectDependentNodes_(node, nodesToDestroy);

    // Flag all cells that are about to be deleted.
    //
    for (Node* nodeToDestroy : nodesToDestroy) {
        nodeToDestroy->isBeingDeleted_ = true;
    }

    // Helper function that tests if the star of a cell will become empty after
    // deleting all cells flagged for deletion.
    //
    auto hasEmptyStar = [](Cell* cell) {
        bool isStarEmpty = true;
        for (Cell* starCell : cell->star()) {
            if (!starCell->isBeingDeleted_) {
                isStarEmpty = false;
                break;
            }
        }
        return isStarEmpty;
    };

    // Update star of cells in the boundary of deleted cells.
    //
    // For example, if we delete an edge, we should remove the edge
    // for the star of its end vertices.
    //
    // In this step, we also detect vertices which are about to become
    // isolated, and delete these if deleteIsolatedVertices is true. Note that
    // there is no need to collectDependentNodes_(isolatedVertex), since being
    // isolated means having an empty star, which means that the vertex has no
    // dependent nodes.
    //
    // Note: we store the isolated vertices as set<Node*> instead of set<Cell*>
    // so that we can later merge with nodesToDelete.
    //
    std::unordered_set<Node*> isolatedKeyVertices;
    std::unordered_set<Node*> isolatedInbetweenVertices;
    for (Node* nodeToDestroy : nodesToDestroy) {
        if (nodeToDestroy->isCell()) {
            Cell* cell = nodeToDestroy->toCellUnchecked();
            for (Cell* boundaryCell : cell->boundary()) {
                if (boundaryCell->isBeingDeleted_) {
                    continue;
                }
                if (deleteIsolatedVertices
                    && boundaryCell->spatialType() == CellSpatialType::Vertex
                    && hasEmptyStar(boundaryCell)) {

                    switch (boundaryCell->cellType()) {
                    case CellType::KeyVertex:
                        isolatedKeyVertices.insert(boundaryCell);
                        break;
                    case CellType::InbetweenVertex:
                        isolatedInbetweenVertices.insert(boundaryCell);
                        break;
                    default:
                        break;
                    }
                    boundaryCell->isBeingDeleted_ = true;
                }
                if (!boundaryCell->isBeingDeleted_) {
                    boundaryCell->star_.removeOne(cell);
                    onNodeModified_(boundaryCell, NodeModificationFlag::StarChanged);
                }
            }
        }
    }

    // Deleting isolated inbetween vertices might indirectly cause key vertices
    // to become isolated, so we detect these in a second pass.
    //
    //       ke1
    // kv1 -------- kv2          Scenario: user hard deletes ie1
    //  |            |
    //  |iv1         | iv2        -> This directly makes iv1, iv2, and iv3 isolated
    //  |            |               (but does not directly make kv5 isolated, since
    //  |    ie1     kv5              the star of kv5 still contained iv2 and iv3)
    //  |            |
    //  |            | iv3
    //  |            |
    // kv3 ------- kv4
    //       ke2
    //
    if (deleteIsolatedVertices) {
        for (Node* inbetweenVertexNode : isolatedInbetweenVertices) {
            Cell* inbetweenVertex = inbetweenVertexNode->toCellUnchecked();
            for (Cell* keyVertex : inbetweenVertex->boundary()) {
                if (keyVertex->isBeingDeleted_) {
                    continue;
                }
                if (hasEmptyStar(keyVertex)) {
                    isolatedKeyVertices.insert(keyVertex);
                    keyVertex->isBeingDeleted_ = true;
                }
                else {
                    keyVertex->star_.removeOne(inbetweenVertex);
                    onNodeModified_(keyVertex, NodeModificationFlag::StarChanged);
                }
            }
        }
        nodesToDestroy.merge(isolatedKeyVertices);
        nodesToDestroy.merge(isolatedInbetweenVertices);
    }

    destroyNodes_(nodesToDestroy);
}

void Operations::softDelete(Node* /*node*/, bool /*deleteIsolatedVertices*/) {
    // TODO
    throw core::LogicError("Soft Delete topological operator is not implemented yet.");
}

KeyVertex*
Operations::glueKeyVertices(core::Span<KeyVertex*> kvs, const geometry::Vec2d& position) {
    if (kvs.isEmpty()) {
        return nullptr;
    }
    KeyVertex* kv0 = kvs[0];

    bool hasDifferentKvs = false;
    for (KeyVertex* kv : kvs.subspan(1)) {
        if (kv != kv0) {
            hasDifferentKvs = true;
            break;
        }
    }

    if (!hasDifferentKvs) {
        setKeyVertexPosition(kv0, position);
        return kv0;
    }

    // Location: top-most input vertex
    Int n = kvs.length();
    core::Array<Node*> nodes(n);
    for (Int i = 0; i < n; ++i) {
        nodes[i] = kvs[i];
    }
    Node* topMostVertex = findTopMost(nodes);
    Group* parentGroup = topMostVertex->parentGroup();
    Node* nextSibling = topMostVertex->nextSibling();

    // TODO: define source operation
    KeyVertex* newKv = createKeyVertex(position, parentGroup, nextSibling, kv0->time());

    std::unordered_set<KeyVertex*> seen;
    for (KeyVertex* kv : kvs) {
        bool inserted = seen.insert(kv).second;
        if (inserted) {
            substitute_(kv, newKv);
            hardDelete(kv, false);
        }
    }

    return newKv;
}

KeyEdge* Operations::glueKeyOpenEdges(
    core::Span<KeyHalfedge> khes,
    std::shared_ptr<KeyEdgeData> geometry,
    const geometry::Vec2d& startPosition,
    const geometry::Vec2d& endPosition) {

    if (khes.isEmpty()) {
        return nullptr;
    }

    Int n = khes.length();

    core::Array<KeyVertex*> startVertices;
    startVertices.reserve(n);
    for (const KeyHalfedge& khe : khes) {
        startVertices.append(khe.startVertex());
    }
    glueKeyVertices(startVertices, startPosition);

    // Note: we can only list end vertices after the glue of
    // start vertices since it can substitute end vertices.
    core::Array<KeyVertex*> endVertices;
    endVertices.reserve(n);
    for (const KeyHalfedge& khe : khes) {
        endVertices.append(khe.endVertex());
    }
    glueKeyVertices(endVertices, endPosition);

    // Location: top-most input edge
    core::Array<Node*> edgeNodes(n);
    for (Int i = 0; i < n; ++i) {
        edgeNodes[i] = khes[i].edge();
    }
    Node* topMostEdge = findTopMost(edgeNodes);
    Group* parentGroup = topMostEdge->parentGroup();
    Node* nextSibling = topMostEdge->nextSibling();

    KeyVertex* startKv = khes[0].startVertex();
    KeyVertex* endKv = khes[0].endVertex();

    // TODO: define source operation
    KeyEdge* newKe =
        createKeyOpenEdge(startKv, endKv, std::move(geometry), parentGroup, nextSibling);

    KeyHalfedge newKhe(newKe, true);
    for (const KeyHalfedge& khe : khes) {
        substitute_(khe, newKhe);
        // It is important that no 2 halfedges refer to the same edge.
        hardDelete(khe.edge(), true);
    }

    newKe->snapGeometry();
    return newKe;
}

KeyEdge* Operations::glueKeyClosedEdges( //
    core::Span<KeyHalfedge> khes,
    std::shared_ptr<KeyEdgeData> geometry) {

    if (khes.isEmpty()) {
        return nullptr;
    }

    Int n = khes.length();

    // Location: top-most input edge
    core::Array<Node*> edgeNodes(n);
    for (Int i = 0; i < n; ++i) {
        edgeNodes[i] = khes[i].edge();
    }
    Node* topMostEdge = findTopMost(edgeNodes);
    Group* parentGroup = topMostEdge->parentGroup();
    Node* nextSibling = topMostEdge->nextSibling();

    // TODO: define source operation
    KeyEdge* newKe = createKeyClosedEdge(std::move(geometry), parentGroup, nextSibling);

    KeyHalfedge newKhe(newKe, true);
    for (const KeyHalfedge& khe : khes) {
        substitute_(khe, newKhe);
        // It is important that no 2 halfedges refer to the same edge.
        hardDelete(khe.edge(), true);
    }

    return newKe;
}

core::Array<KeyEdge*> Operations::unglueKeyEdges(KeyEdge* targetKe) {
    core::Array<KeyEdge*> result;
    if (countUses_(targetKe) <= 1) {
        result.append(targetKe);
        return result;
    }

    // TODO: handle temporal star.

    // Helper
    auto duplicateTargetKe = [this, targetKe, &result]() {
        KeyEdge* newKe = nullptr;
        std::shared_ptr<KeyEdgeData> dataDuplicate = targetKe->data()->clone();
        if (targetKe->isClosed()) {
            // TODO: define source operation
            newKe = createKeyClosedEdge(
                std::move(dataDuplicate),
                targetKe->parentGroup(),
                targetKe->nextSibling());
        }
        else {
            // TODO: define source operation
            newKe = createKeyOpenEdge(
                targetKe->startVertex(),
                targetKe->endVertex(),
                std::move(dataDuplicate),
                targetKe->parentGroup(),
                targetKe->nextSibling());
        }
        result.append(newKe);
        return newKe;
    };

    // Helper. Assumes targetKe's star will be cleared later.
    auto removeTargetKeFromBoundary = [this, targetKe](Cell* boundedCell) {
        boundedCell->boundary_.removeOne(targetKe);
        onBoundaryChanged_(boundedCell);
    };

    // Substitute targetKe by a duplicate in each of its use.
    // Note: star is copied for safety since it may be modified in loop.
    for (Cell* cell : core::Array(targetKe->star_)) {
        switch (cell->cellType()) {
        case CellType::KeyFace: {
            KeyFace* kf = cell->toKeyFaceUnchecked();
            for (KeyCycle& cycle : kf->cycles_) {
                if (cycle.steinerVertex_) {
                    continue;
                }
                KeyHalfedge first = cycle.halfedges().first();
                if (!first.isClosed()) {
                    for (KeyHalfedge& kheRef : cycle.halfedges_) {
                        if (kheRef.edge() == targetKe) {
                            KeyEdge* newKe = duplicateTargetKe();
                            kheRef = KeyHalfedge(newKe, kheRef.direction());
                            addToBoundary_(kf, newKe);
                        }
                    }
                }
                else if (first.edge() == targetKe) {
                    KeyEdge* newKe = duplicateTargetKe();
                    for (KeyHalfedge& kheRef : cycle.halfedges_) {
                        kheRef = KeyHalfedge(newKe, kheRef.direction());
                    }
                    addToBoundary_(kf, newKe);
                    // TODO: instead of having a copy of the edge used N times
                    // use a single edge with its geometry looped N times.
                    // See Boris Dalstein's thesis page 187.
                }
            }
            removeTargetKeFromBoundary(kf);
            break;
        }
        default:
            throw core::LogicError(
                "unglueKeyEdges() doesn't support temporal cells in edge star.");
        }
    }

    // Delete targetKe
    targetKe->star_.clear();
    hardDelete(targetKe, true);

    return result;
}

core::Array<KeyVertex*> Operations::unglueKeyVertices(
    KeyVertex* targetKv,
    core::Array<std::pair<core::Id, core::Array<KeyEdge*>>>& ungluedKeyEdges) {

    core::Array<KeyVertex*> result;
    if (countUses_(targetKv) <= 1) {
        result.append(targetKv);
        return result;
    }

    // TODO: handle temporal star.

    // Unglue incident key edges.
    for (Cell* cell : core::Array(targetKv->star_)) {
        switch (cell->cellType()) {
        case CellType::KeyEdge: {
            KeyEdge* ke = cell->toKeyEdgeUnchecked();
            core::Id id = ke->id();
            core::Array<KeyEdge*> a = unglueKeyEdges(ke);
            if (a.length() > 1) {
                ungluedKeyEdges.emplaceLast(id, std::move(a));
            }
            break;
        }
        default:
            break;
        }
    }

    // Helper
    auto duplicateTargetKv = [this, targetKv, &result]() {
        // TODO: define source operation
        KeyVertex* newKv = createKeyVertex(
            targetKv->position(),
            targetKv->parentGroup(),
            targetKv->nextSibling(),
            targetKv->time());
        result.append(newKv);
        return newKv;
    };

    // Helper. Assumes targetKv's star will be cleared later.
    auto removeTargetKvFromBoundary = [this, targetKv](Cell* boundedCell) {
        boundedCell->boundary_.removeOne(targetKv);
        onBoundaryChanged_(boundedCell);
    };

    // Helper. Assumes the replaced key vertex is `targetKv` and that
    // targetKv's star will be cleared later.
    auto substituteTargetKvAtStartOrEndOfKhe = //
        [this, targetKv, &removeTargetKvFromBoundary](
            const KeyHalfedge& khe, bool startVertex, KeyVertex* newKv) {
            //
            KeyEdge* ke = khe.edge();

            KeyVertex* endKv = nullptr;
            if (khe.direction() == startVertex) {
                endKv = ke->endVertex();
                ke->startVertex_ = newKv;
            }
            else {
                endKv = ke->startVertex();
                ke->endVertex_ = newKv;
            }

            if (endKv != targetKv) {
                removeTargetKvFromBoundary(ke);
            }

            addToBoundary_(ke, newKv);
        };

    // Substitute targetKv by a duplicate in each of its use.
    // Note: star is copied for safety since it may be modified in loop.
    for (Cell* cell : core::Array(targetKv->star_)) {
        switch (cell->cellType()) {
        case CellType::KeyEdge: {
            KeyEdge* ke = cell->toKeyEdgeUnchecked();
            bool hasFaceInStar = false;
            for (Cell* keStarCell : ke->star()) {
                if (keStarCell->cellType() == CellType::KeyFace) {
                    hasFaceInStar = true;
                    break;
                }
            }
            if (!hasFaceInStar) {
                if (ke->isStartVertex(targetKv)) {
                    KeyVertex* newKv = duplicateTargetKv();
                    ke->startVertex_ = newKv;
                    addToBoundary_(ke, newKv);
                }
                if (ke->isEndVertex(targetKv)) {
                    KeyVertex* newKv = duplicateTargetKv();
                    ke->endVertex_ = newKv;
                    addToBoundary_(ke, newKv);
                }
                removeTargetKvFromBoundary(ke);
            }
            break;
        }
        case CellType::KeyFace: {
            KeyFace* kf = cell->toKeyFaceUnchecked();
            for (KeyCycle& cycle : kf->cycles_) {
                if (cycle.steinerVertex()) {
                    if (cycle.steinerVertex() == targetKv) {
                        KeyVertex* newKv = duplicateTargetKv();
                        cycle.steinerVertex_ = newKv;
                        addToBoundary_(kf, newKv);
                    }
                    continue;
                }
                Int numHalfedges = cycle.halfedges_.length();
                // substitute at face corner uses
                for (Int i = 0; i < numHalfedges; ++i) {
                    KeyHalfedge& khe1 = cycle.halfedges_[i];
                    if (khe1.startVertex() == targetKv) {
                        Int previousKheIndex = (i - 1 + numHalfedges) % numHalfedges;
                        KeyHalfedge& khe0 = cycle.halfedges_[previousKheIndex];

                        // (?)---khe0-->(targetKv)---khe1-->(?)
                        KeyVertex* newKv = duplicateTargetKv();
                        substituteTargetKvAtStartOrEndOfKhe(khe0, false, newKv);
                        substituteTargetKvAtStartOrEndOfKhe(khe1, true, newKv);
                        // (?)---khe0-->( newKv  )---khe1-->(?)

                        addToBoundary_(kf, newKv);
                    }
                }
            }
            removeTargetKvFromBoundary(kf);
            break;
        }
        default:
            throw core::LogicError(
                "unglueKeyVertices() doesn't support temporal cells in edge star.");
        }
    }

    // Delete targetKv
    targetKv->star_.clear();
    hardDelete(targetKv, false);

    return result;
}

bool Operations::uncutAtKeyVertex(KeyVertex* targetKv) {

    UncutAtKeyVertexInfo_ info = prepareUncutAtKeyVertex_(targetKv);
    if (!info.isValid) {
        return false;
    }

    if (info.kf) {
        info.kf->cycles_.removeAt(info.cycleIndex);
        info.kf->boundary_.removeOne(targetKv);
        onBoundaryChanged_(info.kf);
        // Delete targetKv
        targetKv->star_.clear();
        hardDelete(targetKv, false);
    }
    else if (info.khe1.edge() == info.khe2.edge()) {
        // Transform open edge into closed edge.
        // TODO: define source operation
        KeyEdge* oldKe = info.khe1.edge();
        KeyEdge* newKe = createKeyClosedEdge(
            oldKe->stealData(), oldKe->parentGroup(), oldKe->nextSibling());

        KeyHalfedge oldKhe(oldKe, true);
        KeyHalfedge newKhe(newKe, true);
        bool hasStar = false;
        for (Cell* starCell : oldKe->star()) {
            *starCell->boundary_.find(oldKe) = newKe;
            newKe->star_.append(starCell);
            starCell->substituteKeyHalfedge_(oldKhe, newKhe);
            onBoundaryChanged_(starCell);
            hasStar = true;
        }
        if (hasStar) {
            onNodeModified_(newKe, NodeModificationFlag::StarChanged);
        }

        // Delete oldKe and targetKv
        oldKe->star_.clear();
        hardDelete(oldKe, false);
        targetKv->star_.clear();
        hardDelete(targetKv, false);
    }
    else {
        KeyEdgeData* keg1 = info.khe1.edge()->geometry();
        KeyEdgeData* keg2 = info.khe2.edge()->geometry();
        KeyVertex* kv1 = nullptr;
        KeyVertex* kv2 = nullptr;
        bool dir1 = false;
        bool dir2 = false;
        if (info.khe1.endVertex() == targetKv) {
            dir1 = true;
            kv1 = info.khe1.startVertex();
        }
        else {
            kv1 = info.khe1.endVertex();
        }
        if (info.khe2.startVertex() == targetKv) {
            dir2 = true;
            kv2 = info.khe2.endVertex();
        }
        else {
            kv2 = info.khe2.startVertex();
        }
        std::shared_ptr<KeyEdgeData> mergedGeometry = keg1->merge(dir1, keg2, dir2);

        // TODO: really pick lower
        KeyEdge* lowerKe = info.khe1.edge();

        // TODO: define source operation
        KeyEdge* newKe = createKeyOpenEdge(
            kv1,
            kv2,
            std::move(mergedGeometry),
            lowerKe->parentGroup(),
            lowerKe->nextSibling());

        bool hasStar = false;
        for (Cell* starCell : lowerKe->star()) {
            KeyFace* kf = starCell->toKeyFace();
            if (!kf) {
                continue;
            }
            *kf->boundary_.find(info.khe1.edge()) = newKe;
            kf->boundary_.removeOne(info.khe2.edge());
            newKe->star_.append(kf);

            for (KeyCycle& cycle : kf->cycles_) {
                if (cycle.steinerVertex()) {
                    continue;
                }
                auto it = cycle.halfedges_.begin();
                while (it != cycle.halfedges_.end()) {
                    KeyHalfedge& khe = *it;
                    if (khe.endVertex() == targetKv) {
                        bool dir = khe.direction() == info.khe1.direction();
                        khe = KeyHalfedge(newKe, dir);
                        ++it;
                    }
                    if (khe.startVertex() == targetKv) {
                        it = cycle.halfedges_.erase(it);
                    }
                }
            }
            onBoundaryChanged_(kf);
            hasStar = true;
        }
        if (hasStar) {
            onNodeModified_(newKe, NodeModificationFlag::StarChanged);
        }
        // Delete khe1, khe2 and targetKv
        info.khe1.edge()->star_.clear();
        hardDelete(info.khe1.edge(), false);
        info.khe2.edge()->star_.clear();
        hardDelete(info.khe2.edge(), false);
        targetKv->star_.clear();
        hardDelete(targetKv, false);
    }

    return true;
}

bool Operations::uncutAtKeyEdge(KeyEdge* ke) {

    UncutAtKeyEdgeInfo_ info = prepareUncutAtKeyEdge_(ke);
    if (!info.isValid) {
        return false;
    }

    //

    return true;
}

void Operations::moveToGroup(Node* node, Group* parentGroup, Node* nextSibling) {
    if (nextSibling) {
        insertNodeBeforeSibling_(node, nextSibling);
    }
    else {
        insertNodeAsLastChild_(node, parentGroup);
    }
}

void Operations::moveBelowBoundary(Node* node) {
    Cell* cell = node->toCell();
    if (!cell) {
        return;
    }
    const auto& boundary = cell->boundary();
    if (boundary.length() == 0) {
        // nothing to do.
        return;
    }
    // currently keeping the same parent
    Node* oldParentNode = cell->parent();
    Node* newParentNode = oldParentNode;
    if (!newParentNode) {
        // `boundary.length() > 0` previously checked.
        newParentNode = (*boundary.begin())->parent();
    }
    if (!newParentNode) {
        return;
    }
    Group* newParent = newParentNode->toGroupUnchecked();
    Node* nextSibling = newParent->firstChild();
    while (nextSibling) {
        bool found = false;
        for (Cell* boundaryCell : boundary) {
            if (nextSibling == boundaryCell) {
                found = true;
                break;
            }
        }
        if (found) {
            break;
        }
        nextSibling = nextSibling->nextSibling();
    }
    if (nextSibling) {
        insertNodeBeforeSibling_(node, nextSibling);
    }
    else {
        // all boundary cells are in another group
        // TODO: use set of ancestors of boundary cells
        insertNodeAsLastChild_(node, newParent);
    }
}

// dev note: update boundary before star

void Operations::setKeyVertexPosition(KeyVertex* kv, const geometry::Vec2d& pos) {
    if (pos == kv->position_) {
        // same data
        return;
    }

    kv->position_ = pos;

    onGeometryChanged_(kv);
}

void Operations::setKeyEdgeData(
    KeyEdge* ke,
    const std::shared_ptr<KeyEdgeData>& geometry) {

    KeyEdge* previousKe = geometry->edge_;
    ke->geometry_ = geometry;
    geometry->edge_ = ke;

    if (previousKe) {
        previousKe->geometry_ = nullptr;
        onGeometryChanged_(previousKe);
    }

    onGeometryChanged_(ke);
}

void Operations::setKeyEdgeSamplingQuality(
    KeyEdge* ke,
    geometry::CurveSamplingQuality quality) {

    if (quality == ke->samplingQuality_) {
        // same data
        return;
    }

    ke->samplingQuality_ = quality;
    dirtyMesh_(ke);
}

void Operations::onNodeCreated_(Node* node) {
    complex_->opDiff_.onNodeCreated(node);
}

void Operations::onNodeInserted_(
    Node* node,
    Node* oldParent,
    NodeInsertionType insertionType) {
    complex_->opDiff_.onNodeInserted(node, oldParent, insertionType);
}

void Operations::onNodeModified_(Node* node, NodeModificationFlags diffFlags) {
    complex_->opDiff_.onNodeModified(node, diffFlags);
}

void Operations::insertNodeBeforeSibling_(Node* node, Node* nextSibling) {
    Group* oldParent = node->parentGroup();
    Group* newParent = nextSibling->parentGroup();
    if (newParent->insertChildUnchecked(nextSibling, node)) {
        onNodeInserted_(node, oldParent, NodeInsertionType::BeforeSibling);
    }
}

void Operations::insertNodeAfterSibling_(Node* node, Node* previousSibling) {
    Group* oldParent = node->parentGroup();
    Group* newParent = previousSibling->parentGroup();
    Node* nextSibling = previousSibling->nextSibling();
    if (newParent->insertChildUnchecked(nextSibling, node)) {
        onNodeInserted_(node, oldParent, NodeInsertionType::AfterSibling);
    }
}

void Operations::insertNodeAsFirstChild_(Node* node, Group* parent) {
    Group* oldParent = node->parentGroup();
    Node* nextSibling = parent->firstChild();
    if (parent->insertChildUnchecked(nextSibling, node)) {
        onNodeInserted_(node, oldParent, NodeInsertionType::FirstChild);
    }
}

void Operations::insertNodeAsLastChild_(Node* node, Group* parent) {
    Group* oldParent = node->parentGroup();
    if (parent->appendChild(node)) {
        onNodeInserted_(node, oldParent, NodeInsertionType::LastChild);
    }
}

Node* Operations::findTopMost(core::Span<Node*> nodes) {
    // currently only looking under a single parent
    // TODO: tree-wide top most.
    if (nodes.isEmpty()) {
        return nullptr;
    }
    Node* node0 = nodes[0];
    Group* parent = node0->parentGroup();
    Node* topMostNode = parent->lastChild();
    while (topMostNode) {
        if (nodes.contains(topMostNode)) {
            break;
        }
        topMostNode = topMostNode->previousSibling();
    }
    return topMostNode;
}

// Assumes node has no children.
// maybe we should also handle star/boundary changes here
void Operations::destroyNode_(Node* node) {
    [[maybe_unused]] Group* group = node->toGroup();
    VGC_ASSERT(!group || group->numChildren() == 0);
    Group* parentGroup = node->parentGroup();
    core::Id nodeId = node->id();
    node->unparent();
    complex()->nodes_.erase(nodeId);
    complex_->opDiff_.onNodeDestroyed(nodeId);
    if (parentGroup) {
        complex_->opDiff_.onNodeModified(
            parentGroup, NodeModificationFlag::ChildrenChanged);
    }
}

// Assumes that all descendants of all `nodes` are also in `nodes`.
void Operations::destroyNodes_(const std::unordered_set<Node*>& nodes) {
    // debug check
    for (Node* node : nodes) {
        Group* group = node->toGroup();
        if (group) {
            for (Node* child : *group) {
                VGC_ASSERT(nodes.count(child)); // == contains
            }
        }
    }
    for (Node* node : nodes) {
        Group* parentGroup = node->parentGroup();
        node->unparent();
        complex_->opDiff_.onNodeDestroyed(node->id());
        if (parentGroup) {
            complex_->opDiff_.onNodeModified(
                parentGroup, NodeModificationFlag::ChildrenChanged);
        }
    }
    for (Node* node : nodes) {
        complex()->nodes_.erase(node->id());
    }
}

void Operations::onBoundaryChanged_(Cell* cell) {
    onNodeModified_(cell, NodeModificationFlag::BoundaryChanged);
    onBoundaryMeshChanged_(cell);
}

void Operations::onGeometryChanged_(Cell* cell) {
    onNodeModified_(cell, NodeModificationFlag::GeometryChanged);
    dirtyMesh_(cell);
}

void Operations::onPropertyChanged_(Cell* cell, core::StringId name) {
    if (name == strings::style) {
        onNodeModified_(cell, NodeModificationFlag::StyleChanged);
    }
    else {
        // todo
    }
}

void Operations::onBoundaryMeshChanged_(Cell* cell) {
    onNodeModified_(cell, NodeModificationFlag::BoundaryMeshChanged);
    dirtyMesh_(cell);
}

void Operations::dirtyMesh_(Cell* cell) {
    if (cell->hasMeshBeenQueriedSinceLastDirtyEvent_) {
        cell->hasMeshBeenQueriedSinceLastDirtyEvent_ = false;
        cell->dirtyMesh_();
        onNodeModified_(cell, NodeModificationFlag::MeshChanged);
        for (Cell* starCell : cell->star()) {
            // No need for recursion since starCell.star() is a subset
            // of cell.star().
            onNodeModified_(starCell, NodeModificationFlag::BoundaryMeshChanged);
            if (starCell->hasMeshBeenQueriedSinceLastDirtyEvent_) {
                starCell->hasMeshBeenQueriedSinceLastDirtyEvent_ = false;
                starCell->dirtyMesh_();
                onNodeModified_(starCell, NodeModificationFlag::MeshChanged);
            }
        }
    }
}

void Operations::addToBoundary_(Cell* boundedCell, Cell* boundingCell) {
    if (!boundingCell) {
        throw core::LogicError("Cannot add null cell to boundary.");
    }
    else if (!boundedCell) {
        throw core::LogicError("Cannot modify the boundary of a null cell.");
    }
    else if (!boundedCell->boundary_.contains(boundingCell)) {
        boundedCell->boundary_.append(boundingCell);
        boundingCell->star_.append(boundedCell);
        onBoundaryChanged_(boundedCell);
        onNodeModified_(boundingCell, NodeModificationFlag::StarChanged);
    }
}

void Operations::addToBoundary_(FaceCell* face, const KeyCycle& cycle) {
    if (cycle.steinerVertex()) {
        // Steiner cycle
        addToBoundary_(face, cycle.steinerVertex());
    }
    else if (cycle.halfedges().first().isClosed()) {
        // Simple cycle
        addToBoundary_(face, cycle.halfedges().first().edge());
    }
    else {
        // Non-simple cycle
        for (const KeyHalfedge& halfedge : cycle.halfedges()) {
            addToBoundary_(face, halfedge.edge());
            addToBoundary_(face, halfedge.endVertex());
        }
    }
}

void Operations::substitute_(KeyVertex* oldVertex, KeyVertex* newVertex) {
    bool hasStar = false;
    for (Cell* starCell : oldVertex->star()) {
        *starCell->boundary_.find(oldVertex) = newVertex;
        newVertex->star_.append(starCell);
        starCell->substituteKeyVertex_(oldVertex, newVertex);
        onBoundaryChanged_(starCell);
        hasStar = true;
    }
    if (hasStar) {
        oldVertex->star_.clear();
        onNodeModified_(oldVertex, NodeModificationFlag::StarChanged);
        onNodeModified_(newVertex, NodeModificationFlag::StarChanged);
    }
}

void Operations::substitute_(const KeyHalfedge& oldKhe, const KeyHalfedge& newKhe) {

    KeyEdge* const oldKe = oldKhe.edge();
    KeyEdge* const newKe = newKhe.edge();
    bool hasStar = false;
    for (Cell* starCell : oldKe->star()) {
        *starCell->boundary_.find(oldKe) = newKe;
        newKe->star_.append(starCell);
        starCell->substituteKeyHalfedge_(oldKhe, newKhe);
        onBoundaryChanged_(starCell);
        hasStar = true;
    }
    if (hasStar) {
        oldKe->star_.clear();
        onNodeModified_(oldKe, NodeModificationFlag::StarChanged);
        onNodeModified_(newKe, NodeModificationFlag::StarChanged);
    }
}

void Operations::collectDependentNodes_(
    Node* node,
    std::unordered_set<Node*>& dependentNodes) {

    if (node->isGroup()) {
        // Delete all children of the group
        Group* group = node->toGroupUnchecked();
        for (Node* child : *group) {
            if (dependentNodes.insert(child).second) {
                collectDependentNodes_(child, dependentNodes);
            }
        }
    }
    else {
        // Delete all cells in the star of the cell
        Cell* cell = node->toCellUnchecked();
        for (Cell* starCell : cell->star()) {
            // No need for recursion since starCell.star() is a subset
            // of cell.star().
            dependentNodes.insert(starCell);
        }
    }
}

// Note: Uncut does not yet support incident inbetween cells. As a
// workaround, we do nothing, as if uncutting here isn't possible, even
// though maybe in theory it is. In the future, we should handle the cases
// where uncutting is actually possible despite the presence of incident
// inbetween cells.
Operations::UncutAtKeyVertexInfo_ Operations::prepareUncutAtKeyVertex_(KeyVertex* kv) {
    UncutAtKeyVertexInfo_ result = {};

    for (Cell* starCell : kv->star()) {
        switch (starCell->cellType()) {
        case CellType::KeyEdge: {
            KeyEdge* ke = starCell->toKeyEdgeUnchecked();
            if (ke->isStartVertex(kv)) {
                if (!result.khe1.edge()) {
                    result.khe1 = KeyHalfedge(ke, false);
                }
                else if (!result.khe2.edge()) {
                    result.khe2 = KeyHalfedge(ke, true);
                }
                else {
                    // Cannot uncut if kv is used more than twice as edge vertex.
                    return result;
                }
            }
            if (ke->isEndVertex(kv)) {
                if (!result.khe1.edge()) {
                    result.khe1 = KeyHalfedge(ke, true);
                }
                else if (!result.khe2.edge()) {
                    result.khe2 = KeyHalfedge(ke, false);
                }
                else {
                    // Cannot uncut if kv is used more than twice as edge vertex.
                    return result;
                }
            }
            break;
        }
        case CellType::KeyFace: {
            KeyFace* kf = starCell->toKeyFaceUnchecked();
            Int cycleIndex = -1;
            for (const KeyCycle& cycle : kf->cycles()) {
                ++cycleIndex;
                if (cycle.steinerVertex() == kv) {
                    if (result.kf) {
                        // Cannot uncut if kv is used more than once as steiner vertex.
                        return result;
                    }
                    result.kf = kf;
                    result.cycleIndex = cycleIndex;
                }
            }
            break;
        }
        case CellType::InbetweenVertex: {
            //InbetweenVertex* iv = starCell->toInbetweenVertexUnchecked();
            // Currently not supported.
            return result;
            break;
        }
        default:
            break;
        }
    }

    if (result.khe1.edge()) {
        if (!result.kf && result.khe2.edge()) {
            if (result.khe1.edge() != result.khe2.edge()) {
                // If edges are different:
                // (inverse op: cut open edge)
                //
                //                     ┌─←─┐
                //                     │   C
                // o ───A──→ X ───B──→ o ──┘
                //
                // Uncutting at X means replacing the chain AB by D.
                // Thus the cycle B*A*ABC would become D*DC but
                // the cycle B*BC would not be representable anymore.
                //
                // In other words, we want the edges to always be used
                // consecutively in the cycles they are part of.
                //
                for (Cell* starCell : kv->star()) {
                    KeyFace* kf = starCell->toKeyFace();
                    if (!kf) {
                        continue;
                    }
                    for (const KeyCycle& cycle : kf->cycles()) {
                        if (cycle.steinerVertex()) {
                            continue;
                        }
                        KeyEdge* previousKe = cycle.halfedges().last().edge();
                        for (const KeyHalfedge& khe : cycle.halfedges()) {
                            if (khe.startVertex() == kv) {
                                if (khe.edge() == previousKe) {
                                    // Cannot uncut if kv is used as a u-turn in cycle.
                                    return result;
                                }
                            }
                            previousKe = khe.edge();
                        }
                    }
                }
                result.isValid = true;
            }
            else {
                // (inverse op: cut closed edge)
                // the only incident edge is a loop, and we don't
                // kv to be used as a u-turn in any cycle.
                for (Cell* starCell : kv->star()) {
                    KeyFace* kf = starCell->toKeyFace();
                    if (!kf) {
                        continue;
                    }
                    for (const KeyCycle& cycle : kf->cycles()) {
                        if (cycle.steinerVertex()) {
                            continue;
                        }
                        if (cycle.halfedges().first().edge() != result.khe1.edge()) {
                            continue;
                        }
                        // All edges in this cycle are equal to result.khe1.edge().
                        // We require them to be in the same direction (no u-turn).
                        bool direction = cycle.halfedges().first().edge();
                        for (const KeyHalfedge& khe : cycle.halfedges()) {
                            if (khe.direction() != direction) {
                                // Cannot uncut if kv is used as a u-turn in cycle.
                                return result;
                            }
                        }
                    }
                }
                result.isValid = true;
            }
        }
    }
    else if (result.kf) {
        // (inverse op: cut face at vertex)
        result.isValid = true;
    }

    return result;
}

Operations::UncutAtKeyEdgeInfo_ Operations::prepareUncutAtKeyEdge_(KeyEdge* ke) {
    UncutAtKeyEdgeInfo_ result = {};

    for (Cell* starCell : ke->star()) {
        switch (starCell->cellType()) {
        case CellType::KeyFace: {
            KeyFace* kf = starCell->toKeyFaceUnchecked();
            Int cycleIndex = -1;
            for (const KeyCycle& cycle : kf->cycles()) {
                ++cycleIndex;
                if (cycle.steinerVertex()) {
                    continue;
                }
                Int componentIndex = -1;
                for (const KeyHalfedge& khe : cycle.halfedges()) {
                    ++componentIndex;
                    if (khe.edge() != ke) {
                        continue;
                    }
                    if (!result.kf1) {
                        result.kf1 = kf;
                        result.cycleIndex1 = cycleIndex;
                        result.componentIndex1 = componentIndex;
                    }
                    else if (!result.kf2) {
                        result.kf2 = kf;
                        result.cycleIndex2 = cycleIndex;
                        result.componentIndex2 = componentIndex;
                    }
                    else {
                        // Cannot uncut if used more than twice as face cycle component.
                        return result;
                    }
                }
            }
            break;
        }
        default:
            break;
        }
    }

    if (result.kf1 && result.kf2) {
        result.isValid = true;
    }

    return result;
}

Int Operations::countSteinerUses_(KeyVertex* kv) {
    Int count = 0;
    for (Cell* starCell : kv->star()) {
        KeyFace* kf = starCell->toKeyFace();
        if (!kf) {
            continue;
        }
        for (const KeyCycle& cycle : kf->cycles()) {
            if (cycle.steinerVertex() == kv) {
                ++count;
            }
        }
    }
    return count;
}

Int Operations::countUses_(KeyVertex* kv) {
    Int count = 0;
    for (Cell* starCell : kv->star()) {
        switch (starCell->cellType()) {
        case CellType::KeyEdge: {
            KeyEdge* ke = starCell->toKeyEdgeUnchecked();
            bool hasFaceInStar = false;
            for (Cell* keStarCell : ke->star()) {
                if (keStarCell->cellType() == CellType::KeyFace) {
                    hasFaceInStar = true;
                    break;
                }
            }
            if (!hasFaceInStar) {
                if (ke->isStartVertex(kv)) {
                    ++count;
                }
                if (ke->isEndVertex(kv)) {
                    ++count;
                }
            }
            break;
        }
        case CellType::KeyFace: {
            KeyFace* kf = starCell->toKeyFaceUnchecked();
            for (const KeyCycle& cycle : kf->cycles()) {
                if (cycle.steinerVertex()) {
                    if (cycle.steinerVertex() == kv) {
                        ++count;
                    }
                    continue;
                }
                for (const KeyHalfedge& khe : cycle.halfedges()) {
                    if (khe.startVertex() == kv) {
                        ++count;
                    }
                }
            }
            break;
        }
        default:
            break;
        }
    }
    return count;
}

Int Operations::countUses_(KeyEdge* ke) {
    Int count = 0;
    for (Cell* starCell : ke->star()) {
        KeyFace* kf = starCell->toKeyFace();
        if (!kf) {
            continue;
        }
        for (const KeyCycle& cycle : kf->cycles()) {
            if (cycle.steinerVertex()) {
                continue;
            }
            for (const KeyHalfedge& khe : cycle.halfedges()) {
                if (khe.edge() == ke) {
                    ++count;
                }
            }
        }
    }
    return count;
}

} // namespace vgc::vacomplex::detail
