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

#include <vgc/dom/strings.h>
#include <vgc/topology/operations.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/logcategories.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

Workspace::Workspace(dom::DocumentPtr document)
    : document_(document) {

    document->changed().connect(onDocumentDiff());

    vac_ = topology::Vac::create();
    vac_->changed().connect(onVacDiff());

    rebuildTreeFromDom_();
    rebuildVacFromTree_();
}

/* static */
WorkspacePtr Workspace::create(dom::DocumentPtr document) {
    return WorkspacePtr(new Workspace(document));
}

//Renderable* Workspace::createRenderable(
//    graphics::Engine* engine,
//    core::AnimTime t,
//    core::Id id) {
//
//    // todo
//}

void Workspace::sync() {

    bool domChanged = document_ && (document_->version() != lastSyncedDomVersion_);
    bool vacChanged = vac_->version() != lastSyncedVacVersion_;

    if (domChanged && vacChanged) {
        VGC_ERROR(
            LogVgcWorkspace,
            "Both DOM and VAC of workspace have been edited since last synchronization. "
            "Rebuilding VAC from DOM.");
        updateTreeFromDom_();
        rebuildVacFromTree_();
        return;
    }
    else if (domChanged) {
        updateTreeAndVacFromDom_();
    }
    else if (vacChanged) {
        updateTreeAndDomFromVac_();
    }
}

void Workspace::onDocumentDiff_(const dom::Diff& /*diff*/) {
    if (isDomBeingUpdated_) {
        // workspace is doing the changes, no need to see the diff.
        return;
    }

    // XXX todo:
    // 1) test for topology change, if none, only update parameters
    // 2) do a true update, identify the impacted cells to rebuild
    //    only the necessary part of the vac
    updateTreeFromDom_();
    rebuildVacFromTree_();
    return;

    //namespace ss = dom::strings;

    // first create everything new
    // processing order:
    //
    //  std::unordered_map<Element*, std::set<core::StringId>> modifiedElements_;
    //  -> invalidate and delete desynced vac parts.
    //  -> add the destroyed cells to a pending rebuild and link list.
    //
    //  core::Array<Node*> createdNodes_;
    //  -> but don't link them yet. add
    //  -> add the destroyed cells to a pending link list.
    //
    //  std::unordered_map<Element*, std::set<core::StringId>> modifiedElements_;
    //  std::set<Node*> reparentedNodes_;
    //  core::Array<Node*> removedNodes_;
    //  std::set<Node*> childrenReorderedNodes_;
    //

    //core::Array<std::unique_ptr<topology::VacCell>> createdCells;

    //for (dom::Node* n : diff.createdNodes()) {
    //    dom::Element* e = dom::Element::cast(n);
    //    if (!e) {
    //        continue;
    //    }
    //    if (e->tagName() == ss::vertex) {
    //        std::unique_ptr<topology::KeyVertex> v =
    //            topology::ops::createUnlinkedKeyVertex(e->internalId(), core::AnimTime());
    //        /*
    //            {"color", core::colors::black},
    //            {"position", geometry::Vec2d()},
    //        */
    //        topology::ops::setKeyVertexPosition(
    //            v.get(), e->getAttribute(ss::position).getVec2d());

    //        // XXX create graphics obj
    //    }
    //    else if (e->tagName() == ss::edge) {
    //        std::unique_ptr<topology::KeyEdge> v =
    //            topology::ops::createUnlinkedKeyEdge(e->internalId(), core::AnimTime());
    //        /*
    //            {"color", core::colors::black},
    //            {"positions", geometry::Vec2dArray()},
    //            {"widths", core::DoubleArray()},
    //            {"startVertex", dom::Path()},
    //            {"endVertex", dom::Path()},
    //        */

    //        // will we deal with scripts here or will dom do it itself ?
    //    }
    //    else if (e->tagName() == ss::layer) {
    //        std::unique_ptr<topology::VacGroup> g =
    //            topology::ops::createUnlinkedVacGroup(e->internalId());
    //    }
    //}

    //for (const auto& me : diff.modifiedElements()) {
    //    dom::Element* e = me.first;
    //    const std::set<core::StringId>& attrs = me.second;
    //    if (!e) {
    //        continue;
    //    }
    //    if (e->tagName() == ss::vertex) {
    //        //
    //    }
    //    else if (e->tagName() == ss::edge) {
    //        // What if startVertex or endVertex changes ?
    //        // It is not really a valid vac operation.
    //        //
    //    }
    //    else if (e->tagName() == ss::layer) {
    //        //
    //    }
    //}

    //std::set<topology::VacCell*> aliveStar;
}

void Workspace::onVacDiff_(const topology::VacDiff& /*diff*/) {
    if (isVacBeingUpdated_) {
        // workspace is doing the changes, no need to see the diff.
        return;
    }

    //
}

void Workspace::updateTreeAndVacFromDom_() {
    if (!document_ || !vac_) {
        return;
    }

    isVacBeingUpdated_ = true;
    // process dom diff
    document_->emitPendingDiff();
    // flush vac changes
    vac_->emitPendingDiff();
    isVacBeingUpdated_ = false;
}

void Workspace::updateTreeAndDomFromVac_() {
    if (!document_ || !vac_) {
        return;
    }

    isDomBeingUpdated_ = true;
    // process vac diff
    vac_->emitPendingDiff();
    // flush dom changes
    document_->emitPendingDiff();
    isDomBeingUpdated_ = false;
}

void Workspace::updateTreeFromDom_() {
    //
}

dom::Element* elementDFSNext(dom::Element* e) {
    dom::Element* res = nullptr;
    if (e) {
        res = e->firstChildElement();
        if (!res) {
            res = e->nextSiblingElement();
            if (!res) {
                res = e->parentElement();
            }
        }
    }
    return res;
}

void Workspace::rebuildTreeFromDom_() {
    // reset tree
    elements_.clear();
    vgcElement_ = nullptr;

    if (!document_) {
        return;
    }

    //vgcElement_;
    //document_->rootElement();
    //dom::Element* root
}

namespace detail {

struct VacElementLists {
    // groups are in DFS order
    core::Array<Element*> groups;
    core::Array<Element*> keyVertices;
    core::Array<Element*> keyEdges;
    core::Array<Element*> keyFaces;
    core::Array<Element*> inbetweenVertices;
    core::Array<Element*> inbetweenEdges;
    core::Array<Element*> inbetweenFaces;
};

} // namespace detail

dom::Element* getElementRefAttribute(dom::Element* domElem, core::StringId name) {
    const dom::Value& domValue = domElem->getAuthoredAttribute(name);
    if (!domValue.has<dom::Path>()) {
        VGC_ERROR(
            LogVgcWorkspace,
            core::format(
                "Element `{}` requires a path attribute `{}`.",
                domElem->tagName(),
                name));
        return nullptr;
    }
    const dom::Path& path = domValue.getPath();
    dom::Element* vertexDomElement = domElem->elementFromPath(path);
    if (!vertexDomElement) {
        VGC_ERROR(
            LogVgcWorkspace,
            "Path attribute `{}` of element `{}` could not be resolved ({}).",
            name,
            domElem->tagName(),
            path);
        return nullptr;
    }
    return vertexDomElement;
}

void Workspace::rebuildVacFromTree_() {

    namespace ss = dom::strings;

    if (!document_ || !vac_ || !vgcElement_) {
        return;
    }
    isVacBeingUpdated_ = true;

    vac_->clear();
    // now, all workspace elements have a dangling vac node pointer.
    for (const std::pair<const core::Id, std::unique_ptr<Element>>& p : elements_) {
        p.second->vacNode_ = nullptr;
    }

    detail::VacElementLists ce;
    fillVacElementListsUsingTagNameRecursive(vgcElement_, ce);

    for (Element* e : ce.groups) {
        // create an unlinked VacGroup
        topology::VacGroup* node = topology::ops::createVacGroup(
            e->domElement_->internalId(),
            static_cast<topology::VacGroup*>(e->parent()->vacNode_));
        e->vacNode_ = node;
        // todo: set attributes
        // ...
    }

    std::set<Element*> parentsToOrderSync;

    for (Element* e : ce.keyVertices) {
        topology::KeyVertex* node = topology::ops::createKeyVertex(
            e->domElement_->internalId(),
            static_cast<topology::VacGroup*>(e->parent()->vacNode_));
        e->vacNode_ = node;
        // todo: set attributes
        // ...
    }

    for (Element* e : ce.keyEdges) {
        dom::Element* domElem = e->domElement_;
        dom::Element* ev0 = getElementRefAttribute(domElem, ss::startVertex);
        dom::Element* ev1 = getElementRefAttribute(domElem, ss::endVertex);
        if (!ev0 || !ev1) {
            continue;
        }
        if (ev0->tagName() != ss::vertex) {
            core::format(
                "Path attribute `{}` of element `{}` must refer to an element `{}`.",
                ss::startVertex,
                domElem->tagName(),
                ss::vertex);
        }
        if (ev1->tagName() != ss::vertex) {
            core::format(
                "Path attribute `{}` of element `{}` must refer to an element `{}`.",
                ss::endVertex,
                domElem->tagName(),
                ss::vertex);
        }
        Element* wv0 = elements_[ev0->internalId()].get();
        Element* wv1 = elements_[ev1->internalId()].get();

        auto v0 = topology::static_cell_cast<topology::KeyVertex>(
            static_cast<topology::VacCell*>(wv0->vacNode_));

        auto v1 = wv1->vacNode_->toCellUnchecked()->toKeyVertexUnchecked();

        topology::KeyEdge* node = topology::ops::createKeyEdge(
            domElem->internalId(),
            static_cast<topology::VacGroup*>(e->parent()->vacNode_),
            v0,
            v1);
        e->vacNode_ = node;

        // todo: set attributes
        // ...
    }

    //

    lastSyncedDomVersion_ = document_->version();
    lastSyncedVacVersion_ = vac_->version();
}

void Workspace::fillVacElementListsUsingTagNameRecursive(
    Element* e,
    detail::VacElementLists& ce) const {

    namespace ss = dom::strings;
    core::StringId tagName = e->domElement_->tagName();
    if (tagName == ss::vertex) {
        ce.keyVertices.append(e);
    }
    else if (tagName == ss::edge) {
        ce.keyEdges.append(e);
    }
    else if (tagName == ss::layer) {
        ce.groups.append(e);
    }

    //if (e->next())
}

} // namespace vgc::workspace
