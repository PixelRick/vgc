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

#include <vgc/topology/operations.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

Workspace::Workspace(dom::DocumentPtr document)
    : document_(document) {

    document->changed().connect(onDocumentDiff());

    vac_ = topology::Vac::create();
    vac_->changed().connect(onVacDiff());

    initVacFromDom_();
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

void Workspace::updateFromDom() {
    if (!document_ || !vac_) {
        return;
    }

    isUpdatingFromDom_ = true;
    document_->emitPendingDiff();
    vac_->emitPendingDiff();
    isUpdatingFromDom_ = false;
}

void Workspace::updateFromVac() {
    if (!document_ || !vac_) {
        return;
    }

    isUpdatingFromVac_ = true;
    vac_->emitPendingDiff();
    document_->emitPendingDiff();
    isUpdatingFromVac_ = false;
}

void Workspace::sync() {
    // XXX todo
}

void Workspace::onDocumentDiff_(const dom::Diff& diff) {
    if (isUpdatingFromVac_) {
        return;
    }

    namespace ss = dom::strings;

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

    core::Array<std::unique_ptr<topology::VacCell>> createdCells;

    for (dom::Node* n : diff.createdNodes()) {
        dom::Element* e = dom::Element::cast(n);
        if (!e) {
            continue;
        }
        if (e->tagName() == ss::vertex) {
            std::unique_ptr<topology::KeyVertex> v =
                topology::ops::createUnlinkedKeyVertex(e->internalId(), core::AnimTime());
            /*
                {"color", core::colors::black},
                {"position", geometry::Vec2d()},
            */
            topology::ops::setKeyVertexPosition(
                v.get(), e->getAttribute(ss::position).getVec2d());

            // XXX create graphics obj
        }
        else if (e->tagName() == ss::edge) {
            std::unique_ptr<topology::KeyEdge> v =
                topology::ops::createUnlinkedKeyEdge(e->internalId(), core::AnimTime());
            /*
                {"color", core::colors::black},
                {"positions", geometry::Vec2dArray()},
                {"widths", core::DoubleArray()},
                {"startVertex", dom::Path()},
                {"endVertex", dom::Path()},
            */

            // will we deal with scripts here or will dom do it itself ?
        }
        else if (e->tagName() == ss::layer) {
            std::unique_ptr<topology::VacGroup> g =
                topology::ops::createUnlinkedVacGroup(e->internalId());
        }
    }

    for (const auto& me : diff.modifiedElements()) {
        dom::Element* e = me.first;
        const std::set<core::StringId>& attrs = me.second;
        if (!e) {
            continue;
        }
        if (e->tagName() == ss::vertex) {
            //
        }
        else if (e->tagName() == ss::edge) {
            // What if startVertex or endVertex changes ?
            // It is not really a valid vac operation.
            //
        }
        else if (e->tagName() == ss::layer) {
            //
        }
    }

    for (dom::Node* n : diff.reparentedNodes()) {
        dom::Element* e = dom::Element::cast(n);
        if (!e) {
            continue;
        }
        if (e->tagName() == ss::vertex) {
            //
        }
        else if (e->tagName() == ss::edge) {
            //
        }
        else if (e->tagName() == ss::layer) {
            //
        }
    }

    std::set<topology::VacCell*> aliveStar;
}

void Workspace::onVacDiff_(const topology::VacDiff& /*diff*/) {
    if (isUpdatingFromDom_) {
        return;
    }

    //
}

void Workspace::initVacFromDom_() {
    if (!document_ || !vac_) {
        return;
    }
    isUpdatingFromDom_ = true;

    vac_->clear();

    // see onDocumentDiff_

    lastSyncedDomVersion_ = document_->version();
    lastSyncedVacVersion_ = vac_->version();
}

} // namespace vgc::workspace
