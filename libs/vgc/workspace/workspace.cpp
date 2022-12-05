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

    document->changed().connect(onDocumentChanged());

    vac_ = topology::Vac::create();
    vac_->changed().connect(onVacChanged());

    initVacFromDocument();
}

//Renderable* Workspace::createRenderable(
//    graphics::Engine* engine,
//    core::AnimTime t,
//    core::Id id) {
//
//    // todo
//}

void Workspace::onDocumentChanged_(const dom::Diff& diff) {
    if (isSyncOngoing_) {
        return;
    }

    namespace ss = dom::strings;

    // XXX todo: build graphics objects at the same time and save cell pointers in.

    // we have to create the new elements in the right order

    // first create deffered everything new

    /*
    processing order:
    //
    core::Array<Node*> createdNodes_;
    std::unordered_map<Element*, std::set<core::StringId>> modifiedElements_;
    std::set<Node*> reparentedNodes_;
    core::Array<Node*> removedNodes_;
    std::set<Node*> childrenReorderedNodes_;
    */

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

    std::set<topology::VacCell*> aliveStar;
}

void Workspace::onVacChanged_(const topology::VacDiff& /*diff*/) {
    if (isSyncOngoing_) {
        return;
    }

    //
}

void Workspace::initVacFromDocument() {
    if (!document_) {
        return;
    }

    //
}

} // namespace vgc::workspace
