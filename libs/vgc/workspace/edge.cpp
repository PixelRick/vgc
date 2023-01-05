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

#include <vgc/workspace/edge.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

geometry::Rect2d KeyEdge::boundingBox(core::AnimTime t) const {
    return geometry::Rect2d::empty;
}

void KeyEdge::updateFromDom_(Workspace* workspace) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    Element* ve0 =
        workspace->getElementFromPathAttribute(domElement, ds::startvertex, ds::vertex);
    Element* ve1 =
        workspace->getElementFromPathAttribute(domElement, ds::endvertex, ds::vertex);
    Element* parentElement = this->parent();

    // fallback to closed edge if only ev0 or ev1 is present
    if (!ve0) {
        ve0 = ve1;
    }
    if (!ve1) {
        ve1 = ve0;
    }

    topology::KeyVertex* kv0 = nullptr;
    topology::KeyVertex* kv1 = nullptr;
    topology::VacGroup* parentGroup = nullptr;

    // update dependencies (vertices)
    if (ve0) {
        ve0->updateFromDom(workspace);
        topology::VacNode* vn0 = ve0->vacNode();
        if (vn0) {
            kv0 = vn0->toCellUnchecked()->toKeyVertexUnchecked();
        }
    }
    if (ve1) {
        ve1->updateFromDom(workspace);
        topology::VacNode* vn1 = ve1->vacNode();
        if (vn1) {
            kv1 = vn1->toCellUnchecked()->toKeyVertexUnchecked();
        }
    }
    if (parentElement) {
        parentElement->updateFromDom(workspace);
        topology::VacNode* parentNode = parentElement->vacNode();
        if (parentNode) {
            // checked cast to group, could be something invalid
            parentGroup = parentNode->toGroup();
        }
    }

    topology::KeyEdge* ke = nullptr;

    // check if it needs to be rebuilt
    if (vacNode_) {
        ke = vacNode_->toCellUnchecked()->toKeyEdgeUnchecked();

        topology::KeyVertex* oldKv0 = ke->startVertex();
        topology::KeyVertex* oldKv1 = ke->endVertex();
        if (!parentGroup || kv0 != oldKv0 || kv1 != oldKv1) {
            topology::ops::removeNode(vacNode_, false);
            vacNode_ = nullptr;
            ke = nullptr;
        }
    }

    if (!vacNode_ && parentGroup) {
        if (kv0 && kv1) {
            ke = topology::ops::createKeyEdge(
                domElement->internalId(), parentGroup, kv0, kv1);
        }
        else {
            ke =
                topology::ops::createKeyClosedEdge(domElement->internalId(), parentGroup);
        }
        vacNode_ = ke;
    }

    if (ke) {
        const auto& points = domElement->getAttribute(ds::positions).getVec2dArray();
        const auto& widths = domElement->getAttribute(ds::widths).getDoubleArray();

        // these do nothing if the data didn't change
        topology::ops::setKeyEdgeCurvePoints(ke, points);
        topology::ops::setKeyEdgeCurveWidths(ke, widths);

        // XXX should we snap here ?
        //     group view matrices may not be ready..
        //     maybe we could add two init functions to workspace::Element
        //     one for intrinsic data, one for dependent data.
    }
}

void KeyEdge::paint_(
    graphics::Engine* /*engine*/,
    core::AnimTime /*t*/,
    PaintOptions /*flags*/) const {
}

} // namespace vgc::workspace
