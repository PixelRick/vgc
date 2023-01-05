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

#include <vgc/dom/document.h>
#include <vgc/dom/element.h>
#include <vgc/workspace/element.h>

namespace vgc::workspace {

bool Element::updateFromDom(Workspace* workspace) {
    if (isBeingUpdated_) {
        VGC_ERROR(LogVgcWorkspace, "Cyclic update dependency detected.");
        return false;
    }
    // if not already up-to-date
    core::Id domVersion = domElement()->document()->versionId();
    if (domVersion_ != domVersion) {
        isBeingUpdated_ = true;
        updateFromDom_(workspace);
        isBeingUpdated_ = false;
        domVersion_ = domVersion;
    }
    return true;
}

geometry::Rect2d Element::boundingBox(core::AnimTime /*t*/) const {
    return geometry::Rect2d::empty;
}

void Element::updateFromDom_(Workspace* workspace) {
}

void Element::prepareForFrame_(core::AnimTime /*t*/) {
}

void Element::paint_(
    graphics::Engine* /*engine*/,
    core::AnimTime /*t*/,
    PaintOptions /*flags*/) const {

    // XXX make it pure virtual once the factory is in.
}

VacElement* Element::findFirstSiblingVacElement_(Element* start) {
    Element* e = start;
    while (e && !e->isVacElement()) {
        e = e->next();
    }
    return static_cast<VacElement*>(e);
}

VacElement::~VacElement() {
    if (vacNode_) {
        topology::ops::removeNode(vacNode_, false);
        vacNode_ = nullptr;
    }
}

} // namespace vgc::workspace
