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

#include <vgc/topology/vac.h>

namespace vgc::topology {

void Vac::onDestroyed() {
}

VacPtr Vac::create() {
    return VacPtr(new Vac());
}

bool Vac::emitPendingDiff() {

    // Vac doesn't keep a relatives map

    //for (const auto& [node, oldRelatives] : previousRelativesMap_) {
    //    if (pendingDiff_.createdNodes_.contains(node)) {
    //        continue;
    //    }
    //    if (pendingDiff_.removedNodes_.contains(node)) {
    //        continue;
    //    }

    //    NodeRelatives newRelatives(node);
    //    if (oldRelatives.parent_ != newRelatives.parent_) {
    //        pendingDiff_.reparentedNodes_.insert(node);
    //    }
    //    else if (
    //        oldRelatives.nextSibling_ != newRelatives.nextSibling_
    //        || oldRelatives.previousSibling_ != newRelatives.previousSibling_) {
    //        // this introduces false positives if old siblings were removed or
    //        // new siblings were added.
    //        pendingDiff_.childrenReorderedNodes_.insert(newRelatives.parent_);
    //    }
    //}
    //previousRelativesMap_.clear();

    //// remove created and removed nodes from modified elements
    //auto& modifiedElements = pendingDiff_.modifiedElements_;
    //for (auto it = modifiedElements.begin(), last = modifiedElements.end(); it != last;) {
    //    Node* node = it->first;
    //    if (pendingDiff_.createdNodes_.contains(node)) {
    //        it = modifiedElements.erase(it);
    //        continue;
    //    }
    //    if (pendingDiff_.removedNodes_.contains(node)) {
    //        it = modifiedElements.erase(it);
    //        continue;
    //    }
    //    ++it;
    //}

    //if (!pendingDiff_.isEmpty()) {

    //    // XXX todo: emit node signals in here ?

    //    changed().emit(pendingDiff_);
    //    pendingDiff_.reset();
    //    pendingDiffKeepAllocPointers_.clear();
    //    return true;
    //}

    return false;
}

} // namespace vgc::topology
