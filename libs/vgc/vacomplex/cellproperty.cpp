// Copyright 2023 The VGC Developers
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

#include <vgc/vacomplex/cellproperty.h>

namespace vgc::vacomplex {

CellProperty::OpResult CellProperty::onTranslate_(const geometry::Vec2d& delta) {
    return OpResult::Unchanged;
}

CellProperty::OpResult CellProperty::onTransform_(const geometry::Mat3d& transformation) {
    return OpResult::Unchanged;
}

std::unique_ptr<CellProperty> CellProperty::onKeyEdgeConcat_(
    DirectedCellProperty hep1,
    DirectedCellProperty hep2) const {

    return nullptr;
}

std::unique_ptr<CellProperty>
CellProperty::onKeyEdgeGlue_(core::Span<DirectedCellProperty> heps) const {
    return nullptr;
}

CellProperty::OpResult CellProperty::onKeyEdgeSnap_(
    const geometry::Vec2d& snapStartPosition,
    const geometry::Vec2d& snapEndPosition) {

    return OpResult::Unchanged;
}

CellProperty::OpResult CellProperty::onKeyEdgeStrokeChanged_(
    geometry::AbstractStroke2d* oldStroke,
    geometry::AbstractStroke2d* newStroke) {

    return OpResult::Unchanged;
}

std::unique_ptr<CellProperty>
CellProperty::onKeyFaceGlue_(core::Span<CellProperty> heps) const {
    return nullptr;
}

CellProperty::OpResult CellProperty::onOperationEnd_() {
    return OpResult::Unchanged;
}

} // namespace vgc::vacomplex
