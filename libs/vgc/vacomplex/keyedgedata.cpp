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

#include <vgc/vacomplex/keyedgedata.h>

#include <vgc/vacomplex/detail/operationsimpl.h>
#include <vgc/vacomplex/keyedge.h>

namespace vgc::vacomplex {

KeyEdgeDataPtr KeyEdgeData::clone() const {
    KeyEdgeDataPtr result = std::make_shared<KeyEdgeData>(isClosed_);
    result->assignClonedProperties(this);
    if (stroke_) {
        result->stroke_ = stroke_->clone();
    }
}

void KeyEdgeData::translate(const geometry::Vec2d& delta) {
    if (stroke_) {
        stroke_->translate(delta);
    }
    CellData::translate(delta);
}

void KeyEdgeData::transform(const geometry::Mat3d& transformation) {
    if (!stroke_) {
        stroke_->transform(transformation);
    }
    CellData::transform(transformation);
}

//std::shared_ptr<KeyEdgeData>
//KeyEdgeData::merge(bool direction, KeyEdgeData* other, bool otherDirection) const {
//
//    // TODO: try both ways, if none works then convert both to a default geometry model for the merge
//    //       then rebuild with the best of the original models that supports being built from the default.
//    return merge_(direction, other, otherDirection);
//}
//
//void KeyEdgeData::dirtyEdgeSampling() const {
//    if (edge_) {
//        Complex* complex = edge_->complex();
//        detail::Operations ops(complex);
//        ops.onGeometryChanged_(edge_);
//    }
//}
//
//void KeyEdgeData::dirtyEdgeStyle() const {
//    if (edge_) {
//        Complex* complex = edge_->complex();
//        detail::Operations ops(complex);
//        ops.onStyleChanged_(edge_);
//    }
//}

} // namespace vgc::vacomplex
