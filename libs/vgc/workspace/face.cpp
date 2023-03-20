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

#include <vgc/core/span.h>
#include <vgc/workspace/edge.h>
#include <vgc/workspace/face.h>

#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

const VacFaceCellFrameData* VacFaceCell::computeStandaloneGeometryAt(core::AnimTime t) {
    VacFaceCellFrameData* data = frameData(t);
    if (data) {
        computeStandaloneGeometry(*data);
    }
    return data;
}

const VacFaceCellFrameData* VacFaceCell::computeGeometryAt(core::AnimTime t) {
    VacFaceCellFrameData* data = frameData(t);
    if (data) {
        computeStandaloneGeometry(*data);
        computeGeometry(*data);
    }
    return data;
}

VacKeyFace::~VacKeyFace() {
}

geometry::Rect2d VacKeyFace::boundingBox(core::AnimTime /*t*/) const {
    return bbox_;
}

bool VacKeyFace::isSelectableAt(
    const geometry::Vec2d& p,
    bool outlineOnly,
    double tol,
    double* outDistance,
    core::AnimTime t) const {

    using Vec2d = geometry::Vec2d;

    if (bbox_.isEmpty()) {
        return false;
    }

    geometry::Rect2d inflatedBbox = bbox_;
    inflatedBbox.setPMin(inflatedBbox.pMin() - Vec2d(tol, tol));
    inflatedBbox.setPMax(inflatedBbox.pMax() + Vec2d(tol, tol));
    if (!inflatedBbox.contains(p)) {
        return false;
    }

    // todo
    return false;
}

ElementStatus VacKeyFace::updateFromDom_(Workspace* workspace) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    // todo

    onUpdateError_();
    return ElementStatus::InvalidAttribute;
}

void VacKeyFace::onDependencyRemoved_(Element* dependency) {
}

void VacKeyFace::preparePaint_(core::AnimTime t, PaintOptions /*flags*/) {
    // todo, use paint options to not compute everything or with lower quality
    computeGeometryAt(t);
}

void VacKeyFace::paint_(graphics::Engine* engine, core::AnimTime t, PaintOptions flags)
    const {

    topology::KeyFace* ke = vacKeyFaceNode();
    if (!ke || t != ke->time()) {
        return;
    }

    // if not already done (should we leave preparePaint_ optional?)
    const_cast<VacKeyFace*>(this)->computeGeometry(frameData_);

    using namespace graphics;
    namespace ds = dom::strings;

    const dom::Element* const domElement = this->domElement();
    // XXX "implicit" cells' domElement would be the composite ?

    constexpr PaintOptions fillOptions = {PaintOption::Selected, PaintOption::Draft};

    // XXX todo: reuse geometry objects, create buffers separately (attributes waiting in FaceGraphics).

    FaceGraphics& graphics = frameData_.graphics_;

    // todo
}

VacFaceCellFrameData* VacKeyFace::frameData(core::AnimTime t) const {
    vacomplex::FaceCell* cell = vacFaceCellNode();
    if (!cell) {
        return nullptr;
    }
    if (frameData_.time() == t) {
        return &frameData_;
    }
    return nullptr;
}

void VacKeyFace::computeStandaloneGeometry(VacFaceCellFrameData& data) {

    // todo
}

void VacKeyFace::computeGeometry(VacFaceCellFrameData& data) {

    if (data.isGeometryComputed_ || data.isComputing_) {
        return;
    }
    topology::KeyFace* ke = vacKeyFaceNode();
    if (!ke) {
        return;
    }

    computeStandaloneGeometry(data);

    data.isComputing_ = true;
    data.isGeometryComputed_ = true;
    data.isComputing_ = false;

    // XXX clear less ?
    data.graphics_.clear();
}

void VacKeyFace::onInputGeometryChanged() {
    frameData_.clear();
    bbox_ = geometry::Rect2d::empty;
}

void VacKeyFace::onUpdateError_() {
    removeVacNode();
}

} // namespace vgc::workspace
