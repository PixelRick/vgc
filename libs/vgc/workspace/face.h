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

#ifndef VGC_WORKSPACE_FACE_H
#define VGC_WORKSPACE_FACE_H

#include <vgc/core/arithmetic.h>
#include <vgc/core/array.h>
#include <vgc/dom/element.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/geometry/vec2f.h>
#include <vgc/geometry/vec4f.h>
#include <vgc/graphics/engine.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/element.h>
#include <vgc/workspace/vertex.h>

namespace vgc::workspace {

struct FaceGraphics {

    void clear() {
        fillGeometry_.reset();
        selectionGeometry_.reset();
    }

    graphics::GeometryViewPtr fillGeometry_;
    graphics::GeometryViewPtr selectionGeometry_;
};

class VGC_WORKSPACE_API VacFaceCellFrameData {
private:
    friend class VacFaceCell;
    friend class VacKeyFace;
    friend class VacInbetweenFace;

public:
    void clear() {
        outlinePolyline_.clear();
        faceTesselationMode_ = -1;
        graphics_.clear();
        isStandaloneGeometryComputed_ = false;
        isGeometryComputed_ = false;
    }

    const core::AnimTime& time() const {
        return time_;
    }

    const geometry::Vec2dArray& outlinePolyline() const {
        return outlinePolyline_;
    }

private:
    core::AnimTime time_;
    geometry::Vec2dArray outlinePolyline_;
    Int samplingVersion_ = -1;
    int faceTesselationMode_ = -1;
    FaceGraphics graphics_;
    bool isStandaloneGeometryComputed_ = false;
    bool isGeometryComputed_ = false;
    bool isComputing_ = false;
};

class VGC_WORKSPACE_API VacFaceCell : public VacElement {
private:
    friend class Workspace;
    friend class VacVertexCell;

protected:
    VacFaceCell(Workspace* workspace, dom::Element* domElement)
        : VacElement(workspace, domElement) {
    }

public:
    vacomplex::FaceCell* vacFaceCellNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toFaceCellUnchecked() : nullptr;
    }

    const VacFaceCellFrameData* computeStandaloneGeometryAt(core::AnimTime t);
    const VacFaceCellFrameData* computeGeometryAt(core::AnimTime t);

protected:
    virtual VacFaceCellFrameData* frameData(core::AnimTime t) const = 0;
    virtual void computeStandaloneGeometry(VacFaceCellFrameData& data) = 0;
    virtual void computeGeometry(VacFaceCellFrameData& data) = 0;

    virtual void onInputGeometryChanged() = 0;
};

class VGC_WORKSPACE_API VacKeyFace : public VacFaceCell {
private:
    friend class Workspace;

public:
    ~VacKeyFace() override;

    VacKeyFace(Workspace* workspace, dom::Element* domElement)
        : VacFaceCell(workspace, domElement) {
    }

    vacomplex::KeyFace* vacKeyFaceNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toKeyFaceUnchecked() : nullptr;
    }

    void setTesselationMode(int mode) {
        int newMode = core::clamp(mode, 0, 2);
        if (faceTesselationModeRequested_ != newMode) {
            faceTesselationModeRequested_ = newMode;
            onInputGeometryChanged();
        }
    }

    geometry::Rect2d boundingBox(core::AnimTime t) const override;

    bool isSelectableAt(
        const geometry::Vec2d& pos,
        bool outlineOnly,
        double tol,
        double* outDistance = nullptr,
        core::AnimTime t = {}) const override;

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;

    void onDependencyRemoved_(Element* dependency) override;

    void preparePaint_(core::AnimTime t, PaintOptions flags) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

    VacFaceCellFrameData* frameData(core::AnimTime t) const override;
    void computeStandaloneGeometry(VacFaceCellFrameData& data) override;
    void computeGeometry(VacFaceCellFrameData& data) override;

    void onInputGeometryChanged() override;

private:
    // currently updated during computeStandaloneGeometry
    geometry::Rect2d bbox_ = {};

    mutable VacFaceCellFrameData frameData_ = {};
    int faceTesselationModeRequested_ = 2;

    void onUpdateError_();
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_FACE_H
