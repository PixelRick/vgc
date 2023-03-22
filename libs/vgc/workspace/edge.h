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

#ifndef VGC_WORKSPACE_EDGE_H
#define VGC_WORKSPACE_EDGE_H

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

// Component (drawn and selectable..)

struct CurveGraphics {

    void clear() {
        controlPoints_.reset();
        thickPolyline_.reset();
        numPoints_ = 0;
    }

    graphics::BufferPtr controlPoints_;
    graphics::BufferPtr thickPolyline_;
    Int numPoints_ = 0;
};

struct StrokeGraphics {

    void clear() {
        meshVertices_.reset();
        meshUVSTs_.reset();
        meshIndices_.reset();
    }

    graphics::BufferPtr meshVertices_;
    graphics::BufferPtr meshUVSTs_;
    graphics::BufferPtr meshIndices_;
};

struct EdgeGraphics {

    void clear() {
        curveGraphics_.clear();
        pointsGeometry_.reset();
        centerlineGeometry_.reset();
        strokeGraphics_.clear();
        strokeGeometry_.reset();
        selectionGeometry_.reset();
    }

    // Center line & Control points
    CurveGraphics curveGraphics_;
    graphics::GeometryViewPtr pointsGeometry_;
    graphics::GeometryViewPtr centerlineGeometry_;

    // Stroke
    StrokeGraphics strokeGraphics_;
    graphics::GeometryViewPtr strokeGeometry_;
    graphics::GeometryViewPtr joinGeometry_;
    graphics::GeometryViewPtr selectionGeometry_;
};

namespace detail {

struct EdgeJoinPatchSample {
    geometry::Vec2d centerPoint;
    geometry::Vec2d sidePoint;
    geometry::Vec4f sideSTUV;
    geometry::Vec2f centerSU;
    Int centerPointSampleIndex = -1;
};

struct EdgeJoinPatchMergeLocation {
    Int halfedgeNextSampleIndex = 0;
    double t = 1.0;
    geometry::CurveSample sample;
};

struct EdgeJoinPatch {

    void clear() {
        sideSamples[0].clear();
        sideSamples[1].clear();
        mergeLocation = {};
        isCap = false;
    }

    std::array<core::Array<EdgeJoinPatchSample>, 2> sideSamples;
    EdgeJoinPatchMergeLocation mergeLocation = {};
    bool isCap = false;
};

} // namespace detail

class VGC_WORKSPACE_API VacEdgeCellFrameData {
private:
    friend class VacEdgeCell;
    friend class VacKeyEdge;
    friend class VacInbetweenEdge;
    friend class VacVertexCell;

public:
    void clear() {
        samples_.clear();
        triangulation_.clear();
        controlPoints_.clear();
        for (auto& patch : patches_) {
            patch.clear();
        }
        edgeTesselationMode_ = -1;
        graphics_.clear();
        isStandaloneGeometryComputed_ = false;
        isGeometryComputed_ = false;
    }

    void clearStartJoinData() {
        patches_[0].clear();
        graphics_.clear();
        isGeometryComputed_ = false;
    }

    void clearEndJoinData() {
        patches_[1].clear();
        graphics_.clear();
        isGeometryComputed_ = false;
    }

    const core::AnimTime& time() const {
        return time_;
    }

    const geometry::CurveSampleArray& samples() const {
        return samples_;
    }

private:
    core::AnimTime time_;
    geometry::CurveSampleArray samples_;
    geometry::Vec2dArray triangulation_; // to remove
    geometry::Vec2fArray controlPoints_;
    Int samplingVersion_ = -1;
    int edgeTesselationMode_ = -1;
    // [0]: start patch
    // [1]: end patch
    std::array<detail::EdgeJoinPatch, 2> patches_;
    EdgeGraphics graphics_;
    bool isStandaloneGeometryComputed_ = false;
    bool isGeometryComputed_ = false;
    bool isComputing_ = false;
};

class VGC_WORKSPACE_API VacEdgeCell : public VacElement {
private:
    friend class Workspace;
    friend class VacVertexCell;

protected:
    VacEdgeCell(Workspace* workspace, dom::Element* domElement)
        : VacElement(workspace, domElement) {
    }

public:
    vacomplex::EdgeCell* vacEdgeCellNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toEdgeCellUnchecked() : nullptr;
    }

    virtual const VacEdgeCellFrameData* computeStandaloneGeometryAt(core::AnimTime t) = 0;
    virtual const VacEdgeCellFrameData* computeGeometryAt(core::AnimTime t) = 0;

protected:
    virtual VacEdgeCellFrameData* frameData(core::AnimTime t) const = 0;

    virtual void onInputGeometryChanged() = 0;
    virtual void onBoundaryGeometryChanged() = 0;

    virtual void clearStartJoinData() = 0;
    virtual void clearEndJoinData() = 0;
    virtual void clearJoinData() = 0;
};

class VGC_WORKSPACE_API VacKeyEdge final : public VacEdgeCell {
private:
    friend class Workspace;
    // for joins and caps
    friend class VacVertexCell;
    friend class VacKeyVertex;

public:
    ~VacKeyEdge() override;

    VacKeyEdge(Workspace* workspace, dom::Element* domElement)
        : VacEdgeCell(workspace, domElement) {
    }

    vacomplex::KeyEdge* vacKeyEdgeNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toKeyEdgeUnchecked() : nullptr;
    }

    void setTesselationMode(int mode) {
        int newMode = core::clamp(mode, 0, 2);
        if (edgeTesselationModeRequested_ != newMode) {
            edgeTesselationModeRequested_ = newMode;
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

    const VacEdgeCellFrameData* computeStandaloneGeometryAt(core::AnimTime t) override;
    const VacEdgeCellFrameData* computeGeometryAt(core::AnimTime t) override;

    const VacEdgeCellFrameData* computeStandaloneGeometry();
    const VacEdgeCellFrameData* computeGeometry();

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;

    void onDependencyRemoved_(Element* dependency) override;

    void preparePaint_(core::AnimTime t, PaintOptions flags) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

    VacEdgeCellFrameData* frameData(core::AnimTime t) const override;

    void onInputGeometryChanged() override;
    void onBoundaryGeometryChanged() override;

    void clearStartJoinData() override;
    void clearEndJoinData() override;
    void clearJoinData() override;

private:
    struct VertexInfo {
        VacKeyVertex* element;
        Int joinGroup = 0;
    };

    std::array<VertexInfo, 2> verticesInfo_ = {};

    // currently updated during computeStandaloneGeometry
    geometry::Rect2d bbox_ = {};

    mutable VacEdgeCellFrameData frameData_ = {};
    int edgeTesselationModeRequested_ = 2;

    void onUpdateError_();
    void computeStandaloneGeometry_();
    void computeGeometry_();
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_EDGE_H
