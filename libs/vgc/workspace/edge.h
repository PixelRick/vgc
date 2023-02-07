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
    Int centerPointSampleIndex = -1;
    geometry::Vec2d centerPoint;
    geometry::Vec2d sidePoint;
    geometry::Vec2d st;
};

struct EdgeJoinPatch {

    void clear() {
        samples.clear();
        sampleOverride_ = 0;
    }

    core::Array<EdgeJoinPatchSample> samples;
    bool isExtension = false;
    Int sampleOverride_ = 0;
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

    void clearJoinData() {
        for (auto& patch : patches_) {
            patch.clear();
        }
        graphics_.clear();
        isGeometryComputed_ = false;
    }

    const core::AnimTime& time() const {
        return time_;
    }

private:
    core::AnimTime time_;
    geometry::CurveSampleArray samples_;
    geometry::Vec2dArray triangulation_; // to remove
    geometry::Vec2fArray controlPoints_;
    Int samplingVersion_ = -1;
    int edgeTesselationMode_ = -1;
    // 0: start left patch
    // 1: start right patch
    // 2: end left patch
    // 3: end right patch
    std::array<detail::EdgeJoinPatch, 4> patches_;
    // isExtension is true if cap is active
    std::array<detail::EdgeJoinPatch, 2> caps_;
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
    topology::EdgeCell* vacEdgeCellNode() const {
        return vacCellUnchecked()->toEdgeCellUnchecked();
    }

    // to be called when standalone geometry changes
    virtual void clearFramesData() = 0;
    virtual void clearFramesJoinData() = 0;
    // to be called when standalone geometry changes or this edge gets removed
    virtual void clearVerticesFramesJoinData() = 0;
    // to be called when vertices are changed or this edge gets removed
    virtual void clearVerticesJoinTopologyData() = 0;

    const VacEdgeCellFrameData* computeStandaloneGeometry(core::AnimTime t);
    const VacEdgeCellFrameData* computeGeometry(core::AnimTime t);

protected:
    virtual VacEdgeCellFrameData* frameData(core::AnimTime t) const = 0;
    virtual void computeStandaloneGeometry_(VacEdgeCellFrameData& data) = 0;
    virtual void computeGeometry_(VacEdgeCellFrameData& data) = 0;
};

class VGC_WORKSPACE_API VacKeyEdge : public VacEdgeCell {
private:
    friend class Workspace;
    friend class VacKeyVertex; // for joins and caps

public:
    ~VacKeyEdge() override = default;

    VacKeyEdge(Workspace* workspace, dom::Element* domElement)
        : VacEdgeCell(workspace, domElement) {
    }

    topology::KeyEdge* vacKeyEdgeNode() const {
        return vacCellUnchecked()->toKeyEdgeUnchecked();
    }

    void setTesselationMode(int mode) {
        edgeTesselationModeRequested_ = core::clamp(mode, 0, 2);
    }

    void clearFramesData() override;
    void clearFramesJoinData() override;
    void clearVerticesFramesJoinData() override;
    void clearVerticesJoinTopologyData() override;

    geometry::Rect2d boundingBox(core::AnimTime t) const override;

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;

    void preparePaint_(core::AnimTime t, PaintOptions flags) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

    void onVacNodeRemoved_() override;

    VacEdgeCellFrameData* frameData(core::AnimTime t) const override;
    void computeStandaloneGeometry_(VacEdgeCellFrameData& data) override;
    void computeGeometry_(VacEdgeCellFrameData& data) override;

private:
    struct VertexInfo {
        VacKeyVertex* element;
        Int joinGroup = 0;
    };

    std::array<VertexInfo, 2> vertices_ = {};

    mutable VacEdgeCellFrameData frameData_ = {};
    int edgeTesselationModeRequested_ = 2;

    void onUpdateError_();
    void computeJoin_(VacVertexCell* v, bool isStart);
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_EDGE_H
