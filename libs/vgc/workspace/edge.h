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
#include <vgc/geometry/vec4d.h>
#include <vgc/geometry/vec4f.h>
#include <vgc/graphics/engine.h>
#include <vgc/graphics/geometryview.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/element.h>
#include <vgc/workspace/vertex.h>

namespace vgc::workspace {

// Terminology:
// - Centerline: the sampled centerline from input geometry.
// - OffsetLines: the lines offseted by the varying halfwidths from the centerline,
//                and parameterized with corresponding centerline arclength.
// - JoinedLines: Centerline and OffsetLines with patches for joins/caps.
// - Stroke: Triangulated and parameterized "(s, t, u, v)" hull.
//           Optionally post-treated to remove overlaps. (Shader-ready)

// Component (drawn and selectable..)

class VGC_WORKSPACE_API StuvMesh {
public:
    StuvMesh() noexcept = default;

    void clear() {
        xyVertices_.clear();
        stuvVertices_.clear();
        indices_.clear();
    }

    /// Returns the xy component of vertices.
    const geometry::Vec2dArray& xyVertices() const {
        return xyVertices_;
    }

    /// Returns the stuv component of vertices.
    const geometry::Vec4dArray& stuvVertices() const {
        return stuvVertices_;
    }

    /// Returns the vertex indices.
    const core::IntArray& indices() const {
        return indices_;
    }

    bool isTriangleStrip() const {
        return isStrip_;
    }

    bool isTriangleList() const {
        return !isStrip_;
    }

    Int appendVertex(const geometry::Vec2d& position, const geometry::Vec4d& stuv) {
        Int idx = xyVertices_.length();
        xyVertices_.emplaceLast(position);
        stuvVertices_.emplaceLast(stuv);
        return idx;
    }

    void appendIndex(Int i) {
        const Int n = xyVertices_.length();
        if (i < 0 || i >= n) {
            throw core::IndexError("Vertex index is out of range.");
        }
        indices_.append(i);
    }

    void appendIndexUnchecked(Int i) {
        indices_.append(i);
    }

    void appendIndices(std::initializer_list<Int> is) {
        const Int n = xyVertices_.length();
        for (Int i : is) {
            if (i < 0 || i >= n) {
                throw core::IndexError("Vertex index is out of range.");
            }
        }
        indices_.extend(is);
    }

    void appendIndicesUnchecked(std::initializer_list<Int> is) {
        indices_.extend(is);
    }

    void setTopology(bool isStrip) {
        isStrip_ = isStrip;
    }

private:
    geometry::Vec2dArray xyVertices_;
    geometry::Vec4dArray stuvVertices_;
    core::IntArray indices_;
    bool isStrip_;
};

/// \enum vgc::workspace::StrokeStripGeometry
/// \brief Stores stroke geometry as a triangle strip of xy and stuv vertices.
///
class VGC_WORKSPACE_API StuvStripBuilder {
public:
    StuvStripBuilder(StuvMesh& mesh)
        : mesh_(mesh) {

        mesh_.clear();
        mesh_.setTopology(true);
    }

    Int appendVertex(const geometry::Vec2d& position, const geometry::Vec4d& stuv) {
        Int idx = meshVertices_.length();
        meshVertices_.emplaceLast(position);
        meshUVSTs_.emplaceLast(stuv);
        return idx;
    }

    Int continueStrip(const geometry::Vec2d& position, const geometry::Vec4d& stuv) {
        Int idx = appendVertex(position, stuv);
        continueStrip(idx);
        return idx;
    }

    void continueStrip(Int i) {
        const Int n = meshVertices_.length();
        if (i < 0 || i >= n) {
            throw core::IndexError("Vertex index is out of range.");
        }
        meshIndices_.append(i);
    }

private:
    ParameterizedMesh& mesh_;
};

class VGC_WORKSPACE_API StrokeGeometry {
public:
    void clear() {
        meshVertices_.clear();
        meshUVSTs_.clear();
        meshIndices_.clear();
    }

    const geometry::Vec2dArray& meshVertices() const {
        return meshVertices_;
    }

    const geometry::Vec4dArray& meshUVSTs() const {
        return meshUVSTs_;
    }

    const core::IntArray& meshIndices() const {
        return meshIndices_;
    }

    Int appendVertex(const geometry::Vec2d& position, const geometry::Vec4d& stuv) {
        Int idx = meshVertices_.length();
        meshVertices_.emplaceLast(position);
        meshUVSTs_.emplaceLast(stuv);
        return idx;
    }

    void appendTriangle(Int i0, Int i1, Int i2) {
        const Int n = meshVertices_.length();
        if ((i0 < 0 || i0 >= n) || (i1 < 0 || i1 >= n) || (i1 < 0 || i1 >= n)) {
            throw core::IndexError("Vertex index is out of range.");
        }
        meshIndices_.extend({i0, i1, i2});
    }

private:
    geometry::Vec2dArray meshVertices_;
    geometry::Vec4dArray meshUVSTs_;
    core::IntArray meshIndices_;
};

namespace detail {

graphics::GeometryViewPtr createStrokeGraphicsGeometry(
    graphics::Engine* engine,
    const StrokeStripGeometry& geometry);

graphics::GeometryViewPtr
createStrokeGraphicsGeometry(graphics::Engine* engine, const StrokeGeometry& geometry);

} // namespace detail

class VGC_WORKSPACE_API EdgeGraphics {
public:
    void clear() {
        centerlineGeometry_.reset();
        strokeGeometry_.reset();
        joinGeometry_.reset();
        selectionGeometry_.reset();
    }

    const graphics::GeometryViewPtr& centerlineGeometry() const {
        return centerlineGeometry_;
    }

    void setCenterlineGeometry(const graphics::GeometryViewPtr& centerlineGeometry) {
        centerlineGeometry_ = centerlineGeometry;
    }

    void clearCenterlineGeometry() {
        centerlineGeometry_.reset();
    }

    const graphics::GeometryViewPtr& strokeGeometry() const {
        return strokeGeometry_;
    }

    void setStrokeGeometry(const graphics::GeometryViewPtr& strokeGeometry) {
        strokeGeometry_ = strokeGeometry;
    }

    void clearStrokeGeometry() {
        strokeGeometry_.reset();
    }

    const graphics::GeometryViewPtr& joinGeometry() const {
        return joinGeometry_;
    }

    void setJoinGeometry(const graphics::GeometryViewPtr& joinGeometry) {
        joinGeometry_ = joinGeometry;
    }

    void clearJoinGeometry() {
        joinGeometry_.reset();
    }

    const graphics::GeometryViewPtr& selectionGeometry() const {
        return selectionGeometry_;
    }

    void setSelectionGeometry(const graphics::GeometryViewPtr& selectionGeometry) {
        selectionGeometry_ = selectionGeometry;
    }

    void clearSelectionGeometry() {
        selectionGeometry_.reset();
    }

private:
    // Centerline
    graphics::GeometryViewPtr centerlineGeometry_;

    // Stroke
    graphics::GeometryViewPtr strokeGeometry_;
    graphics::GeometryViewPtr joinGeometry_;

    // Selection (requires centerline)
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
    std::array<core::Array<EdgeJoinPatchSample>, 2> sideSamples;
    EdgeJoinPatchMergeLocation mergeLocation = {};
    bool isCap = false;

    void clear() {
        sideSamples[0].clear();
        sideSamples[1].clear();
        mergeLocation = {};
        isCap = false;
    }
};

} // namespace detail

enum class VacEdgeComputationStage : UInt8 {
    Clear,
    Centerline,
    OffsetLines,
    JoinedLines,
    Stroke,
    EnumMax = Stroke,
    EnumMin = Clear
};

inline constexpr VacEdgeComputationStage previousStage(VacEdgeComputationStage stage) {
    if (stage == VacEdgeComputationStage::EnumMin) {
        return stage;
    }
    return static_cast<VacEdgeComputationStage>(core::toUnderlying(stage) - 1);
}

inline constexpr VacEdgeComputationStage nextStage(VacEdgeComputationStage stage) {
    if (stage == VacEdgeComputationStage::EnumMax) {
        return stage;
    }
    return static_cast<VacEdgeComputationStage>(core::toUnderlying(stage) + 1);
}

class VGC_WORKSPACE_API VacEdgeCellFrameData {
private:
    friend class VacEdgeCell;
    friend class VacKeyEdge;
    friend class VacInbetweenEdge;
    friend class VacVertexCell;

public:
    VacEdgeCellFrameData() noexcept = default;

    VacEdgeComputationStage stage() const {
        return stage_;
    }

    const core::AnimTime& time() const {
        return time_;
    }

    const geometry::Vec2dArray& controlPoints() const {
        return controlPoints_;
    }

    const geometry::CurveSampleArray& samples() const {
        return samples_;
    }

    void clear() {
        resetToStage(VacEdgeComputationStage::Clear);
    }

    void clearStartJoinData() {
        resetToStage(VacEdgeComputationStage::JoinedLines);
        patches_[0].clear();
        resetToStage(previousStage(VacEdgeComputationStage::JoinedLines));
    }

    void clearEndJoinData() {
        resetToStage(VacEdgeComputationStage::JoinedLines);
        patches_[1].clear();
        resetToStage(previousStage(VacEdgeComputationStage::JoinedLines));
    }

    void resetToStage(VacEdgeComputationStage stage) {
        if (stage >= stage_) {
            return;
        }
        switch (stage_) {
        case VacEdgeComputationStage::Stroke:
            if (stage >= stage_) {
                break;
            }
            else {
                strokeStrip_.clear();
                stroke_.clear();
                graphics_.clearStrokeGeometry();
                graphics_.clearJoinGeometry();
                [[fallthrough]];
            }
        case VacEdgeComputationStage::JoinedLines:
            if (stage >= stage_) {
                break;
            }
            else {
                patches_[0].clear();
                patches_[1].clear();
                [[fallthrough]];
            }
        case VacEdgeComputationStage::OffsetLines:
            if (stage >= stage_) {
                break;
            }
            else {
                samples_.clear();
                triangulation_.clear();
                graphics_.clearCenterlineGeometry();
                graphics_.clearSelectionGeometry();
                [[fallthrough]];
            }
        case VacEdgeComputationStage::Centerline:
            if (stage >= stage_) {
                break;
            }
            else {
                samples_.clear();
                triangulation_.clear();
                [[fallthrough]];
            }
        case VacEdgeComputationStage::Clear:
            if (stage >= stage_) {
                break;
            }
            else {
                controlPoints_.clear();
                [[fallthrough]];
            }
        }
        stage_ = stage;
    }

private:
    core::AnimTime time_;
    geometry::Vec2dArray controlPoints_;

    // centerline + offsetLines stages
    geometry::CurveSampleArray samples_;
    geometry::Vec2dArray triangulation_; // to remove
    Int samplingVersion_ = -1;
    int edgeTesselationMode_ = -1;

    // joinedLines stage
    // [0]: start patch
    // [1]: end patch
    std::array<detail::EdgeJoinPatch, 2> patches_;

    // stroke stage
    StrokeStripGeometry strokeStrip_;
    StrokeGeometry stroke_;

    EdgeGraphics graphics_;
    VacEdgeComputationStage stage_ = VacEdgeComputationStage::Clear;
    bool isComputing_ = false;
};

class VGC_WORKSPACE_API VacEdgeCell : public VacElement {
private:
    friend class Workspace;
    friend class VacVertexCell;

protected:
    VacEdgeCell(Workspace* workspace)
        : VacElement(workspace) {
    }

public:
    vacomplex::EdgeCell* vacEdgeCellNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toEdgeCellUnchecked() : nullptr;
    }

    virtual const VacEdgeCellFrameData* computeCenterlineAt(core::AnimTime t) = 0;
    virtual const VacEdgeCellFrameData* computeOuterLinesAt(core::AnimTime t) = 0;
    virtual const StrokeGeometry* computeStrokeGeometryAt(core::AnimTime t) = 0;
};

class VGC_WORKSPACE_API VacKeyEdge final : public VacEdgeCell {
private:
    friend class Workspace;
    // for joins and caps
    friend class VacVertexCell;
    friend class VacKeyVertex;

public:
    ~VacKeyEdge() override;

    VacKeyEdge(Workspace* workspace)
        : VacEdgeCell(workspace) {
    }

    vacomplex::KeyEdge* vacKeyEdgeNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toKeyEdgeUnchecked() : nullptr;
    }

    void setTesselationMode(int mode);

    std::optional<core::StringId> domTagName() const override;

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

private:
    struct VertexInfo {
        VacKeyVertex* element = nullptr;
        Int joinGroup = 0;
    };

    std::array<VertexInfo, 2> verticesInfo_ = {};

    // currently updated during computeStandaloneGeometry
    geometry::Rect2d bbox_ = {};

    mutable VacEdgeCellFrameData frameData_ = {};
    int edgeTesselationModeRequested_ = 2;
    ChangeFlags alreadyNotifiedChanges_ = {};

    ElementStatus updateFromDom_(Workspace* workspace) override;
    void updateFromVac_() override;

    void onDependencyChanged_(Element* dependency, ChangeFlags changes) override;
    void onDependencyRemoved_(Element* dependency) override;

    void preparePaint_(core::AnimTime t, PaintOptions flags) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

    VacEdgeCellFrameData* frameData(core::AnimTime t) const;

    void onInputGeometryChanged();
    void onBoundaryGeometryChanged();

    void clearStartJoinData();
    void clearEndJoinData();
    void clearJoinData();

    void onUpdateError_();
    void computeStandaloneGeometry_();
    void computeGeometry_();
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_EDGE_H
