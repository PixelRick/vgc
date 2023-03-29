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

#ifndef VGC_WORKSPACE_VERTEX_H
#define VGC_WORKSPACE_VERTEX_H

#include <vgc/core/arithmetic.h>
#include <vgc/core/array.h>
#include <vgc/core/span.h>
#include <vgc/dom/element.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/element.h>

namespace vgc::workspace {

class VacVertexCell;
class VacKeyVertex;
class VacInbetweenVertex;
class VacEdgeCell;
class VacEdgeCellFrameData;
class VacVertexCellFrameData;

namespace detail {

class VacJoinFrameData;

// references an incident halfedge in a join
class VGC_WORKSPACE_API VacJoinHalfedge {
public:
    friend VacVertexCell;

    VacJoinHalfedge(VacEdgeCell* edgeCell, bool isReverse, Int32 group)
        : edgeCell_(edgeCell)
        , group_(group)
        , isReverse_(isReverse) {
    }

    VacEdgeCell* edgeCell() const {
        return edgeCell_;
    }

    bool isReverse() const {
        return isReverse_;
    }

    Int group() const {
        return group_;
    }

    struct GrouplessEqualTo {
        constexpr bool
        operator()(const VacJoinHalfedge& lhs, const VacJoinHalfedge& rhs) const {
            return lhs.edgeCell_ == rhs.edgeCell_ && lhs.isReverse_ == rhs.isReverse_;
        }
    };

private:
    VacEdgeCell* edgeCell_;
    Int32 group_;
    bool isReverse_;
};

// outgoing halfedge
class VGC_WORKSPACE_API VacJoinHalfedgeFrameData {
public:
    friend VacVertexCell;
    friend VacVertexCellFrameData;
    friend VacJoinFrameData;

    VacJoinHalfedgeFrameData(const VacJoinHalfedge& halfedge)
        : halfedge_(halfedge) {
    }

    const VacJoinHalfedge& halfedge() const {
        return halfedge_;
    }

    VacEdgeCell* edgeCell() const {
        return halfedge_.edgeCell();
    }

    bool isReverse() const {
        return halfedge_.isReverse();
    }

    Int group() const {
        return halfedge_.group();
    }

    double angle() const {
        return angle_;
    }

    double angleToNext() const {
        return angleToNext_;
    }

private:
    VacJoinHalfedge halfedge_;
    VacEdgeCellFrameData* edgeData_ = nullptr;
    geometry::Vec2d outgoingTangent_ = {};
    geometry::Vec2d halfwidths_ = {};
    geometry::Vec2d patchCutLimits_ = {};
    double patchLength_ = 0;
    struct SidePatchData {
        // straight join model data
        double filletLength = 0;
        double joinHalfwidth = 0;
        bool isCutFillet = false;
        double extLength = 0;
        //Ray borderRay = {};

        void clear() {
            filletLength = 0;
            joinHalfwidth = 0;
            isCutFillet = false;
            extLength = 0;
        }
    };
    std::array<SidePatchData, 2> sidePatchData_ = {};
    core::Array<geometry::CurveSample> workingSamples_;
    double angle_ = 0.0;
    double angleToNext_ = 0.0;

    void clearResult() {
        workingSamples_.clear();
        sidePatchData_[0].clear();
        sidePatchData_[1].clear();
    }
};

class VGC_WORKSPACE_API VacJoinFrameData {
private:
    friend VacVertexCellFrameData;
    friend VacVertexCell;

    core::Array<VacJoinHalfedgeFrameData> halfedgesData_;

    void clear() {
        halfedgesData_.clear();
    }

    void clearResults() {
        for (VacJoinHalfedgeFrameData& he : halfedgesData_) {
            he.clearResult();
        }
    }
};

} // namespace detail

class VGC_WORKSPACE_API VacVertexCellFrameData {
public:
    friend VacVertexCell;
    friend VacKeyVertex;
    friend VacInbetweenVertex;

    VacVertexCellFrameData(const core::AnimTime& t)
        : time_(t) {
    }

    const core::AnimTime& time() const {
        return time_;
    }

    bool hasPosition() const {
        return isPositionComputed_;
    }

    const geometry::Vec2d& position() const {
        return position_;
    }

    void debugPaint(graphics::Engine* engine);

private:
    core::AnimTime time_;
    geometry::Vec2d position_ = core::noInit;
    mutable graphics::GeometryViewPtr joinDebugLinesRenderGeometry_;
    mutable graphics::GeometryViewPtr joinDebugQuadRenderGeometry_;
    detail::VacJoinFrameData joinData_;
    bool isPositionComputed_ = false;
    bool isJoinComputed_ = false;
    bool isComputing_ = false;

    bool hasJoinData() const {
        return isJoinComputed_;
    }

    void clearJoinData() {
        position_ = {};
        joinData_.clearResults();
        isJoinComputed_ = false;
        joinDebugLinesRenderGeometry_.reset();
        joinDebugQuadRenderGeometry_.reset();
    }

    void clearJoinResults() {
        joinData_.clearResults();
        isJoinComputed_ = false;
    }
};

class VGC_WORKSPACE_API VacVertexCell : public VacElement {
private:
    friend class Workspace;
    friend class VacEdgeCell;
    friend class VacKeyEdge;
    friend class VacInbetweenVEdge;

protected:
    VacVertexCell(Workspace* workspace)
        : VacElement(workspace) {
    }

public:
    vacomplex::VertexCell* vacVertexCellNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toVertexCellUnchecked() : nullptr;
    }

    virtual const VacVertexCellFrameData* computeFrameDataAt(core::AnimTime t) = 0;

protected:
    void rebuildJoinHalfedgesArray() const;
    void clearJoinHalfedgesJoinData() const;

    // only dirty results but keep precomputed data in VacJoinHalfedgeFrameData
    virtual void dirtyJoinResults() = 0;

private:
    mutable core::Array<detail::VacJoinHalfedge> joinHalfedges_;

    void addJoinHalfedge_(const detail::VacJoinHalfedge& joinHalfedge);
    void removeJoinHalfedge_(const detail::VacJoinHalfedge& joinHalfedge);

    // should both dirty results and clear corresponding VacJoinHalfedgeFrameData
    virtual void onJoinEdgeGeometryChange_(VacEdgeCell* edge) = 0;
};

class VGC_WORKSPACE_API VacKeyVertex final : public VacVertexCell {
private:
    friend class Workspace;
    friend class VacEdgeCell;
    friend class VacKeyEdge;
    friend class VacInbetweenVEdge;

public:
    ~VacKeyVertex() override = default;

    VacKeyVertex(Workspace* workspace)
        : VacVertexCell(workspace)
        , frameData_({}) {
    }

    vacomplex::KeyVertex* vacKeyVertexNode() const {
        vacomplex::Cell* cell = vacCellUnchecked();
        return cell ? cell->toKeyVertexUnchecked() : nullptr;
    }

    std::optional<core::StringId> domTagName() const override;

    geometry::Rect2d boundingBox(core::AnimTime t) const override;

    bool isSelectableAt(
        const geometry::Vec2d& pos,
        bool outlineOnly,
        double tol,
        double* outDistance = nullptr,
        core::AnimTime t = {}) const override;

protected:
    void onPaintDraw(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

    void dirtyJoinResults() override;

private:
    mutable VacVertexCellFrameData frameData_;

    ElementStatus updateFromDom_(Workspace* workspace) override;
    void updateFromVac_() override;

    void onJoinEdgeGeometryChange_(VacEdgeCell* edge) override;

    void computePosition_();
    void computeJoin_();
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_VERTEX_H
