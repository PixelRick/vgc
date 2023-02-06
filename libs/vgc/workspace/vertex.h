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
class VacEdgeCell;
class VacEdgeCellFrameCache;

namespace detail {

// references an incident halfedge in a join
class VGC_WORKSPACE_API VacJoinHalfedge {
public:
    friend VacVertexCell;

    VacJoinHalfedge(VacEdgeCell* edgeCell, bool isReverse, Int32 group)
        : edgeCell_(edgeCell)
        , group_(group)
        , isReverse_(isReverse) {
    }

    constexpr VacEdgeCell* edgeCell() const {
        return edgeCell_;
    }

    constexpr bool isReverse() const {
        return isReverse_;
    }

private:
    VacEdgeCell* edgeCell_;
    Int32 group_;
    bool isReverse_;
};

class VGC_WORKSPACE_API VacJoinHalfedgeFrameCache {
public:
    friend VacVertexCell;

    VacJoinHalfedgeFrameCache(const VacJoinHalfedge& halfedge)
        : halfedge_(halfedge) {
    }

    const VacJoinHalfedge& halfedge() const {
        return halfedge_;
    }

    constexpr double angle() const {
        return angle_;
    }

    constexpr double angleToNext() const {
        return angleToNext_;
    }

private:
    VacJoinHalfedge halfedge_;
    VacEdgeCellFrameCache* edgeCache_ = nullptr;
    geometry::Vec2d outgoingTangent_ = {};
    double angle_ = 0.0;
    double angleToNext_ = 0.0;
};

class VGC_WORKSPACE_API VacVertexCellFrameCache {
public:
    friend VacVertexCell;

    VacVertexCellFrameCache(const core::AnimTime& t)
        : time_(t) {
    }

    void clearJoinData() {
        debugLinesRenderGeometry_.reset();
        debugQuadRenderGeometry_.reset();
        halfedges_.clear();
        isComputingJoin_ = false;
        isJoinComputed_ = false;
    }

    const core::AnimTime& time() const {
        return time_;
    }

    const geometry::Vec2d& pos() const {
        return pos_;
    }

    void debugPaint(graphics::Engine* engine);

private:
    core::AnimTime time_;
    geometry::Vec2d pos_ = core::noInit;
    mutable graphics::GeometryViewPtr debugLinesRenderGeometry_;
    mutable graphics::GeometryViewPtr debugQuadRenderGeometry_;
    // join data
    core::Array<detail::VacJoinHalfedgeFrameCache> halfedges_;
    bool isComputingJoin_ = false;
    bool isJoinComputed_ = false;
};

} // namespace detail

class VGC_WORKSPACE_API VacVertexCell : public VacElement {
private:
    friend class Workspace;
    friend class VacKeyVertex;
    friend class VacInbetweenVertex;

protected:
    VacVertexCell(Workspace* workspace, dom::Element* domElement)
        : VacElement(workspace, domElement) {
    }

public:
    vacomplex::VertexCell* vacVertexCellNode() const {
        return vacCellUnchecked()->toVertexCellUnchecked();
    }

    const core::Array<detail::VacJoinHalfedge>& joinHalfedges() const;

    void clearFrameCaches() const;
    void clearJoinCaches() const;

    detail::VacVertexCellFrameCache* computeJoin(core::AnimTime t) const;

protected:
    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

    detail::VacVertexCellFrameCache* frameCache(core::AnimTime t) const;
    void initFrameCache_(detail::VacVertexCellFrameCache& cache) const;
    void computeJoinHalfedgesData_(detail::VacVertexCellFrameCache& cache) const;
    void computeJoin_(detail::VacVertexCellFrameCache& cache) const;

private:
    mutable core::Array<detail::VacJoinHalfedge> joinHalfedges_;
    mutable core::Array<detail::VacVertexCellFrameCache> frameCacheEntries_;
    mutable bool isJoinHalfedgesDirty_ = true;

    void computeJoinHalfedges_() const;
};

class VGC_WORKSPACE_API VacKeyVertex : public VacVertexCell {
private:
    friend class Workspace;

public:
    ~VacKeyVertex() override = default;

    VacKeyVertex(Workspace* workspace, dom::Element* domElement)
        : VacVertexCell(workspace, domElement) {
    }

    topology::KeyVertex* vacKeyVertexNode() const {
        return vacCellUnchecked()->toKeyVertexUnchecked();
    }

    geometry::Rect2d boundingBox(core::AnimTime t) const override;

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;
};

class VGC_WORKSPACE_API VacInbetweenVertex : public VacVertexCell {
private:
    friend class Workspace;

public:
    ~VacInbetweenVertex() override = default;

    VacInbetweenVertex(Workspace* workspace, dom::Element* domElement)
        : VacVertexCell(workspace, domElement) {
    }

    vacomplex::InbetweenVertex* vacInbetweenVertexNode() const {
        return vacCellUnchecked()->toInbetweenVertexUnchecked();
    }

    geometry::Rect2d boundingBox(core::AnimTime t) const override;

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;
    void preparePaint_(core::AnimTime t, PaintOptions flags) override;
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_VERTEX_H
