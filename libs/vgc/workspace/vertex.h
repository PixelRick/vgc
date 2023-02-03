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
#include <vgc/dom/element.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/element.h>

namespace vgc::workspace {

class VacEdgeCell;
class EdgeGeometryCache;

namespace detail {

struct VacJoinEdgeFrameData {
    VacEdgeCell* element_;
    // what to do if it dies ?
    topology::vacomplex::EdgeCell* vacNode_;
    EdgeGeometryCache* geometry_;
    geometry::Vec2d outgoingTangent_;
    double angle_;
    double angleToNext_;
    Int numSamples_;
    bool isReverse_;
    bool isSliceDirty_;
};

class VGC_WORKSPACE_API VacVertexCellFrameData {
public:
    // join computation requires collaboration
    friend class VacVertexCell;
    friend class VacKeyVertex;
    friend class VacInbetweenVertex;
    friend class VacEdgeCell;
    friend class VacKeyEdge;
    friend class VacInbetweenEdge;

    void reset(geometry::Vec2d pos) {
        debugLinesRenderGeometry_.reset();
        debugQuadRenderGeometry_.reset();
        edges_.clear();
        pos_ = pos;
        isComputingJoins_ = false;
        areJoinsDirty_ = true;
    }

private:
    void computeJoins_();

    mutable graphics::GeometryViewPtr debugLinesRenderGeometry_;
    mutable graphics::GeometryViewPtr debugQuadRenderGeometry_;
    core::Array<detail::VacJoinEdgeFrameData> edges_;
    geometry::Vec2d pos_;
    bool isComputingJoins_ = false;
    bool areJoinsDirty_ = false;
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
    topology::vacomplex::VertexCell* vacVertexCellNode() const {
        return vacCellUnchecked()->toVertexCellUnchecked();
    }

protected:
    virtual detail::VacVertexCellFrameData* getOrCreateFrameData(core::AnimTime t) = 0;

    static void
    debugPaint(graphics::Engine* engine, const detail::VacVertexCellFrameData& data);
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

    detail::VacVertexCellFrameData* getOrCreateFrameData(core::AnimTime t) override;

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

private:
    detail::VacVertexCellFrameData frameData_;

    void debugPaint_(graphics::Engine* engine) const;
};

class VGC_WORKSPACE_API VacInbetweenVertex : public VacVertexCell {
private:
    friend class Workspace;

public:
    ~VacInbetweenVertex() override = default;

    VacInbetweenVertex(Workspace* workspace, dom::Element* domElement)
        : VacVertexCell(workspace, domElement) {
    }

    topology::vacomplex::InbetweenVertex* vacInbetweenVertexNode() const {
        return vacCellUnchecked()->toInbetweenVertexUnchecked();
    }

    geometry::Rect2d boundingBox(core::AnimTime t) const override;

    detail::VacVertexCellFrameData* getOrCreateFrameData(core::AnimTime t) override;

protected:
    ElementStatus updateFromDom_(Workspace* workspace) override;
    void preparePaint_(core::AnimTime t, PaintOptions flags) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) const override;

private:
    core::Array<std::pair<core::AnimTimeRange, detail::VacVertexCellFrameData>> cache_;
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_VERTEX_H
