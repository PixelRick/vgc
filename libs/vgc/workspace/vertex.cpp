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

#include <vgc/workspace/vertex.h>

#include <vgc/workspace/edge.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

namespace detail {

void VacVertexCellFrameData::debugPaint(graphics::Engine* engine) {

    using namespace graphics;
    using detail::VacJoinHalfedgeFrameData;

    if (!debugQuadRenderGeometry_) {
        debugQuadRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYRGB, IndexFormat::UInt16);
        geometry::Vec2f p(position_);
        core::FloatArray vertices({
            p.x() - 5, p.y() - 5, 0, 0, 0, //
            p.x() - 5, p.y() + 5, 0, 0, 0, //
            p.x() + 5, p.y() - 5, 0, 0, 0, //
            p.x() + 5, p.y() + 5, 0, 0, 0, //
        });
        engine->updateVertexBufferData(debugQuadRenderGeometry_, std::move(vertices));
        core::Array<UInt16> lineIndices({0, 1, 2, 3});
        engine->updateBufferData(
            debugQuadRenderGeometry_->indexBuffer(), std::move(lineIndices));
    }

    if (!debugLinesRenderGeometry_ && joinData_.halfedgesData_.length()) {
        debugLinesRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotWRGBA, IndexFormat::UInt16);

        core::FloatArray lineInstData;
        lineInstData.extend({0.f, 0.f, 1.f, 1.5f, 0.64f, 0.02f, 1.0f, 1.f});

        float lineLength = 100.f;

        geometry::Vec4fArray lineVertices;
        core::Array<UInt16> lineIndices;

        for (const VacJoinHalfedgeFrameData& s : joinData_.halfedgesData_) {
            geometry::Vec2f p(position_);
            double angle0 = s.angle();
            double angle1 = s.angle() + s.angleToNext();
            double midAngle = angle0 + angle1;
            if (angle0 > angle1) {
                midAngle += core::pi * 2;
            }
            midAngle *= 0.5;
            if (midAngle > core::pi * 2) {
                midAngle -= core::pi * 2;
            }
            geometry::Vec2f d(
                static_cast<float>(std::cos(midAngle)),
                static_cast<float>(std::sin(midAngle)));
            geometry::Vec2f n = d.orthogonalized();
            d *= lineLength;
            // clang-format off
            Int i = lineVertices.length();
            lineVertices.emplaceLast(p.x(), p.y(), -n.x()        , -n.y()        );
            lineVertices.emplaceLast(p.x(), p.y(),  n.x()        ,  n.y()        );
            lineVertices.emplaceLast(p.x(), p.y(), -n.x() + d.x(), -n.y() + d.y());
            lineVertices.emplaceLast(p.x(), p.y(),  n.x() + d.x(),  n.y() + d.y());
            lineIndices.extend(
                {static_cast<UInt16>(i),
                static_cast<UInt16>(i + 1),
                static_cast<UInt16>(i + 2),
                static_cast<UInt16>(i + 3),
                static_cast<UInt16>(-1)});
            // clang-format on
        }

        engine->updateBufferData(
            debugLinesRenderGeometry_->indexBuffer(), //
            std::move(lineIndices));
        engine->updateBufferData(
            debugLinesRenderGeometry_->vertexBuffer(0), std::move(lineVertices));
        engine->updateBufferData(
            debugLinesRenderGeometry_->vertexBuffer(1), std::move(lineInstData));
    }

    if (debugLinesRenderGeometry_) {
        engine->setProgram(graphics::BuiltinProgram::SreenSpaceDisplacement);
        engine->draw(debugLinesRenderGeometry_);
    }

    if (debugQuadRenderGeometry_ && joinData_.halfedgesData_.length() == 1) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(debugQuadRenderGeometry_);
    }
}

} // namespace detail

geometry::Rect2d VacKeyVertex::boundingBox(core::AnimTime /*t*/) const {
    vacomplex::KeyVertex* kv = vacKeyVertexNode();
    if (kv) {
        geometry::Vec2d pos = vacKeyVertexNode()->position();
        return geometry::Rect2d(pos, pos);
    }
    return geometry::Rect2d::empty;
}

bool VacKeyVertex::isSelectableAt(
    const geometry::Vec2d& p,
    bool /*outlineOnly*/,
    double tol,
    double* outDistance,
    core::AnimTime /*t*/) const {

    vacomplex::KeyVertex* kv = vacKeyVertexNode();
    if (kv) {
        geometry::Vec2d pos = vacKeyVertexNode()->position();
        double d = (p - pos).length();
        if (d < tol) {
            if (outDistance) {
                *outDistance = d;
            }
            return true;
        }
    }
    return false;
}

void VacVertexCell::computeJoin(core::AnimTime t) {
    detail::VacVertexCellFrameData* c = frameData(t);
    if (c) {
        computePosition(*c);
        computeJoin(*c);
    }
}

void VacVertexCell::paint_(graphics::Engine* engine, core::AnimTime t, PaintOptions flags)
    const {

    if (flags.has(PaintOption::Outline)) {
        detail::VacVertexCellFrameData* fd = frameData(t);
        if (fd) {
            //const_cast<VacVertexCell*>(this)->computePosition_(*fd);
            const_cast<VacVertexCell*>(this)->computeJoin(*fd);
            fd->debugPaint(engine);
        }
    }
}

detail::VacVertexCellFrameData* VacVertexCell::frameData(core::AnimTime t) const {
    vacomplex::VertexCell* cell = vacVertexCellNode();
    if (!cell) {
        return nullptr;
    }
    auto it = frameDataEntries_.begin();
    for (; it != frameDataEntries_.end(); ++it) {
        if (it->time() == t) {
            return &*it;
        }
    }
    if (cell->existsAt(t)) {
        it = frameDataEntries_.emplace(it, t);
        return &*it;
    }
    return nullptr;
}

void VacVertexCell::computePosition(detail::VacVertexCellFrameData& data) {
    if (data.isPositionComputed_ || data.isComputing_) {
        return;
    }
    vacomplex::VertexCell* v = vacVertexCellNode();
    if (!v) {
        return;
    }

    data.isComputing_ = true;

    data.position_ = v->position(data.time());

    data.isPositionComputed_ = true;
    data.isComputing_ = false;
}

void VacVertexCell::computeJoin(detail::VacVertexCellFrameData& data) {
    if (data.isJoinComputed_ || data.isComputing_) {
        return;
    }

    computePosition(data);

    data.isComputing_ = true;

    detail::VacJoinFrameData& joinData = data.joinData_;
    geometry::Vec2d vertexPosition = data.position();

    // collect standalone edge data and halfwidths at join
    for (const detail::VacJoinHalfedge& he : joinHalfedges_) {
        VacEdgeCell* cell = he.edgeCell();
        auto edgeData = const_cast<VacEdgeCellFrameData*>(
            cell->computeStandaloneGeometryAt(data.time()));
        if (!edgeData) {
            // huh ?
            continue;
        }

        const geometry::CurveSampleArray& samples = edgeData->samples_;
        if (samples.length() < 2) {
            continue;
        }

        detail::VacJoinHalfedgeFrameData& heData =
            joinData.halfedgesData_.emplaceLast(he);
        heData.edgeData_ = edgeData;

        const bool isReverse = he.isReverse();
        if (!isReverse) {
            geometry::CurveSample sample = samples.first();
            heData.halfwidths_ = sample.halfwidths();
        }
        else {
            geometry::CurveSample sample = samples.last();
            heData.halfwidths_[0] = sample.halfwidth(1);
            heData.halfwidths_[1] = sample.halfwidth(0);
        }
    }

    // compute patch limits (future: hook for plugins)
    for (detail::VacJoinHalfedgeFrameData& heData : joinData.halfedgesData_) {
        // We tested experimentally a per-slice equation, but it prevents us from
        // defining the angular order based on the result.
        // To keep more freedom we limit the input to halfwidths per edge.
        //
        // Let's keep it simple for now and use a single coefficient and multiply
        // each halfwidth to obtain the length limit.
        //
        constexpr double cutLimitCoefficient = 4.0;
        heData.patchCutLimits_ = cutLimitCoefficient * heData.halfwidths_;
    }

    // we define the interpolation length based on the cut limit
    //constexpr double interpolationLimitCoefficient = 1.5;

    const Int numHalfedges = joinData.halfedgesData_.length();
    if (numHalfedges == 0) {
        // nothing to do
    }
    else if (numHalfedges == 1) {
        // cap, todo
        //const detail::VacJoinHalfedgeFrameData& halfedgeData = data.halfedgesData_[0];
        //Int basePatchIndex = halfedgeData.halfedge().isReverse() ? 2 : 0;
        //VacEdgeCellFrameData* edgeData = halfedgeData.edgeData_;
        //if (edgeData) {
        //    Int numSamples = edgeData->samples_.length();
        //    edgeData->patches_[basePatchIndex].sampleOverride_ = numSamples / 3;
        //    edgeData->patches_[basePatchIndex + 1].sampleOverride_ = 0;
        //}
    }
    else {
        // sort by incident angle
        std::sort(
            data.joinData_.halfedgesData_.begin(),
            data.joinData_.halfedgesData_.end(),
            [](const detail::VacJoinHalfedgeFrameData& a,
               const detail::VacJoinHalfedgeFrameData& b) {
                return a.angle() < b.angle();
            });

        detail::VacJoinHalfedgeFrameData* halfedgeDataA =
            &data.joinData_.halfedgesData_.last();
        detail::VacJoinHalfedgeFrameData* halfedgeDataB =
            &data.joinData_.halfedgesData_.first();
        double angleA = halfedgeDataA->angle() - core::pi * 2;
        double angleB = 0;
        for (Int i = 0; i < data.joinData_.halfedgesData_.length();
             ++i, halfedgeDataA = halfedgeDataB++, angleA = angleB) {

            angleB = halfedgeDataB->angle();
            halfedgeDataA->angleToNext_ = angleB - angleA;
        }
    }

    data.isJoinComputed_ = true;
    data.isComputing_ = false;
}

ElementStatus VacKeyVertex::updateFromDom_(Workspace* /*workspace*/) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    vacomplex::KeyVertex* kv = vacKeyVertexNode();
    if (!kv) {
        VacElement* parentElement = parentVacElement();
        if (!parentElement) {
            return ElementStatus::ErrorInParent;
        }
        vacomplex::Node* parentNode = parentElement->vacNode();
        if (!parentNode) {
            return ElementStatus::ErrorInParent;
        }
        kv = topology::ops::createKeyVertex(
            domElement->internalId(), parentNode->toGroupUnchecked());
        setVacNode(kv);
    }

    const auto& position = domElement->getAttribute(ds::position).getVec2d();
    if (kv->position() != position) {
        topology::ops::setKeyVertexPosition(kv, position);
        onInputGeometryChanged();
    }

    return ElementStatus::Ok;
}

void VacVertexCell::rebuildJoinHalfedgesArray() const {

    vacomplex::VertexCell* v = vacVertexCellNode();
    if (!v) {
        joinHalfedges_.clear();
        return;
    }

    joinHalfedges_.clear();
    for (vacomplex::Cell* vacCell : v->star()) {
        vacomplex::EdgeCell* vacEdge = vacCell->toEdgeCell();
        VacElement* edge = static_cast<VacElement*>(workspace()->find(vacEdge->id()));
        // XXX replace dynamic_cast
        VacEdgeCell* edgeCell = dynamic_cast<VacKeyEdge*>(edge);
        if (edgeCell) {
            if (vacEdge->isStartVertex(v)) {
                joinHalfedges_.emplaceLast(edgeCell, 0, false);
            }
            if (vacEdge->isEndVertex(v)) {
                joinHalfedges_.emplaceLast(edgeCell, 0, true);
            }
        }
    }
}

void VacVertexCell::clearJoinHalfedgesJoinData() const {
    for (const detail::VacJoinHalfedge& joinHalfedge : joinHalfedges_) {
        if (joinHalfedge.isReverse()) {
            joinHalfedge.edgeCell()->clearEndJoinData();
        }
        else {
            joinHalfedge.edgeCell()->clearStartJoinData();
        }
    }
}

// this vertex is the halfedge start vertex
void VacVertexCell::addJoinHalfedge_(const detail::VacJoinHalfedge& joinHalfedge) {
    joinHalfedges_.emplaceLast(joinHalfedge);
    for (auto& entry : frameDataEntries_) {
        entry.clearJoinData();
    }
}

// this vertex is the halfedge start vertex
void VacVertexCell::removeJoinHalfedge_(const detail::VacJoinHalfedge& joinHalfedge) {
    auto equal = detail::VacJoinHalfedge::GrouplessEqualTo();
    joinHalfedges_.removeOneIf(
        [&](const detail::VacJoinHalfedge& x) { return equal(joinHalfedge, x); });
    for (auto& entry : frameDataEntries_) {
        entry.clearJoinData();
    }
}

void VacVertexCell::onInputGeometryChanged() {
    frameDataEntries_.clear();
    clearJoinHalfedgesJoinData();
}

void VacVertexCell::onJoinEdgeGeometryChanged_(VacEdgeCell* /*edge*/) {
    for (auto& entry : frameDataEntries_) {
        entry.clearJoinData();
    }
    clearJoinHalfedgesJoinData();
}

geometry::Rect2d VacInbetweenVertex::boundingBox(core::AnimTime t) const {
    vacomplex::InbetweenVertex* iv = vacInbetweenVertexNode();
    if (iv) {
        geometry::Vec2d pos = iv->position(t);
        return geometry::Rect2d(pos, pos);
    }
    return geometry::Rect2d();
}

ElementStatus VacInbetweenVertex::updateFromDom_(Workspace* /*workspace*/) {
    return ElementStatus::Ok;
}

void VacInbetweenVertex::preparePaint_(core::AnimTime /*t*/, PaintOptions /*flags*/) {
}

} // namespace vgc::workspace
