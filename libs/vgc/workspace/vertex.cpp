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

void VacVertexCellFrameCache::debugPaint(graphics::Engine* engine) {

    using namespace graphics;
    using detail::VacJoinHalfedgeFrameCache;

    if (!debugQuadRenderGeometry_) {
        debugQuadRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYRGB, IndexFormat::UInt16);
        geometry::Vec2f p(pos_);
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

    if (!debugLinesRenderGeometry_ && halfedges_.length()) {
        debugLinesRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA, IndexFormat::UInt16);

        core::FloatArray lineInstData;
        lineInstData.extend({0.f, 0.f, 1.f, 0.64f, 0.02f, 1.0f, 1.f, 0.f /*padding*/});

        float lineHalfWidth = 1.5f;
        float lineLength = 100.f;

        geometry::Vec4fArray lineVertices;
        core::Array<UInt16> lineIndices;

        for (const VacJoinHalfedgeFrameCache& s : halfedges_) {
            geometry::Vec2f p(pos_);
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
            geometry::Vec2f n = d.orthogonalized() * lineHalfWidth;
            d *= lineLength;
            // clang-format off
            Int i = lineVertices.length();
            lineVertices.emplaceLast(p.x(), p.y(), -n.x(), -n.y());
            lineVertices.emplaceLast(p.x(), p.y(), n.x(), n.y());
            lineVertices.emplaceLast(p.x(), p.y(), -n.x() + d.x(), -n.y() + d.y());
            lineVertices.emplaceLast(p.x(), p.y(), n.x() + d.x(), n.y() + d.y());
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

    if (debugQuadRenderGeometry_ && halfedges_.length() == 1) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(debugQuadRenderGeometry_);
    }
}

} // namespace detail

geometry::Rect2d VacKeyVertex::boundingBox(core::AnimTime /*t*/) const {
    vacomplex::KeyVertex* kv = vacKeyVertexNode();
    if (kv) {
        geometry::Vec2d pos = vacKeyVertexNode()->position({});
        return geometry::Rect2d(pos, pos);
    }
    return geometry::Rect2d::empty;
}

const core::Array<detail::VacJoinHalfedge>& VacVertexCell::joinHalfedges() const {
    if (!isJoinHalfedgesDirty_) {
        joinHalfedges_.clear();
        computeJoinHalfedges_();
    }
    return joinHalfedges_;
}

void VacVertexCell::clearFrameCaches() const {
    frameCacheEntries_.clear();
}

void VacVertexCell::clearJoinCaches() const {
    for (auto& entry : frameCacheEntries_) {
        entry.clearJoinData();
    }
}

detail::VacVertexCellFrameCache* VacVertexCell::computeJoin(core::AnimTime t) const {
    detail::VacVertexCellFrameCache* c = frameCache(t);
    if (c) {
        computeJoin_(*c);
    }
    return c;
}

void VacVertexCell::computeJoinHalfedges_() const {
    // get the KeyEdges that we have to join (later we'll have to do that by join group index)
    vacomplex::VertexCell* v = vacVertexCellNode();
    if (!v) {
        return;
    }

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

void VacVertexCell::paint_(graphics::Engine* engine, core::AnimTime t, PaintOptions flags)
    const {

    if (flags.has(PaintOption::Outline)) {
        detail::VacVertexCellFrameCache* fd = frameCache(t);
        if (fd) {
            fd->debugPaint(engine);
        }
    }
}

detail::VacVertexCellFrameCache* VacVertexCell::frameCache(core::AnimTime t) const {
    vacomplex::VertexCell* v = vacVertexCellNode();
    if (!v) {
        return nullptr;
    }
    auto it = frameCacheEntries_.begin();
    for (; it != frameCacheEntries_.end(); ++it) {
        if (it->time() == t) {
            return &*it;
        }
    }
    if (v->existsAt(t)) {
        it = frameCacheEntries_.emplace(it, t);
        initFrameCache_(*it);
        return &*it;
    }
    return nullptr;
}

void VacVertexCell::initFrameCache_(detail::VacVertexCellFrameCache& cache) const {
    vacomplex::VertexCell* v = vacVertexCellNode();
    cache.pos_ = v->position(cache.time());
}

void VacVertexCell::computeJoinHalfedgesData_(
    detail::VacVertexCellFrameCache& cache) const {

    // XXX use dirty bool to know if it is initialized..
    if (cache.halfedges_.isEmpty()) {
        for (const detail::VacJoinHalfedge& he : joinHalfedges()) {
            cache.halfedges_.emplaceLast(he);
        }
    }

    for (detail::VacJoinHalfedgeFrameCache& heCache : cache.halfedges_) {
        VacEdgeCellFrameCache* edgeCache =
            heCache.halfedge().edgeCell()->computeStandaloneGeometry(cache.time());

        const geometry::CurveSampleArray& samples = edgeCache->samples_;
        if (samples.length() < 2) {
            continue;
        }
        geometry::CurveSample sample =
            samples[heCache.halfedge().isReverse() ? samples.size() - 2 : 1];

        // XXX add xAxisAngle to Vec class
        geometry::Vec2d outgoingTangent = (sample.position() - cache.pos()).normalized();
        double angle = geometry::Vec2d(1.f, 0).angle(outgoingTangent);
        if (angle < 0) {
            angle += core::pi * 2;
        }

        heCache.edgeCache_ = edgeCache;
        heCache.outgoingTangent_ = outgoingTangent;
        heCache.angle_ = angle;
    }
}

void VacVertexCell::computeJoin_(detail::VacVertexCellFrameCache& cache) const {
    if (cache.isJoinComputed_ || cache.isComputingJoin_) {
        return;
    }
    cache.isComputingJoin_ = true;

    const Int numHalfedges = cache.halfedges_.length();
    if (numHalfedges == 0) {
        // nothing to do
    }
    else if (numHalfedges == 1) {
        // cap, todo
    }
    else {
        // sort by incident angle
        std::sort(
            cache.halfedges_.begin(),
            cache.halfedges_.end(),
            [](const detail::VacJoinHalfedgeFrameCache& a,
               const detail::VacJoinHalfedgeFrameCache& b) {
                return a.angle() < b.angle();
            });

        detail::VacJoinHalfedgeFrameCache* previousHalfedge = &cache.halfedges_.last();
        double previousAngle = previousHalfedge->angle() - core::pi;
        for (detail::VacJoinHalfedgeFrameCache& halfedge : cache.halfedges_) {
            halfedge.angleToNext_ = halfedge.angle() - previousAngle;
            previousAngle = halfedge.angle();
        }
    }

    cache.isJoinComputed_ = true;
}

ElementStatus VacKeyVertex::updateFromDom_(Workspace* /*workspace*/) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    vacomplex::KeyVertex* kv = vacKeyVertexNode();
    if (!kv) {
        kv = topology::ops::createKeyVertex(
            domElement->internalId(), parentVacElement()->vacNode()->toGroupUnchecked());
        resetVacNode(kv);
    }

    const auto& position = domElement->getAttribute(ds::position).getVec2d();
    topology::ops::setKeyVertexPosition(kv, position);

    // XXX also reset frame data !
    notifyChanges();

    return ElementStatus::Ok;
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
