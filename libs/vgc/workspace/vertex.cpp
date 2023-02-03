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

void VacVertexCellFrameData::computeJoins() {
    computeJoins_();
}

void VacVertexCellFrameData::computeJoins_() {

    if (!areJoinsDirty_ || isComputingJoins_) {
        return;
    }
    isComputingJoins_ = true;

    using detail::VacJoinEdgeFrameData;

    frameData.debugQuadGeometry_.reset();
    frameData.debugLinesGeometry_.reset();
    edges_.clear();
    slices_.clear();

    // get the KeyEdges that we have to join (later we'll have to do that by join group index)
    for (topology::vacomplex::Cell* cell : kv->star()) {
        topology::KeyEdge* vacKe = cell->toKeyEdge();
        // XXX replace dynamic_cast
        KeyEdge* ke = dynamic_cast<KeyEdge*>(workspace()->find(vacKe->id()));
        if (ke) {
            EdgeGeometryDataCache* geometry = ke->computeRawGeometry(kv->time());
            bool isReverse = (vacKe->startVertex() != kv);
            const geometry::CurveSampleArray& samples = geometry->samples_;
            if (samples.length() < 2) {
                continue;
            }
            geometry::CurveSample sample = samples[isReverse ? samples.size() - 2 : 1];

            // XXX add xAxisAngle to Vec class
            geometry::Vec2d outgoingTangent = (sample.position() - pos_).normalized();
            double angle = geometry::Vec2d(1.f, 0).angle(outgoingTangent);
            if (angle < 0) {
                angle += core::pi * 2;
            }

            edges_.emplaceLast(JoinEdge{
                ke,
                vacKe,
                geometry,
                outgoingTangent,
                angle,
                isReverse,
                samples.length()});
        }
    }

    const Int numEdges = edges_.length();
    if (numEdges == 0) {
        // nothing to do
    }
    else if (numEdges == 1) {
        // cap, todo
    }
    else {
        // sort by incident angle
        std::sort(edges_.begin(), edges_.end(), [](const JoinEdge& a, const JoinEdge& b) {
            return a.angle < b.angle;
        });
        slices_.resizeNoInit(edges_.length());
        JoinEdge* previousEdge = &edges_.last();
        double previousAngle = previousEdge->angle - core::pi;
        auto sliceIt = slices_.begin();
        Int i = 0;
        for (JoinEdge& edgeRef : edges_) {
            JoinEdge* edge = &edgeRef;
            sliceIt->edges = {previousEdge, edge};
            sliceIt->angle = edge->angle - previousAngle;
            previousEdge = edge;
            previousAngle = edge->angle;
            //VGC_DEBUG_TMP("slice {} angle: {}", i, sliceIt->angle);
            ++i;
            ++sliceIt;
        }
    }

    isComputingJoins_ = false;
}

} // namespace detail

void VacVertexCell::debugPaint(
    graphics::Engine* engine,
    const detail::VacVertexCellFrameData& frameData) {

    using namespace graphics;
    using detail::VacJoinEdgeFrameData;

    if (!frameData.debugQuadRenderGeometry_) {
        frameData.debugQuadRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYRGB, IndexFormat::UInt16);
        geometry::Vec2f p(frameData.pos_);
        core::FloatArray vertices({
            p.x() - 5, p.y() - 5, 0, 0, 0, //
            p.x() - 5, p.y() + 5, 0, 0, 0, //
            p.x() + 5, p.y() - 5, 0, 0, 0, //
            p.x() + 5, p.y() + 5, 0, 0, 0, //
        });
        engine->updateVertexBufferData(
            frameData.debugQuadRenderGeometry_, std::move(vertices));
        core::Array<UInt16> lineIndices({0, 1, 2, 3});
        engine->updateBufferData(
            frameData.debugQuadRenderGeometry_->indexBuffer(), std::move(lineIndices));
    }

    if (!frameData.debugLinesRenderGeometry_ && frameData.edges_.length()) {
        frameData.debugLinesRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA, IndexFormat::UInt16);

        core::FloatArray lineInstData;
        lineInstData.extend({0.f, 0.f, 1.f, 0.64f, 0.02f, 1.0f, 1.f, 0.f /*padding*/});

        float lineHalfWidth = 1.5f;
        float lineLength = 100.f;

        geometry::Vec4fArray lineVertices;
        core::Array<UInt16> lineIndices;

        for (const VacJoinEdgeFrameData& s : frameData.edges_) {
            geometry::Vec2f p(frameData.pos_);
            double angle0 = s.angle_;
            double angle1 = s.angle_ + s.angleToNext_;
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
            frameData.debugLinesRenderGeometry_->indexBuffer(), //
            std::move(lineIndices));
        engine->updateBufferData(
            frameData.debugLinesRenderGeometry_->vertexBuffer(0),
            std::move(lineVertices));
        engine->updateBufferData(
            frameData.debugLinesRenderGeometry_->vertexBuffer(1),
            std::move(lineInstData));
    }

    if (frameData.debugLinesRenderGeometry_) {
        engine->setProgram(graphics::BuiltinProgram::SreenSpaceDisplacement);
        engine->draw(frameData.debugLinesRenderGeometry_);
    }

    if (frameData.debugQuadRenderGeometry_ && frameData.edges_.length() == 1) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(frameData.debugQuadRenderGeometry_);
    }
}

geometry::Rect2d VacKeyVertex::boundingBox(core::AnimTime /*t*/) const {
    topology::KeyVertex* kv = vacKeyVertexNode();
    if (kv) {
        geometry::Vec2d pos = vacKeyVertexNode()->position({});
        return geometry::Rect2d(pos, pos);
    }
}

detail::VacVertexCellFrameData* VacKeyVertex::getOrCreateFrameData(core::AnimTime t) {
    topology::KeyVertex* kv = vacKeyVertexNode();
    if (kv && kv->time() == t) {
        frameData_.pos_ = kv->position({});
        return &frameData_;
    }
    return nullptr;
}

ElementStatus VacKeyVertex::updateFromDom_(Workspace* /*workspace*/) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    topology::KeyVertex* kv = nullptr;
    if (!vacNode_) {
        kv = topology::ops::createKeyVertex(
            domElement->internalId(), parentVacElement()->vacNode()->toGroupUnchecked());
        vacNode_ = kv;
    }
    else {
        kv = vacNode_->toCellUnchecked()->toKeyVertexUnchecked();
    }

    const auto& position = domElement->getAttribute(ds::position).getVec2d();
    topology::ops::setKeyVertexPosition(kv, position);

    // XXX also reset frame data !
    notifyChanges();

    return ElementStatus::Ok;
}

void VacKeyVertex::paint_(
    graphics::Engine* engine,
    core::AnimTime /*t*/,
    PaintOptions flags) const {

    if (flags.has(PaintOption::Outline)) {
        debugPaint_(engine);
    }
}

void VacKeyVertex::debugPaint_(graphics::Engine* engine) const {
    debugPaint(engine, frameData_);
}

geometry::Rect2d VacInbetweenVertex::boundingBox(core::AnimTime t) const {
    topology::vacomplex::InbetweenVertex* iv = vacInbetweenVertexNode();
    if (iv) {
        geometry::Vec2d pos = iv->position(t);
        return geometry::Rect2d(pos, pos);
    }
    return geometry::Rect2d();
}

detail::VacVertexCellFrameData*
VacInbetweenVertex::getOrCreateFrameData(core::AnimTime t) {
    // todo
    return nullptr;
}

ElementStatus VacInbetweenVertex::updateFromDom_(Workspace* /*workspace*/) {
    return ElementStatus::Ok;
}

void VacInbetweenVertex::preparePaint_(core::AnimTime /*t*/, PaintOptions /*flags*/) {
}

void VacInbetweenVertex::paint_(
    graphics::Engine* /*engine*/,
    core::AnimTime /*t*/,
    PaintOptions /*flags*/) const {
}

} // namespace vgc::workspace
