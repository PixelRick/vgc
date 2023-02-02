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

void KeyVertex::computeJoins(core::AnimTime t) {
    topology::KeyVertex* kv = vacKeyVertex();
    if (kv && t == kv->time()) {
        computeJoins_();
    }
}

geometry::Rect2d KeyVertex::boundingBox(core::AnimTime /*t*/) const {
    geometry::Vec2d pos = vacKeyVertex()->position({});
    return geometry::Rect2d(pos, pos);
}

ElementStatus KeyVertex::updateFromDom_(Workspace* /*workspace*/) {
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

    notifyChanges();

    return ElementStatus::Ok;
}

void KeyVertex::paint_(graphics::Engine* engine, core::AnimTime /*t*/, PaintOptions flags)
    const {

    if (flags.has(PaintOption::Outline)) {
        debugPaint_(engine);
    }
}

void KeyVertex::debugPaint_(graphics::Engine* engine) const {

    using namespace graphics;
    using detail::JoinSlice;

    if (!debugQuadGeometry_) {
        debugQuadGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYRGB, IndexFormat::UInt16);
        geometry::Vec2f p(pos_);
        core::FloatArray vertices({
            p.x() - 5, p.y() - 5, 0, 0, 0, //
            p.x() - 5, p.y() + 5, 0, 0, 0, //
            p.x() + 5, p.y() - 5, 0, 0, 0, //
            p.x() + 5, p.y() + 5, 0, 0, 0, //
        });
        engine->updateVertexBufferData(debugQuadGeometry_, std::move(vertices));
        core::Array<UInt16> lineIndices({0, 1, 2, 3});
        engine->updateBufferData(
            debugQuadGeometry_->indexBuffer(), std::move(lineIndices));
    }

    if (!debugLinesGeometry_ && slices_.length()) {
        debugLinesGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA, IndexFormat::UInt16);

        core::FloatArray lineInstData;
        lineInstData.extend({0.f, 0.f, 1.f, 0.64f, 0.02f, 1.0f, 1.f, 0.f /*padding*/});

        float lineHalfWidth = 1.5f;
        float lineLength = 100.f;

        geometry::Vec4fArray lineVertices;
        core::Array<UInt16> lineIndices;

        for (const JoinSlice& s : slices_) {
            geometry::Vec2f p(pos_);
            double angle0 = s.edges[0]->angle;
            double angle1 = s.edges[1]->angle;
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
            debugLinesGeometry_->indexBuffer(), std::move(lineIndices));
        engine->updateBufferData(
            debugLinesGeometry_->vertexBuffer(0), std::move(lineVertices));
        engine->updateBufferData(
            debugLinesGeometry_->vertexBuffer(1), std::move(lineInstData));
    }

    if (debugLinesGeometry_) {
        engine->setProgram(graphics::BuiltinProgram::SreenSpaceDisplacement);
        engine->draw(debugLinesGeometry_);
    }

    if (debugQuadGeometry_ && slices_.length() < 2) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(debugQuadGeometry_);
    }
}

void KeyVertex::computeJoins_() {

    if (!areJoinsDirty_ || isComputingJoins_) {
        return;
    }
    isComputingJoins_ = true;

    using detail::JoinEdge;
    using detail::JoinSlice;

    topology::KeyVertex* kv = vacNode()->toCellUnchecked()->toKeyVertexUnchecked();
    if (!kv) {
        isComputingJoins_ = false;
        return;
    }
    pos_ = kv->position({});

    debugQuadGeometry_.reset();
    debugLinesGeometry_.reset();
    edges_.clear();
    slices_.clear();

    // get the KeyEdges that we have to join (later we'll have to do that by join group index)
    for (topology::VacCell* cell : kv->star()) {
        topology::KeyEdge* vacKe = cell->toKeyEdge();
        // XXX replace dynamic_cast
        KeyEdge* ke = dynamic_cast<KeyEdge*>(workspace()->find(vacKe->id()));
        if (ke) {
            ke->computeRawGeometry(kv->time());
            bool isReverse = (vacKe->startVertex() != kv);
            const geometry::CurveSampleArray& samples = ke->geometry_.samples_;
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

            edges_.emplaceLast(
                JoinEdge{vacKe, ke, outgoingTangent, angle, isReverse, samples.length()});
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

geometry::Rect2d InbetweenVertex::boundingBox(core::AnimTime t) const {
    geometry::Vec2d pos = vacInbetweenVertex()->position(t);
    return geometry::Rect2d(pos, pos);
}

ElementStatus InbetweenVertex::updateFromDom_(Workspace* /*workspace*/) {
    return ElementStatus::Ok;
}

void InbetweenVertex::preparePaint_(core::AnimTime /*t*/, PaintOptions /*flags*/) {
}

void InbetweenVertex::paint_(
    graphics::Engine* /*engine*/,
    core::AnimTime /*t*/,
    PaintOptions /*flags*/) const {
}

} // namespace vgc::workspace
