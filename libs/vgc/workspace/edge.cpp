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

#include <vgc/workspace/edge.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

geometry::Rect2d KeyEdge::boundingBox(core::AnimTime /*t*/) const {
    return geometry::Rect2d::empty;
}

ElementError KeyEdge::updateFromDom_(Workspace* workspace) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    // dirty geometry
    edgeTesselationMode_ = -1;

    Element* ve0 =
        workspace->getElementFromPathAttribute(domElement, ds::startvertex, ds::vertex);
    Element* ve1 =
        workspace->getElementFromPathAttribute(domElement, ds::endvertex, ds::vertex);
    Element* parentElement = this->parent();

    // fallback to closed edge if only ev0 or ev1 is present
    if (!ve0) {
        ve0 = ve1;
    }
    if (!ve1) {
        ve1 = ve0;
    }

    topology::KeyVertex* kv0 = nullptr;
    topology::KeyVertex* kv1 = nullptr;
    topology::VacGroup* parentGroup = nullptr;

    // update dependencies (vertices)
    if (ve0) {
        workspace->updateElementFromDom(ve0);
        topology::VacNode* vn0 = ve0->vacNode();
        if (vn0) {
            kv0 = vn0->toCellUnchecked()->toKeyVertexUnchecked();
        }
    }
    if (ve1) {
        workspace->updateElementFromDom(ve1);
        topology::VacNode* vn1 = ve1->vacNode();
        if (vn1) {
            kv1 = vn1->toCellUnchecked()->toKeyVertexUnchecked();
        }
    }
    if (parentElement) {
        workspace->updateElementFromDom(parentElement);
        topology::VacNode* parentNode = parentElement->vacNode();
        if (parentNode) {
            // checked cast to group, could be something invalid
            parentGroup = parentNode->toGroup();
        }
    }

    topology::KeyEdge* ke = nullptr;

    // check if it needs to be rebuilt
    if (vacNode_) {
        ke = vacNode_->toCellUnchecked()->toKeyEdgeUnchecked();

        topology::KeyVertex* oldKv0 = ke->startVertex();
        topology::KeyVertex* oldKv1 = ke->endVertex();
        if (!parentGroup || kv0 != oldKv0 || kv1 != oldKv1) {
            topology::ops::removeNode(vacNode_, false);
            vacNode_ = nullptr;
            ke = nullptr;
        }
    }

    if (!vacNode_ && parentGroup) {
        if (kv0 && kv1) {
            ke = topology::ops::createKeyEdge(
                domElement->internalId(), parentGroup, kv0, kv1);
        }
        else {
            // XXX warning if kv0 || kv1 ?
            ke =
                topology::ops::createKeyClosedEdge(domElement->internalId(), parentGroup);
        }
        vacNode_ = ke;
    }
    // XXX warning on null parent group ?

    if (ke) {
        const auto& points = domElement->getAttribute(ds::positions).getVec2dArray();
        const auto& widths = domElement->getAttribute(ds::widths).getDoubleArray();

        // these do nothing if the data didn't change
        topology::ops::setKeyEdgeCurvePoints(ke, points);
        topology::ops::setKeyEdgeCurveWidths(ke, widths);

        // XXX should we snap here ?
        //     group view matrices may not be ready..
        //     maybe we could add two init functions to workspace::Element
        //     one for intrinsic data, one for dependent data.

        return ElementError::None;
    }

    return ElementError::InvalidAttribute;
}

void KeyEdge::preparePaint_(core::AnimTime /*t*/, PaintOptions /*flags*/) {
    // todo, use paint options to not compute everything or with lower quality
    updateGeometry_();
}

void KeyEdge::paint_(graphics::Engine* engine, core::AnimTime /*t*/, PaintOptions flags)
    const {

    // if not already done (should we leave preparePaint_ optional?)
    const_cast<KeyEdge*>(this)->updateGeometry_();

    using namespace graphics;
    namespace ds = dom::strings;

    const dom::Element* const domElement = this->domElement();

    constexpr PaintOptions strokeOptions = {PaintOption::Selected, PaintOption::Draft};

    if (flags.hasAny(strokeOptions)
        || !flags.has(PaintOption::Outline) && !cachedGraphics_.strokeGeometry_) {
        cachedGraphics_.strokeGeometry_ =
            engine->createDynamicTriangleStripView(BuiltinGeometryLayout::XY_iRGBA);

        GeometryViewCreateInfo createInfo = {};
        createInfo.setBuiltinGeometryLayout(BuiltinGeometryLayout::XY_iRGBA);
        createInfo.setPrimitiveType(PrimitiveType::TriangleStrip);
        createInfo.setVertexBuffer(0, cachedGraphics_.strokeGeometry_->vertexBuffer(0));
        BufferPtr selectionInstanceBuffer = engine->createVertexBuffer(4 * 4);
        createInfo.setVertexBuffer(1, selectionInstanceBuffer);
        cachedGraphics_.selectionGeometry_ = engine->createGeometryView(createInfo);

        core::Color color = domElement->getAttribute(ds::color).getColor();

        engine->updateBufferData(
            cachedGraphics_.strokeGeometry_->vertexBuffer(0), //
            strokeVertices_);
        engine->updateBufferData(
            cachedGraphics_.strokeGeometry_->vertexBuffer(1), //
            core::Array<float>({color.r(), color.g(), color.b(), color.a()}));
        engine->updateBufferData(
            selectionInstanceBuffer, //
            core::Array<float>(
                {1.0f - color.r(), 1.0f - color.g(), 1.0f - color.b(), 1.0f}));
    }

    constexpr PaintOptions centerlineOptions = {PaintOption::Outline};

    if (flags.hasAny(centerlineOptions) && !cachedGraphics_.centerlineGeometry_) {
        cachedGraphics_.centerlineGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA);

        core::FloatArray lineInstData;
        lineInstData.extend({0.f, 0.f, 1.f, 0.02f, 0.64f, 1.0f, 1.f});

        engine->updateBufferData(
            cachedGraphics_.centerlineGeometry_->vertexBuffer(0), lineVertices_);
        engine->updateBufferData(
            cachedGraphics_.centerlineGeometry_->vertexBuffer(1),
            std::move(lineInstData));
    }

    constexpr PaintOptions pointsOptions = {PaintOption::Outline};

    if (flags.hasAny(pointsOptions) && !cachedGraphics_.pointsGeometry_) {
        cachedGraphics_.pointsGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA);

        float pointHalfSize = 5.f;
        float lineHalfSize = 2.f;

        core::Array<geometry::Vec4f> pointVertices;
        // clang-format off
        pointVertices.extend({
            {0, 0, -pointHalfSize, -pointHalfSize},
            {0, 0, -pointHalfSize,  pointHalfSize},
            {0, 0,  pointHalfSize, -pointHalfSize},
            {0, 0,  pointHalfSize,  pointHalfSize} });
        // clang-format on

        engine->updateBufferData(
            cachedGraphics_.pointsGeometry_->vertexBuffer(0), std::move(pointVertices));
        engine->updateBufferData(
            cachedGraphics_.pointsGeometry_->vertexBuffer(1), pointInstData_);
    }

    if (flags.has(PaintOption::Selected)) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(cachedGraphics_.selectionGeometry_, -1, 0);
    }
    else if (!flags.has(PaintOption::Outline)) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(cachedGraphics_.strokeGeometry_, -1, 0);
    }

    if (flags.has(PaintOption::Outline)) {
        engine->setProgram(graphics::BuiltinProgram::SreenSpaceDisplacement);
        engine->draw(cachedGraphics_.centerlineGeometry_, -1, 0);
        engine->draw(cachedGraphics_.pointsGeometry_, -1, pointInstData_.length());
    }
}

void KeyEdge::updateGeometry_() {

    topology::KeyEdge* ke = vacKeyEdge();
    if (!ke) {
        // error ?
        return;
    }

    // check if we need an update
    if (edgeTesselationModeRequested_ == edgeTesselationMode_) {
        return;
    }
    edgeTesselationMode_ = edgeTesselationModeRequested_;

    strokeVertices_.clear();
    pointInstData_.clear();
    lineVertices_.clear();

    geometry::Curve curve;
    curve.setPositionData(&ke->points());
    curve.setWidthData(&ke->widths());

    float pointHalfSize = 5.f;
    float lineHalfSize = 2.f;

    double maxAngle = 0.05;
    int minQuads = 1;
    int maxQuads = 64;
    if (edgeTesselationModeRequested_ <= 2) {
        if (edgeTesselationModeRequested_ == 0) {
            maxQuads = 1;
        }
        else if (edgeTesselationModeRequested_ == 1) {
            minQuads = 10;
            maxQuads = 10;
        }
        geometry::CurveSamplingParameters samplingParams = {};
        samplingParams.setMaxAngle(maxAngle * 0.5); // matches triangulate()
        samplingParams.setMinIntraSegmentSamples(minQuads - 1);
        samplingParams.setMaxIntraSegmentSamples(maxQuads - 1);
        core::Array<geometry::CurveSample> samples;
        curve.sampleRange(samplingParams, samples);
        for (const geometry::CurveSample& s : samples) {
            geometry::Vec2d p0 = s.leftPoint();
            strokeVertices_.emplaceLast(geometry::Vec2f(p0));
            geometry::Vec2d p1 = s.rightPoint();
            strokeVertices_.emplaceLast(geometry::Vec2f(p1));
            geometry::Vec2f p = geometry::Vec2f(s.position());
            geometry::Vec2f n = geometry::Vec2f(s.normal());
            // clang-format off
            lineVertices_.emplaceLast(p.x(), p.y(), -lineHalfSize * n.x(), -lineHalfSize * n.y());
            lineVertices_.emplaceLast(p.x(), p.y(),  lineHalfSize * n.x(),  lineHalfSize * n.y());
            // clang-format on
        }
    }
    else {
        geometry::Vec2dArray triangles = curve.triangulate(maxAngle, 1, 64);
        for (const geometry::Vec2d& p : triangles) {
            strokeVertices_.emplaceLast(geometry::Vec2f(p));
        }
    }

    const geometry::Vec2dArray& d = *curve.positionData();
    Int numPoints = d.length();
    const float dl = 1.f / numPoints;
    for (Int j = 0; j < numPoints; ++j) {
        geometry::Vec2d dp = d[j];
        float l = j * dl;
        pointInstData_.extend(
            {static_cast<float>(dp.x()),
             static_cast<float>(dp.y()),
             0.f,
             (l > 0.5f ? 2 * (1.f - l) : 1.f),
             0.f,
             (l < 0.5f ? 2 * l : 1.f),
             1.f});
    }

    cachedGraphics_.clear();
}

} // namespace vgc::workspace
