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

#include <vgc/core/span.h>
#include <vgc/workspace/edge.h>
#include <vgc/workspace/vertex.h>

#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

VacEdgeCellFrameCache* VacKeyEdge::computeStandaloneGeometry(core::AnimTime t) {
    topology::KeyEdge* ke = vacKeyEdgeNode();
    if (ke && t == ke->time()) {
        computeStandaloneGeometry_();
    }
    // XXX to fix...
    return nullptr;
}

VacEdgeCellFrameCache* VacKeyEdge::computeGeometry(core::AnimTime t) {
    topology::KeyEdge* ke = vacKeyEdgeNode();
    if (ke && t == ke->time()) {
        computeGeometry_();
    }
    // XXX to fix...
    return nullptr;
}

geometry::Rect2d VacKeyEdge::boundingBox(core::AnimTime /*t*/) const {
    return geometry::Rect2d::empty;
}

ElementStatus VacKeyEdge::updateFromDom_(Workspace* workspace) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();

    std::array<std::optional<Element*>, 2> verticesOpt = {
        workspace->getElementFromPathAttribute(domElement, ds::startvertex, ds::vertex),
        workspace->getElementFromPathAttribute(domElement, ds::endvertex, ds::vertex)};

    Element* parentElement = this->parent();
    // XXX impl custom safe cast

    std::array<VacKeyVertex*, 2> oldVertices = {
        vertices_[0].element, vertices_[1].element};
    std::array<VacKeyVertex*, 2> newVertices = {};
    std::array<bool, 2> verticesNeedJoinUpdate = {};

    for (Int i = 0; i < 2; ++i) {
        VacKeyVertex* v = dynamic_cast<VacKeyVertex*>(verticesOpt[i].value_or(nullptr));
        newVertices[i] = v;
        verticesNeedJoinUpdate[i] |= replaceDependency(vertices_[i].element, v);
        vertices_[i].element = v;
    }

    // What's the cleanest way to report/notify that this edge has actually changed ?
    // What are the different categories of changes that matter to dependents ?
    // For instance an edge wanna know if a vertex moves or has a new style (new join)

    if (verticesOpt[0].has_value() != verticesOpt[1].has_value()) {
        onUpdateError_();
        return ElementStatus::InvalidAttribute;
    }

    for (Int i = 0; i < 2; ++i) {
        if (verticesOpt[i].has_value() && !newVertices[i]) {
            onUpdateError_();
            return ElementStatus::UnresolvedDependency;
        }
    }

    std::array<vacomplex::KeyVertex*, 2> vacKvs = {};
    vacomplex::Group* parentGroup = nullptr;

    // update dependencies (vertices)
    for (Int i = 0; i < 2; ++i) {
        VacKeyVertex* v = newVertices[i];
        if (v) {
            workspace->updateElementFromDom(v);
            vacomplex::Node* node = v->vacNode();
            if (node) {
                vacKvs[i] = node->toCellUnchecked()->toKeyVertexUnchecked();
            }
            if (v->hasError() || !vacKvs[i]) {
                onUpdateError_();
                return ElementStatus::ErrorInDependency;
            }
        }
    }
    // update group
    if (parentElement) {
        workspace->updateElementFromDom(parentElement);
        vacomplex::Node* parentNode = parentElement->vacNode();
        if (parentNode) {
            // checked cast to group, could be something invalid
            parentGroup = parentNode->toGroup();
        }
    }
    if (!parentGroup) {
        onUpdateError_();
        return ElementStatus::ErrorInParent;
    }

    vacomplex::KeyEdge* ke = nullptr;

    // check if it needs to be rebuilt
    ke = vacNode()->toCellUnchecked()->toKeyEdgeUnchecked();
    if (ke) {
        vacomplex::KeyVertex* oldKv0 = ke->startVertex();
        vacomplex::KeyVertex* oldKv1 = ke->endVertex();
        if (vacKvs[0] != oldKv0 || vacKvs[0] != oldKv1) {
            // dirty geometry
            isStandaloneGeometryDirty_ = true;
            isGeometryDirty_ = true;
            geometry_.clear();
            ke = nullptr;
        }
    }

    if (!ke) {
        const core::Id id = domElement->internalId();
        if (vacKvs[0] && vacKvs[1]) {
            ke = topology::ops::createKeyOpenEdge(id, parentGroup, vacKvs[0], vacKvs[1]);
        }
        else {
            // XXX warning if kv0 || kv1 ?
            ke = topology::ops::createKeyClosedEdge(id, parentGroup);
        }
        resetVacNode(ke);
    }

    // XXX warning on null parent group ?

    if (ke) {
        const auto& points = domElement->getAttribute(ds::positions).getVec2dArray();
        const auto& widths = domElement->getAttribute(ds::widths).getDoubleArray();

        if (&ke->points() != &points.get() || &ke->widths() != &widths.get()) {
            topology::ops::setKeyEdgeCurvePoints(ke, points);
            topology::ops::setKeyEdgeCurveWidths(ke, widths);
            // dirty geometry
            cachedGraphics_.clear();
            isStandaloneGeometryDirty_ = true;
            isGeometryDirty_ = true;
            geometry_.clear();
            clearJoinsCaches();
        }

        // XXX should we snap here ?
        //     group view matrices may not be ready..
        //     maybe we could add two init functions to workspace::Element
        //     one for intrinsic data, one for dependent data.

        return ElementStatus::Ok;
    }

    onUpdateError_();
    return ElementStatus::InvalidAttribute;
}

void VacKeyEdge::preparePaint_(core::AnimTime t, PaintOptions /*flags*/) {
    // todo, use paint options to not compute everything or with lower quality
    topology::KeyEdge* ke = vacKeyEdgeNode();
    if (t == ke->time()) {
        computeGeometry_();
    }
}

void VacKeyEdge::paint_(graphics::Engine* engine, core::AnimTime t, PaintOptions flags)
    const {

    topology::KeyEdge* ke = vacKeyEdgeNode();
    if (t != ke->time()) {
        return;
    }

    // if not already done (should we leave preparePaint_ optional?)
    const_cast<VacKeyEdge*>(this)->computeGeometry_();

    using namespace graphics;
    namespace ds = dom::strings;

    const dom::Element* const domElement = this->domElement();
    // XXX "implicit" cells' domElement would be the composite ?

    constexpr PaintOptions strokeOptions = {PaintOption::Selected, PaintOption::Draft};

    // XXX todo: reuse geometry objects, create buffers separately (attributes waiting in EdgeGraphics).

    if (flags.hasAny(strokeOptions)
        || (!flags.has(PaintOption::Outline) && !cachedGraphics_.strokeGeometry_)) {
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

        geometry::Vec2fArray strokeVertices;
        if (edgeTesselationModeRequested_ <= 2) {

            auto samplesStart = geometry_.samples_.begin();
            core::Span<const geometry::CurveSample> coreSamples(
                samplesStart + geometry_.startSampleOverride_,
                samplesStart + geometry_.endSampleOverride_);

            for (const geometry::CurveSample& s : coreSamples) {
                geometry::Vec2d p0 = s.leftPoint();
                strokeVertices.emplaceLast(geometry::Vec2f(p0));
                geometry::Vec2d p1 = s.rightPoint();
                strokeVertices.emplaceLast(geometry::Vec2f(p1));
            }
        }
        else {
            for (const geometry::Vec2d& p : geometry_.triangulation_) {
                strokeVertices.emplaceLast(geometry::Vec2f(p));
            }
        }
        engine->updateBufferData(
            cachedGraphics_.strokeGeometry_->vertexBuffer(0), //
            std::move(strokeVertices));

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
        lineInstData.extend({0.f, 0.f, 1.f, 0.02f, 0.64f, 1.0f, 1.f, 0.f /*padding*/});

        float lineHalfSize = 2.f;

        geometry::Vec4fArray lineVertices;
        for (const geometry::CurveSample& s : geometry_.samples_) {
            geometry::Vec2f p = geometry::Vec2f(s.position());
            geometry::Vec2f n = geometry::Vec2f(s.normal());
            // clang-format off
            lineVertices.emplaceLast(p.x(), p.y(), -lineHalfSize * n.x(), -lineHalfSize * n.y());
            lineVertices.emplaceLast(p.x(), p.y(),  lineHalfSize * n.x(),  lineHalfSize * n.y());
            // clang-format on
        }

        engine->updateBufferData(
            cachedGraphics_.centerlineGeometry_->vertexBuffer(0),
            std::move(lineVertices));
        engine->updateBufferData(
            cachedGraphics_.centerlineGeometry_->vertexBuffer(1),
            std::move(lineInstData));
    }

    constexpr PaintOptions pointsOptions = {PaintOption::Outline};

    if (flags.hasAny(pointsOptions) && !cachedGraphics_.pointsGeometry_) {
        cachedGraphics_.pointsGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA);

        float pointHalfSize = 5.f;

        core::Array<geometry::Vec4f> pointVertices;
        // clang-format off
        pointVertices.extend({
            {0, 0, -pointHalfSize, -pointHalfSize},
            {0, 0, -pointHalfSize,  pointHalfSize},
            {0, 0,  pointHalfSize, -pointHalfSize},
            {0, 0,  pointHalfSize,  pointHalfSize} });
        // clang-format on

        core::FloatArray pointInstData;
        const Int numPoints = geometry_.cps_.length();
        const float dl = 1.f / numPoints;
        for (Int j = 0; j < numPoints; ++j) {
            geometry::Vec2f p = geometry_.cps_[j];
            float l = j * dl;
            pointInstData.extend(
                {p.x(),
                 p.y(),
                 0.f,
                 (l > 0.5f ? 2 * (1.f - l) : 1.f),
                 0.f,
                 (l < 0.5f ? 2 * l : 1.f),
                 1.f});
        }

        engine->updateBufferData(
            cachedGraphics_.pointsGeometry_->vertexBuffer(0), std::move(pointVertices));
        engine->updateBufferData(
            cachedGraphics_.pointsGeometry_->vertexBuffer(1), std::move(pointInstData));
    }

    if (flags.has(PaintOption::Selected)) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(cachedGraphics_.selectionGeometry_);
    }
    else if (!flags.has(PaintOption::Outline)) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(cachedGraphics_.strokeGeometry_);
    }

    if (flags.has(PaintOption::Outline)) {
        engine->setProgram(graphics::BuiltinProgram::SreenSpaceDisplacement);
        engine->draw(cachedGraphics_.centerlineGeometry_);
        engine->drawInstanced(cachedGraphics_.pointsGeometry_);
    }
}

void VacKeyEdge::onVacNodeRemoved_() {
    clearJoinsCaches();
}

void VacKeyEdge::onUpdateError_() {
    resetVacNode();
}

void VacKeyEdge::clearJoinsCaches() const {
    for (Int i = 0; i < 2; ++i) {
        VacKeyVertex* v = vertices_[i].element;
        if (v) {
            v->clearJoinCaches();
        }
    }
}

void VacKeyEdge::computeStandaloneGeometry_() {

    if (!isStandaloneGeometryDirty_ || isComputingGeometry_) {
        return;
    }
    isComputingGeometry_ = true;

    topology::KeyEdge* ke = vacKeyEdgeNode();
    if (!ke) {
        // error ?
        isComputingGeometry_ = false;
        return;
    }

    // check if we need an update
    if (edgeTesselationModeRequested_ == geometry_.edgeTesselationMode_) {
        isComputingGeometry_ = false;
        return;
    }

    geometry_.clear();
    geometry_.edgeTesselationMode_ = edgeTesselationModeRequested_;

    geometry::Curve curve;
    curve.setPositions(ke->points());
    curve.setWidths(ke->widths());

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
        curve.sampleRange(samplingParams, geometry_.samples_);
        geometry_.endSampleOverride_ = geometry_.samples_.length();
    }
    else {
        geometry_.triangulation_ = curve.triangulate(maxAngle, 1, 64);
    }
    geometry_.samplingVersion_++;

    for (const geometry::Vec2d& p : curve.positions()) {
        geometry_.cps_.emplaceLast(geometry::Vec2f(p));
    }

    cachedGraphics_.clear();
    isComputingGeometry_ = false;
}

void VacKeyEdge::computeGeometry_() {

    computeStandaloneGeometry_();
    if (!isGeometryDirty_ || isComputingGeometry_) {
        return;
    }
    isComputingGeometry_ = true;

    topology::KeyEdge* ke = vacKeyEdgeNode();

    // XXX shouldn't do it for draft -> add quality enum for current cached geometry
    VacKeyVertex* v0 = vertices_[0].element;
    if (v0) {
        v0->computeJoin(ke->time());
    }
    VacKeyVertex* v1 = vertices_[0].element;
    if (v1 && v1 != v0) {
        v1->computeJoin(ke->time());
    }

    cachedGraphics_.clear();
    isComputingGeometry_ = false;
}

void VacKeyEdge::computeJoin_(VacVertexCell* /*v*/, bool /*isStart*/) {

    using namespace topology;

    // XXX instead use the join struct as join builder...
}

} // namespace vgc::workspace
