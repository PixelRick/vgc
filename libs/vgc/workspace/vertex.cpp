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

    if (!debugLinesRenderGeometry_ && halfedgesData_.length()) {
        debugLinesRenderGeometry_ = engine->createDynamicTriangleStripView(
            BuiltinGeometryLayout::XYDxDy_iXYRotRGBA, IndexFormat::UInt16);

        core::FloatArray lineInstData;
        lineInstData.extend({0.f, 0.f, 1.f, 0.64f, 0.02f, 1.0f, 1.f, 0.f /*padding*/});

        float lineHalfWidth = 1.5f;
        float lineLength = 100.f;

        geometry::Vec4fArray lineVertices;
        core::Array<UInt16> lineIndices;

        for (const VacJoinHalfedgeFrameData& s : halfedgesData_) {
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

    if (debugQuadRenderGeometry_ && halfedgesData_.length() == 1) {
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

template<bool fromEnd>
Int findOverrideLimit(
    VacEdgeCellFrameData* edgeData,
    double halfwidthArcRatio,
    geometry::EdgeSide side) {

    Int index = 0;
    const geometry::CurveSampleArray& samples = edgeData->samples();
    if constexpr (!fromEnd) {
        for (auto it = samples.begin(); it != samples.end(); ++it, ++index) {
            const double hw = it->halfwidth(side);
            const double s = it->s();
            if (s * halfwidthArcRatio > hw) {
                break;
            }
        }
    }
    else {
        const double endS = samples.last().s();
        for (auto it = samples.rbegin(); it != samples.rend(); ++it, ++index) {
            const double hw = it->halfwidth(side);
            const double s = endS - it->s();
            if (s * halfwidthArcRatio > hw) {
                break;
            }
        }
    }
    return std::min(index, samples.length() / 3);
}

Int findOverrideLimit(
    VacEdgeCellFrameData* edgeData,
    double halfwidthArcRatio,
    geometry::EdgeSide side,
    bool fromEnd = false) {

    if (fromEnd) {
        return findOverrideLimit<true>(edgeData, halfwidthArcRatio, side);
    }
    else {
        return findOverrideLimit<false>(edgeData, halfwidthArcRatio, side);
    }
}

void VacVertexCell::computeJoin(detail::VacVertexCellFrameData& data) {
    if (data.isJoinComputed_ || data.isComputing_) {
        return;
    }

    computePosition(data);

    data.isComputing_ = true;

    for (const detail::VacJoinHalfedge& he : joinHalfedges_) {
        data.halfedgesData_.emplaceLast(he);
    }

    for (detail::VacJoinHalfedgeFrameData& heData : data.halfedgesData_) {

        VacEdgeCell* cell = heData.halfedge().edgeCell();
        VacEdgeCellFrameData* edgeData = cell->frameData(data.time());
        if (!edgeData) {
            // huh ?
            continue;
        }
        cell->computeStandaloneGeometry(*edgeData);

        const bool isReverse = heData.halfedge().isReverse();
        const geometry::CurveSampleArray& samples = edgeData->samples_;
        if (samples.length() < 2) {
            continue;
        }
        geometry::CurveSample sample = samples[isReverse ? samples.size() - 2 : 1];

        // XXX add xAxisAngle to Vec class
        geometry::Vec2d outgoingTangent =
            (sample.position() - data.position()).normalized();
        double angle = geometry::Vec2d(1.f, 0).angle(outgoingTangent);
        if (angle < 0) {
            angle += core::pi * 2;
        }

        heData.edgeData_ = edgeData;
        heData.outgoingTangent_ = outgoingTangent;
        heData.angle_ = angle;
    }

    const Int numHalfedges = data.halfedgesData_.length();
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
            data.halfedgesData_.begin(),
            data.halfedgesData_.end(),
            [](const detail::VacJoinHalfedgeFrameData& a,
               const detail::VacJoinHalfedgeFrameData& b) {
                return a.angle() < b.angle();
            });

        detail::VacJoinHalfedgeFrameData* halfedgeFirst = &data.halfedgesData_.first();
        detail::VacJoinHalfedgeFrameData* halfedgeA = &data.halfedgesData_.last();
        detail::VacJoinHalfedgeFrameData* halfedgeB = halfedgeFirst;
        double angleA = halfedgeA->angle() - core::pi * 2;
        for (Int i = 0; i < data.halfedgesData_.length(); ++i) {

            double angleB = halfedgeB->angle();
            halfedgeA->angleToNext_ = angleB - angleA;

            const bool isReverseA = halfedgeA->halfedge().isReverse();
            const bool isReverseB = halfedgeB->halfedge().isReverse();

            const geometry::CurveSampleArray& samplesA = halfedgeA->edgeData_->samples();
            const geometry::CurveSampleArray& samplesB = halfedgeB->edgeData_->samples();

            Int maxOverrideA = 0;
            Int maxOverrideB = 0;

            /*
            maxOverrideA = findOverrideLimit(
                halfedgeA->edgeData_,
                0.5,
                isReverseA ? geometry::EdgeSide::Left : geometry::EdgeSide::Right,
                isReverseA);
            maxOverrideB = findOverrideLimit(
                halfedgeB->edgeData_,
                0.5,
                isReverseB ? geometry::EdgeSide::Right : geometry::EdgeSide::Left,
                isReverseB);
            */

            core::Array<detail::EdgeJoinPatchSample> patchSamplesA;
            core::Array<detail::EdgeJoinPatchSample> patchSamplesB;

            double hwJoinA = 0;
            double hwJoinB = 0;

            if (!isReverseA) {
                hwJoinA = samplesA.first().rightHalfwidth();
            }
            else {
                hwJoinA = samplesA.last().leftHalfwidth();
            }

            if (!isReverseB) {
                hwJoinB = samplesB.first().leftHalfwidth();
            }
            else {
                hwJoinB = samplesB.last().rightHalfwidth();
            }

            //double hwJoinMin = (std::min)(hwJoinA, hwJoinB);
            double hwJoinMax = (std::max)(hwJoinA, hwJoinB);
            double patchLengthMax = hwJoinMax * 2.4137;
            double cutLengthMax = patchLengthMax * 0.812327;

            if (!isReverseA) {
                Int index = 0;
                for (auto it = samplesA.begin(); it != samplesA.end(); ++it, ++index) {
                    const double hw = it->rightHalfwidth();
                    const double s = it->s();
                    if (s > patchLengthMax) {
                        // we should interp a new sample
                        break;
                    }
                    auto& p = patchSamplesA.emplaceLast();
                    p.centerPointSampleIndex = index;
                    p.centerPoint = it->position();
                    p.sidePoint = it->rightPoint();
                    p.st = geometry::Vec2d(s, hw);
                }
            }
            else {
                const double endS = samplesA.last().s();
                Int index = samplesA.size() - 1;
                for (auto it = samplesA.rbegin(); it != samplesA.rend(); ++it, --index) {
                    const double hw = it->leftHalfwidth();
                    const double s = endS - it->s();
                    if (s > patchLengthMax) {
                        // we should interp a new sample
                        break;
                    }
                    auto& p = patchSamplesA.emplaceLast();
                    p.centerPointSampleIndex = index;
                    p.centerPoint = it->position();
                    p.sidePoint = it->leftPoint();
                    p.st = geometry::Vec2d(s, hw);
                }
            }

            if (!isReverseB) {
                Int index = 0;
                for (auto it = samplesB.begin(); it != samplesB.end(); ++it, ++index) {
                    const double hw = it->leftHalfwidth();
                    const double s = it->s();
                    if (s > patchLengthMax) {
                        // we should interp a new sample
                        break;
                    }
                    auto& p = patchSamplesB.emplaceLast();
                    p.centerPointSampleIndex = index;
                    p.centerPoint = it->position();
                    p.sidePoint = it->leftPoint();
                    p.st = geometry::Vec2d(s, hw);
                }
            }
            else {
                const double endS = samplesB.last().s();
                Int index = samplesB.size() - 1;
                for (auto it = samplesB.rbegin(); it != samplesB.rend(); ++it, --index) {
                    const double hw = it->rightHalfwidth();
                    const double s = endS - it->s();
                    if (s > patchLengthMax) {
                        // we should interp a new sample
                        break;
                    }
                    auto& p = patchSamplesB.emplaceLast();
                    p.centerPointSampleIndex = index;
                    p.centerPoint = it->position();
                    p.sidePoint = it->rightPoint();
                    p.st = geometry::Vec2d(s, hw);
                }
            }

            maxOverrideA = patchSamplesA.size() - 1;
            maxOverrideB = patchSamplesB.size() - 1;

            // compute intersection of straight outlines

            struct Ray {
                geometry::Vec2d pos;
                geometry::Vec2d dir;

                std::optional<geometry::Vec2d> intersectWith(const Ray& other) {

                    const geometry::Vec2d d1 = dir;
                    const geometry::Vec2d d2 = other.dir;

                    double ddet = d1.det(d2);
                    if (std::abs(ddet) > core::epsilon) {
                        geometry::Vec2d w = other.pos - pos;
                        double iddet = 1 / ddet;
                        double t0 = w.det(d2) * iddet;
                        double t1 = w.det(d1) * iddet;
                        return geometry::Vec2d(t0, t1);
                    }

                    return std::nullopt;
                }
            };

            Ray borderRayA = {
                patchSamplesA.first().sidePoint,
                halfedgeA->outgoingTangent_.normalized()};

            Ray borderRayB = {
                patchSamplesB.first().sidePoint,
                halfedgeB->outgoingTangent_.normalized()};

            std::optional<geometry::Vec2d> isect = borderRayA.intersectWith(borderRayB);
            if (isect.has_value()) {
                const geometry::Vec2d& ts = isect.value();
                geometry::Vec2d vA = borderRayA.pos + borderRayA.dir * ts[0];
                geometry::Vec2d vB = borderRayB.pos + borderRayB.dir * ts[1];
                double patchEndA = patchSamplesA.last().st[0];
                double patchEndB = patchSamplesB.last().st[0];
                geometry::Vec2d bisect = vA - data.position();
                geometry::Vec2d bisectDir = bisect;
                double bisectLen = bisectDir.length();
                bisectDir.normalize();
                // vA should equal vB

                geometry::Vec2d fallbackPointA = vA;
                geometry::Vec2d fallbackPointB = vB;

                const bool isExtA = ts[0] <= 0;
                const bool isExtB = ts[1] <= 0;

                if (isExtA && ts[1] > cutLengthMax && hwJoinA >= hwJoinB) {
                    // special case
                    double w = 1 - cutLengthMax / ts[1];

                    geometry::Vec2d pA =
                        borderRayA.pos + w * cutLengthMax * borderRayA.dir;
                    geometry::Vec2d pB =
                        borderRayB.pos + (1 - w) * cutLengthMax * borderRayB.dir;
                    geometry::Vec2d pC = w * pB + (1 - w) * pA;

                    patchSamplesA.emplaceFirst();
                    auto it = patchSamplesA.begin();
                    it->centerPoint = data.position();
                    it->sidePoint = pB;
                    ++it;
                    // slope
                    double wcutA = w * cutLengthMax;
                    geometry::Vec2d lastCenter = data.position();
                    for (; it != patchSamplesA.end() && (it->st[0] < wcutA); ++it) {
                        double ns = it->st[0] / wcutA;
                        it->sidePoint = ns * pA + (1 - ns) * pC;
                        lastCenter = it->centerPoint;
                    }
                    it = patchSamplesA.emplace(it);
                    it->centerPoint = lastCenter;
                    it->sidePoint = pA;
                    ++it;
                    // converge samples
                    for (; it != patchSamplesA.end(); ++it) {
                        double ns = (it->st[0] - wcutA) / (patchEndA - wcutA);
                        geometry::Vec2d ref = borderRayA.pos + it->st[0] * borderRayA.dir;
                        it->sidePoint = ns * it->sidePoint + (1 - ns) * ref;
                    }

                    double wcutB = (1 - w) * cutLengthMax;
                    it = patchSamplesB.begin();
                    lastCenter = data.position();
                    for (; it != patchSamplesB.end() && (it->st[0] < wcutB); ++it) {
                        double ns = it->st[0] / wcutB;
                        it->sidePoint = ns * pB + (1 - ns) * data.position();
                        lastCenter = it->centerPoint;
                    }
                    it = patchSamplesB.emplace(it);
                    it->centerPoint = lastCenter;
                    it->sidePoint = pB;
                    ++it;
                    // converge samples
                    for (; it != patchSamplesB.end(); ++it) {
                        double ns = (it->st[0] - wcutB) / (patchEndB - wcutB);
                        geometry::Vec2d ref = borderRayB.pos + it->st[0] * borderRayB.dir;
                        it->sidePoint = ns * it->sidePoint + (1 - ns) * ref;
                    }
                }
                else if (isExtB && ts[0] > cutLengthMax && hwJoinB >= hwJoinA) {
                    // special case
                    double w = 1 - cutLengthMax / ts[0];

                    geometry::Vec2d pB =
                        borderRayB.pos + w * cutLengthMax * borderRayB.dir;
                    geometry::Vec2d pA =
                        borderRayA.pos + (1 - w) * cutLengthMax * borderRayA.dir;
                    geometry::Vec2d pC = w * pA + (1 - w) * pB;

                    patchSamplesB.emplaceFirst();
                    auto it = patchSamplesB.begin();
                    it->centerPoint = data.position();
                    it->sidePoint = pA;
                    ++it;
                    // slope
                    double wcutB = w * cutLengthMax;
                    geometry::Vec2d lastCenter = data.position();
                    for (; it != patchSamplesB.end() && (it->st[0] < wcutB); ++it) {
                        double ns = it->st[0] / wcutB;
                        it->sidePoint = ns * pB + (1 - ns) * pC;
                        lastCenter = it->centerPoint;
                    }
                    it = patchSamplesB.emplace(it);
                    it->centerPoint = lastCenter;
                    it->sidePoint = pB;
                    ++it;
                    // converge samples
                    for (; it != patchSamplesB.end(); ++it) {
                        double ns = (it->st[0] - wcutB) / (patchEndB - wcutB);
                        geometry::Vec2d ref = borderRayB.pos + it->st[0] * borderRayB.dir;
                        it->sidePoint = ns * it->sidePoint + (1 - ns) * ref;
                    }

                    double wcutA = (1 - w) * cutLengthMax;
                    it = patchSamplesA.begin();
                    lastCenter = data.position();
                    for (; it != patchSamplesA.end() && (it->st[0] < wcutA); ++it) {
                        double ns = it->st[0] / wcutA;
                        it->sidePoint = ns * pA + (1 - ns) * data.position();
                        lastCenter = it->centerPoint;
                    }
                    it = patchSamplesA.emplace(it);
                    it->centerPoint = lastCenter;
                    it->sidePoint = pA;
                    ++it;
                    // converge samples
                    for (; it != patchSamplesA.end(); ++it) {
                        double ns = (it->st[0] - wcutA) / (patchEndA - wcutA);
                        geometry::Vec2d ref = borderRayA.pos + it->st[0] * borderRayA.dir;
                        it->sidePoint = ns * it->sidePoint + (1 - ns) * ref;
                    }
                }
                else {
                    if (ts[0] > cutLengthMax) {
                        double delta = ts[0] - cutLengthMax;
                        double tOverlap = (std::max)(0.0, cutLengthMax - delta);
                        geometry::Vec2d lastCenter = data.position();
                        auto it = patchSamplesA.begin();
                        for (; it != patchSamplesA.end() && (it->st[0] < tOverlap);
                             ++it) {
                            double v = it->st[0] / ts[0];
                            it->st[1] = v * hwJoinA;
                            it->sidePoint = data.position() + v * bisect;
                            lastCenter = it->centerPoint;
                        }
                        it = patchSamplesA.emplace(it);
                        it->centerPoint = lastCenter;
                        it->sidePoint = data.position() + (tOverlap / ts[0]) * bisect;
                        geometry::Vec2d p = it->sidePoint;
                        fallbackPointA = p;
                        ++it;
                        geometry::Vec2d b =
                            borderRayA.pos + cutLengthMax * borderRayA.dir;
                        for (; it != patchSamplesA.end() && (it->st[0] < cutLengthMax);
                             ++it) {
                            double v = (it->st[0] - tOverlap) / (cutLengthMax - tOverlap);
                            it->sidePoint = v * b + (1 - v) * p;
                        }
                        for (; it != patchSamplesA.end(); ++it) {
                            double v =
                                (it->st[0] - cutLengthMax) / (patchEndA - cutLengthMax);
                            geometry::Vec2d p0 =
                                borderRayA.pos + it->st[0] * borderRayA.dir;
                            it->sidePoint = v * it->sidePoint + (1 - v) * p0;
                        }
                    }
                    else if (ts[0] > 0) {
                        double t = ts[0];
                        geometry::Vec2d lastCenter = data.position();
                        auto it = patchSamplesA.begin();
                        for (; it != patchSamplesA.end() && (it->st[0] < t); ++it) {
                            double v = it->st[0] / t;
                            it->st[1] = v * hwJoinA;
                            it->sidePoint = data.position() + v * bisect;
                            lastCenter = it->centerPoint;
                        }
                        it = patchSamplesA.emplace(it);
                        it->centerPoint = lastCenter;
                        it->sidePoint = vA;
                        ++it;
                        for (; it != patchSamplesA.end(); ++it) {
                            double v = (it->st[0] - t) / (patchEndA - t);
                            geometry::Vec2d p =
                                borderRayA.pos + it->st[0] * borderRayA.dir;
                            it->sidePoint = v * it->sidePoint + (1 - v) * p;
                        }
                    }

                    if (ts[1] > cutLengthMax) {
                        double delta = ts[1] - cutLengthMax;
                        double tOverlap = (std::max)(0.0, cutLengthMax - delta);
                        geometry::Vec2d lastCenter = data.position();
                        auto it = patchSamplesB.begin();
                        for (; it != patchSamplesB.end() && (it->st[0] < tOverlap);
                             ++it) {
                            double v = it->st[0] / ts[1];
                            it->st[1] = v * hwJoinB;
                            it->sidePoint = data.position() + v * bisect;
                            lastCenter = it->centerPoint;
                        }
                        it = patchSamplesB.emplace(it);
                        it->centerPoint = lastCenter;
                        it->sidePoint = data.position() + (tOverlap / ts[1]) * bisect;
                        geometry::Vec2d p = it->sidePoint;
                        fallbackPointB = p;
                        ++it;
                        geometry::Vec2d b =
                            borderRayB.pos + cutLengthMax * borderRayB.dir;
                        for (; it != patchSamplesB.end() && (it->st[0] < cutLengthMax);
                             ++it) {
                            double v = (it->st[0] - tOverlap) / (cutLengthMax - tOverlap);
                            it->sidePoint = v * b + (1 - v) * p;
                        }
                        for (; it != patchSamplesB.end(); ++it) {
                            double v =
                                (it->st[0] - cutLengthMax) / (patchEndB - cutLengthMax);
                            geometry::Vec2d p0 =
                                borderRayB.pos + it->st[0] * borderRayB.dir;
                            it->sidePoint = v * it->sidePoint + (1 - v) * p0;
                        }
                    }
                    else if (ts[1] > 0) {
                        double t = ts[1];
                        geometry::Vec2d lastCenter = data.position();
                        auto it = patchSamplesB.begin();
                        for (; it != patchSamplesB.end() && (it->st[0] < t); ++it) {
                            double v = it->st[0] / t;
                            it->st[1] = v * hwJoinB;
                            it->sidePoint = data.position() + v * bisect;
                            lastCenter = it->centerPoint;
                        }
                        it = patchSamplesB.emplace(it);
                        it->centerPoint = lastCenter;
                        it->sidePoint = vB;
                        ++it;
                        for (; it != patchSamplesB.end(); ++it) {
                            double v = (it->st[0] - t) / (patchEndB - t);
                            geometry::Vec2d p =
                                borderRayB.pos + it->st[0] * borderRayB.dir;
                            it->sidePoint = v * it->sidePoint + (1 - v) * p;
                        }
                    }

                    if (isExtA) {
                        // extension, rough
                        auto it = patchSamplesA.begin();
                        for (; it != patchSamplesA.end(); ++it) {
                            double v = it->st[0] / patchEndA;
                            geometry::Vec2d p =
                                borderRayA.pos + it->st[0] * borderRayA.dir;
                            it->sidePoint = v * it->sidePoint + (1 - v) * p;
                        }
                        auto& pA = patchSamplesA.emplaceFirst();
                        pA.centerPoint = data.position();
                        pA.sidePoint = fallbackPointB;
                    }

                    if (isExtB) {
                        // extension, rough
                        auto it = patchSamplesB.begin();
                        for (; it != patchSamplesB.end(); ++it) {
                            double v = it->st[0] / patchEndB;
                            geometry::Vec2d p =
                                borderRayB.pos + it->st[0] * borderRayB.dir;
                            it->sidePoint = v * it->sidePoint + (1 - v) * p;
                        }
                        auto& pB = patchSamplesB.emplaceFirst();
                        pB.centerPoint = data.position();
                        pB.sidePoint = fallbackPointA;
                    }

                    //detail::EdgeJoinPatchSample* pquad = nullptr;
                    //pquad = &patchSamplesB.emplaceLast();
                    //pquad->centerPoint = data.position() + patchEndB * borderRayB.dir;
                    //pquad->sidePoint = borderRayB.pos + patchEndB * borderRayB.dir;
                    //pquad = &patchSamplesB.emplaceLast();
                    //pquad->centerPoint = data.position();
                    //pquad->sidePoint = borderRayB.pos;
                    //pquad = &patchSamplesA.emplaceLast();
                    //pquad->centerPoint = data.position() + patchEndA * borderRayA.dir;
                    //pquad->sidePoint = borderRayA.pos + patchEndA * borderRayA.dir;
                    //pquad = &patchSamplesA.emplaceLast();
                    //pquad->centerPoint = data.position();
                    //pquad->sidePoint = borderRayA.pos;
                }
            }

            // fill data

            detail::EdgeJoinPatch& patchA =
                halfedgeA->edgeData_->patches_[isReverseA ? 2 : 1];
            patchA.sampleOverride_ = maxOverrideA;
            patchA.samples = patchSamplesA;

            detail::EdgeJoinPatch& patchB =
                halfedgeB->edgeData_->patches_[isReverseB ? 3 : 0];
            patchB.sampleOverride_ = maxOverrideB;
            patchB.samples = patchSamplesB;

            halfedgeA = halfedgeB;
            angleA = angleB;
            ++halfedgeB;
        }
    }

    data.isJoinComputed_ = true;
    data.isComputing_ = false;
} // namespace vgc::workspace

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
