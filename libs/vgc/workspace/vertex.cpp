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
            //d = geometry::Vec2f(s.outgoingTangent_);

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
    Int side) {

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
    Int side,
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

    detail::VacJoinFrameData& joinData = data.joinData_;
    geometry::Vec2d vertexPosition = data.position();

    // collect standalone edge data and halfwidths at join
    for (const detail::VacJoinHalfedge& he : joinHalfedges_) {
        VacEdgeCell* cell = he.edgeCell();
        VacEdgeCellFrameData* edgeData = cell->frameData(data.time());
        if (!edgeData) {
            // huh ?
            continue;
        }
        cell->computeStandaloneGeometry(*edgeData);
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
    constexpr double interpolationLimitCoefficient = 1.5;

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
        // Our current method considers incident straight lines of constant widths
        // and interpolates the original samples toward the computed samples.
        // This brings a few problems:
        // - The original samples projected onto the straight line model
        //   must remain in order. Otherwise it would result in a self-overlap.
        // - If the centerline is not contained in between the straight model outlines
        //   the interpolated outlines would cross it.
        //   We can either adapt the centerline or limit the patch length.

        // Limitations to work on:
        // - The two joins of a collapsing edge produce overlaps.
        //   In the context of animation we have to prevent popping when the two vertices
        //   become one.

        // compute the straight line model tangents and fix limits
        for (detail::VacJoinHalfedgeFrameData& heData : joinData.halfedgesData_) {
            // We approximate the tangent using the position of the first sample
            // that is either at a distance equal to the cut limit or is at half arclength.
            //
            const geometry::CurveSampleArray& samples = heData.edgeData_->samples_;
            const bool isReverse = heData.isReverse();
            const double endS = samples.last().s();
            // the patch cannot use more than half of the edge
            const double maxS = endS * 0.5;
            // we'll interpolate the center-line too and it is common to both sides
            const double patchLengthLimit =
                interpolationLimitCoefficient
                * std::max(heData.patchCutLimits_[0], heData.patchCutLimits_[1]);
            const double sqPatchLengthLimit = patchLengthLimit * patchLengthLimit;
            double patchLength = patchLengthLimit;
            //
            core::Array<geometry::CurveSample>& workingSamples = heData.workingSamples_;
            workingSamples.clear();
            if (!isReverse) {
                auto it = samples.begin();
                Int i = 0;
                auto previousIt = it;
                double previousSqDist = 0;
                for (; it != samples.end(); previousIt = it++, ++i) {
                    const geometry::CurveSample& sample = *it;
                    const geometry::Vec2d position = it->position();
                    const double sqDist = (vertexPosition - position).squaredLength();
                    const double s = it->s();
                    double tStop = 2;
                    if (s > maxS) {
                        const double previousS = previousIt->s();
                        tStop = (maxS - previousS) / (s - previousS);
                    }
                    if (sqDist > sqPatchLengthLimit) {
                        const double distance = std::sqrt(sqDist);
                        const double previousDistance = std::sqrt(previousSqDist);
                        tStop = (std::min)(
                            tStop,
                            (patchLengthLimit - previousDistance)
                                / (distance - previousDistance));
                    }
                    if (tStop <= 1) {
                        geometry::CurveSample mergeSample =
                            geometry::lerp(*previousIt, *it, tStop);
                        const double distance =
                            (vertexPosition - mergeSample.position()).length();
                        workingSamples.emplaceLast(mergeSample);
                        patchLength = (std::min)(patchLengthLimit, distance);
                        detail::EdgeJoinPatchMergeLocation& mergeLocation =
                            heData.edgeData_->patches_[0].mergeLocation;
                        mergeLocation.halfedgeNextSampleIndex = i;
                        mergeLocation.t = tStop;
                        mergeLocation.sample = mergeSample;
                        break;
                    }
                    workingSamples.emplaceLast(sample);
                    previousSqDist = sqDist;
                }
            }
            else {
                std::array<bool, 2> sideDone = {};
                auto it = samples.rbegin();
                Int i = 0;
                auto previousIt = it;
                double previousSqDist = 0;
                for (; it != samples.rend(); previousIt = it++, ++i) {
                    const geometry::CurveSample& sample = *it;
                    const geometry::Vec2d position = it->position();
                    const double sqDist = (vertexPosition - position).squaredLength();
                    const double s = endS - it->s();
                    double tStop = 2;
                    if (s > maxS) {
                        // lerp a new sample
                        const double previousS = endS - previousIt->s();
                        tStop = (maxS - previousS) / (s - previousS);
                    }
                    if (sqDist > sqPatchLengthLimit) {
                        const double distance = std::sqrt(sqDist);
                        const double previousDistance = std::sqrt(previousSqDist);
                        tStop = (std::min)(
                            tStop,
                            (patchLengthLimit - previousDistance)
                                / (distance - previousDistance));
                    }
                    if (tStop <= 1) {
                        geometry::CurveSample mergeSample =
                            geometry::lerp(*previousIt, *it, tStop);
                        const double distance =
                            (vertexPosition - mergeSample.position()).length();
                        workingSamples.emplaceLast(
                            mergeSample.position(),
                            -mergeSample.normal(),
                            geometry::Vec2d(
                                mergeSample.halfwidth(1), mergeSample.halfwidth(0)),
                            endS - mergeSample.s());
                        patchLength = (std::min)(patchLengthLimit, distance);
                        detail::EdgeJoinPatchMergeLocation& mergeLocation =
                            heData.edgeData_->patches_[1].mergeLocation;
                        mergeLocation.halfedgeNextSampleIndex = i;
                        mergeLocation.t = tStop;
                        mergeLocation.sample = mergeSample;
                        break;
                    }
                    workingSamples.emplaceLast(
                        sample.position(),
                        -sample.normal(),
                        geometry::Vec2d(sample.halfwidth(1), sample.halfwidth(0)),
                        s);
                    previousSqDist = sqDist;
                }
            }
            heData.patchLength_ = patchLength;
            const double patchCutLimit = patchLength / interpolationLimitCoefficient;
            heData.patchCutLimits_[0] =
                (std::min)(heData.patchCutLimits_[0], patchCutLimit);
            heData.patchCutLimits_[1] =
                (std::min)(heData.patchCutLimits_[1], patchCutLimit);

            geometry::Vec2d outgoingTangent =
                (workingSamples.last().position() - vertexPosition).normalized();
            heData.outgoingTangent_ = outgoingTangent;

            double angle = outgoingTangent.xAxisAngle();
            if (angle < 0) {
                angle += core::pi * 2;
            }
            heData.angle_ = angle;
        }

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

            auto& sidePatchDataA0 = halfedgeDataA->sidePatchData_[0];
            auto& sidePatchDataB1 = halfedgeDataB->sidePatchData_[1];

            detail::Ray borderRayA = {
                vertexPosition
                    + halfedgeDataA->outgoingTangent_.orthogonalized()
                          * halfedgeDataA->halfwidths_[0],
                halfedgeDataA->outgoingTangent_};
            sidePatchDataA0.borderRay = borderRayA;

            detail::Ray borderRayB = {
                vertexPosition
                    - halfedgeDataB->outgoingTangent_.orthogonalized()
                          * halfedgeDataB->halfwidths_[1],
                halfedgeDataB->outgoingTangent_};
            sidePatchDataB1.borderRay = borderRayB;

            sidePatchDataA0.clear();
            sidePatchDataB1.clear();

            sidePatchDataA0.joinHalfwidth = halfedgeDataA->halfwidths_[0];
            sidePatchDataB1.joinHalfwidth = halfedgeDataB->halfwidths_[1];

            std::optional<geometry::Vec2d> isect = borderRayA.intersectWith(borderRayB);
            if (isect.has_value()) {
                geometry::Vec2d ts = isect.value();
                if (ts[0] > 0 && ts[1] > 0) {
                    sidePatchDataA0.filletLength =
                        (std::min)(ts[0], halfedgeDataA->patchCutLimits_[0]);
                    sidePatchDataA0.isCutFillet = true;
                    sidePatchDataA0.joinHalfwidth = 0;
                    sidePatchDataB1.filletLength =
                        (std::min)(ts[1], halfedgeDataB->patchCutLimits_[1]);
                    sidePatchDataB1.isCutFillet = true;
                    sidePatchDataB1.joinHalfwidth = 0;
                }
                else if (ts[0] < 0 && ts[1] < 0) {
                    sidePatchDataA0.extLength = -ts[0];
                    sidePatchDataB1.extLength = -ts[1];
                }
                else {
                    if (halfedgeDataA->halfwidths_[0] > halfedgeDataB->halfwidths_[1]) {
                        geometry::Vec2d farCorner = borderRayA.origin;
                        detail::Ray splitRay{
                            vertexPosition, (farCorner - vertexPosition).normalized()};
                        geometry::Vec2d split = splitRay.intersectWith(borderRayB)
                                                    .value_or(geometry::Vec2d());

                        sidePatchDataA0.joinHalfwidth = split[0];
                        sidePatchDataA0.filletLength = halfedgeDataA->patchLength_;
                        if (ts[0] > 0) {
                            sidePatchDataA0.filletLength =
                                (std::min)(ts[0], halfedgeDataA->patchLength_);
                        }

                        const double tB = split[1];
                        if (tB > 0) {
                            sidePatchDataB1.filletLength =
                                (std::min)(tB, halfedgeDataB->patchCutLimits_[1]);
                            sidePatchDataB1.isCutFillet = true;
                            sidePatchDataB1.joinHalfwidth = 0;
                        }
                        else {
                            sidePatchDataB1.extLength = -tB;
                        }
                    }
                    else {
                        geometry::Vec2d farCorner = borderRayB.origin;
                        detail::Ray splitRay{
                            vertexPosition, (farCorner - vertexPosition).normalized()};
                        geometry::Vec2d split = splitRay.intersectWith(borderRayA)
                                                    .value_or(geometry::Vec2d());

                        sidePatchDataB1.joinHalfwidth = split[0];
                        sidePatchDataB1.filletLength = halfedgeDataB->patchLength_;
                        if (ts[1] > 0) {
                            sidePatchDataB1.filletLength =
                                (std::min)(ts[1], halfedgeDataB->patchLength_);
                        }

                        const double tA = split[1];
                        if (tA > 0) {
                            sidePatchDataA0.filletLength =
                                (std::min)(tA, halfedgeDataA->patchCutLimits_[0]);
                            sidePatchDataA0.isCutFillet = true;
                            sidePatchDataA0.joinHalfwidth = 0;
                        }
                        else {
                            sidePatchDataA0.extLength = -tA;
                        }
                    }
                }
            }
            else {
                // todo
            }
        }

        // now create the actual patches
        for (auto& halfedgeData : data.joinData_.halfedgesData_) {
            geometry::CurveSampleArray& workingSamples = halfedgeData.workingSamples_;

            const double maxFilletLength = (std::max)(
                halfedgeData.sidePatchData_[0].filletLength,
                halfedgeData.sidePatchData_[1].filletLength);

            detail::Ray centerRay = {vertexPosition, halfedgeData.outgoingTangent_};
            geometry::Vec2d centerRayNormal =
                halfedgeData.outgoingTangent_.orthogonalized();

            const double tFilletMax = maxFilletLength / halfedgeData.patchLength_;
            const double sMax = workingSamples.last().s();
            const double sFilletMax = tFilletMax * sMax;
            // straighten samples
            auto it = workingSamples.begin();
            auto previousIt = it;
            for (; it != workingSamples.end(); previousIt = it++) {
                const double s = it->s();
                if (s > sFilletMax) {
                    const double previousS = previousIt->s();
                    const double d = (sFilletMax / sMax) * halfedgeData.patchLength_;
                    const double t = (sFilletMax - previousS) / (s - previousS);
                    const double ot = 1 - t;
                    geometry::CurveSample newSample(
                        centerRay.pointAt(d),
                        centerRayNormal,
                        previousIt->halfwidths() * ot + it->halfwidths() * t,
                        sFilletMax);
                    it = workingSamples.emplace(it, newSample);
                    previousIt = it++;
                    break;
                }
                const double d = (s / sMax) * halfedgeData.patchLength_;
                it->setPosition(centerRay.pointAt(d));
                it->setNormal(centerRayNormal);
                geometry::Vec2f hwf(it->halfwidths());
                if (s == sFilletMax) {
                    previousIt = it++;
                    break;
                }
            }
            // lerp samples
            const double sInterp = sMax - sFilletMax;
            for (; it != workingSamples.end(); previousIt = it++) {
                const double s = it->s();
                const double d = (s / sMax) * halfedgeData.patchLength_;
                const double t = (s - sFilletMax) / sInterp;
                const double ot = 1 - t;
                geometry::Vec2d rayPoint = centerRay.pointAt(d);
                it->setPosition(rayPoint * ot + it->position() * t);
                it->setNormal((centerRayNormal * ot + it->normal() * t).normalized());
            }

            for (Int i = 0; i < 2; ++i) {
                core::Array<detail::EdgeJoinPatchSample> patchSamples;
                const auto& sidePatchData = halfedgeData.sidePatchData_[i];
                const auto& otherSidePatchData = halfedgeData.sidePatchData_[1 - i];
                const double halfwidth = halfedgeData.halfwidths_[i];

                // extension
                if (sidePatchData.extLength > 0) {
                    // todo: miter limit
                    auto& p = patchSamples.emplaceLast();
                    p.centerPoint = vertexPosition;
                    p.sidePoint =
                        sidePatchData.borderRay.pointAt(-sidePatchData.extLength);
                    p.sideSTUV =
                        geometry::Vec4f(0, static_cast<float>(halfwidth), 0, 1.f);
                    p.centerSU = geometry::Vec2f(0, 0);
                }

                const double tFillet =
                    sidePatchData.filletLength / halfedgeData.patchLength_;
                const double sFillet = tFillet * sMax;

                const double tFillet2 =
                    otherSidePatchData.filletLength / halfedgeData.patchLength_;
                const double sFillet2 = tFillet2 * sMax;

                const int normalMultiplier = i ? -1 : 1;

                it = workingSamples.begin();
                if (0) { // debug
                    for (; it != workingSamples.end(); ++it) {
                        const double s = it->s();
                        auto& p = patchSamples.emplaceLast();
                        p.centerPoint = it->position();
                        double hw = it->halfwidth(i);
                        p.sidePoint =
                            p.centerPoint + normalMultiplier * hw * it->normal();
                        p.sideSTUV = geometry::Vec4f(s, hw, s, 1.0);
                        p.centerSU = geometry::Vec2f(s, s);
                    }
                }
                else if (sidePatchData.isCutFillet) {
                    if (sFillet > 0) {
                        // lerp halfwidths from join halfwidth to cut halfwidth
                        previousIt = it;
                        for (; it != workingSamples.end(); previousIt = it++) {
                            const double s = it->s();
                            if (s > sFillet) {
                                const double d =
                                    (sFillet / sMax) * halfedgeData.patchLength_;
                                auto& p = patchSamples.emplaceLast();
                                p.centerPoint = centerRay.pointAt(d);
                                p.sidePoint = sidePatchData.borderRay.pointAt(d);
                                p.sideSTUV =
                                    geometry::Vec4f(sFillet, halfwidth, sFillet, 1.f);
                                p.centerSU = geometry::Vec2f(sFillet, sFillet);
                                break;
                            }
                            const double t = s / sFillet;
                            const double ot = 1 - t;
                            auto& p = patchSamples.emplaceLast();
                            p.centerPoint = it->position();
                            const double hw =
                                sidePatchData.joinHalfwidth * ot + halfwidth * t;
                            p.sidePoint =
                                p.centerPoint + normalMultiplier * hw * centerRayNormal;
                            p.sideSTUV = geometry::Vec4f(s, hw, s, 1.0);
                            p.centerSU = geometry::Vec2f(s, s);
                            if (s == sFillet) {
                                previousIt = it++;
                                break;
                            }
                        }
                    }
                    // lerp halfwidths from cut halfwidth to original halfwidth
                    Int fixIndex = 0;
                    const double sInterp2 = sMax - sFillet;
                    for (; it != workingSamples.end(); ++it) {
                        const double s = it->s();
                        const double d = (s / sMax) * halfedgeData.patchLength_;
                        const double t = (s - sFillet) / sInterp2;
                        const double ot = 1 - t;
                        // temporary fix
                        //if (s == sFillet2 && sFillet2 > 0 && sFillet2 != sFillet) {
                        //    fixIndex = patchSamples.size();
                        //    auto& p = patchSamples.emplaceLast(patchSamples.last());
                        //    p.centerPoint = it->position();
                        //    p.centerSU = geometry::Vec2f(s, s);
                        //    continue;
                        //}
                        auto& p = patchSamples.emplaceLast();
                        p.centerPoint = it->position();
                        const double hw = halfwidth * ot + it->halfwidth(i) * t;
                        p.sidePoint =
                            p.centerPoint + normalMultiplier * hw * it->normal();
                        p.sideSTUV = geometry::Vec4f(s, hw, s, 1.0);
                        p.centerSU = geometry::Vec2f(s, s);
                    }
                }
                else {
                    // lerp halfwidths from box halfwidth to original halfwidth
                    // additional lerp from join halfwidth to computed halfwidth between 0 and sFillet
                    Int fixIndex = 0;
                    for (; it != workingSamples.end(); ++it) {
                        const double s = it->s();
                        const double t = s / sMax;
                        const double ot = 1 - t;
                        // temporary fix
                        //if (s == sFillet2 && sFillet2 > 0 && sFillet2 != sFillet) {
                        //    fixIndex = patchSamples.size();
                        //    auto& p = patchSamples.emplaceLast(patchSamples.last());
                        //    p.centerPoint = it->position();
                        //    p.centerSU = geometry::Vec2f(s, s);
                        //}
                        auto& p = patchSamples.emplaceLast();
                        p.centerPoint = it->position();
                        double hw = halfwidth * ot + it->halfwidth(i) * t;
                        if (sFillet > 0 && s < sFillet) {
                            // this works but looks like magic written this way
                            const double t1 = sFillet / sMax;
                            const double ot1 = 1 - t1;
                            const double mhw = halfwidth * ot1 + it->halfwidth(i) * t1;

                            const double t2 = s / sFillet;
                            const double ot2 = 1 - t2;
                            hw = sidePatchData.joinHalfwidth * ot2 + mhw * t2;
                        }
                        p.sidePoint =
                            p.centerPoint + normalMultiplier * hw * it->normal();
                        p.sideSTUV = geometry::Vec4f(s, hw, s, 1.0);
                        p.centerSU = geometry::Vec2f(s, s);
                    }
                }

                // fill data
                const bool isReverse = halfedgeData.isReverse();
                detail::EdgeJoinPatch& patch0 =
                    halfedgeData.edgeData_->patches_[isReverse ? 1 : 0];
                patch0.sideSamples[i] = std::move(patchSamples);
            }
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
