// Copyright 2023 The VGC Developers
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

#include <vgc/dom/strings.h>
#include <vgc/geometry/bezier.h>
#include <vgc/geometry/vec4d.h>
#include <vgc/workspace/freehandedgegeometry.h>

namespace vgc::workspace {

namespace {

double cubicEaseInOut(double t) {
    double t2 = t * t;
    return -2 * t * t2 + 3 * t2;
}

} // namespace

void FreehandEdgeGeometry::setPoints(const SharedConstPoints& points) {
    if (isBeingEdited_) {
        points_ = points;
    }
    else {
        sharedConstPoints_ = points;
        originalArclengths_.clear();
    }
    dirtyEdgeSampling();
}

void FreehandEdgeGeometry::setPoints(geometry::Vec2dArray points) {
    if (isBeingEdited_) {
        points_ = std::move(points);
    }
    else {
        sharedConstPoints_ = SharedConstPoints(std::move(points));
        originalArclengths_.clear();
    }
    dirtyEdgeSampling();
}

void FreehandEdgeGeometry::setWidths(const SharedConstWidths& widths) {
    if (isBeingEdited_) {
        widths_ = widths;
    }
    else {
        sharedConstWidths_ = widths;
    }
    dirtyEdgeSampling();
}

void FreehandEdgeGeometry::setWidths(core::DoubleArray widths) {
    if (isBeingEdited_) {
        widths_ = std::move(widths);
        dirtyEdgeSampling();
    }
    else {
        sharedConstWidths_ = SharedConstWidths(std::move(widths));
    }
}

std::shared_ptr<vacomplex::KeyEdgeGeometry> FreehandEdgeGeometry::clone() const {
    auto ret = std::make_shared<FreehandEdgeGeometry>();
    ret->sharedConstPoints_ = sharedConstPoints_;
    ret->sharedConstWidths_ = sharedConstWidths_;
    return ret;
}

vacomplex::EdgeSampling FreehandEdgeGeometry::computeSampling(
    geometry::CurveSamplingQuality quality,
    const geometry::Vec2d& snapStartPosition,
    const geometry::Vec2d& snapEndPosition,
    vacomplex::EdgeSnapTransformationMode /*mode*/) const {

    geometry::Curve curve;
    geometry::CurveSampleArray samples;
    geometry::Vec2dArray tmpPoints;
    core::DoubleArray tmpWidths;

    const geometry::Vec2dArray& points = this->points();
    const core::DoubleArray& widths = this->widths();

    if (points.isEmpty()) {
        // fallback to segment
        tmpPoints = {snapStartPosition, snapEndPosition};
        tmpWidths = {1.0, 1.0};
        curve.setPositions(tmpPoints);
        curve.setWidths(tmpWidths);
    }
    else if (points.first() == snapStartPosition && points.last() == snapEndPosition) {
        curve.setPositions(points);
        curve.setWidths(widths);
    }
    else {
        core::DoubleArray tmpArclengths;
        computeSnappedLinearS_(
            tmpPoints, points, tmpArclengths, snapStartPosition, snapEndPosition);
        curve.setPositions(tmpPoints);
        curve.setWidths(widths);
    }

    if (isBeingEdited_) {
        quality = geometry::CurveSamplingQuality::AdaptiveLow;
    }

    curve.sampleRange(samples, geometry::CurveSamplingParameters(quality));
    VGC_ASSERT(samples.length() > 0);

    return vacomplex::EdgeSampling(std::move(samples));
}

vacomplex::EdgeSampling FreehandEdgeGeometry::computeSampling(
    geometry::CurveSamplingQuality quality,
    vacomplex::EdgeSnapTransformationMode /*mode*/) const {

    geometry::Curve curve;
    geometry::CurveSampleArray samples;
    geometry::Vec2dArray tmpPoints;
    core::DoubleArray tmpWidths;

    const geometry::Vec2dArray& points = this->points();
    const core::DoubleArray& widths = this->widths();

    if (points.isEmpty()) {
        // fallback to segment
        tmpPoints = {geometry::Vec2d(), geometry::Vec2d()};
        tmpWidths = {1.0, 1.0};
        curve.setPositions(tmpPoints);
        curve.setWidths(tmpWidths);
    }
    else {
        curve.setPositions(points);
        curve.setWidths(widths);
    }

    if (isBeingEdited_) {
        quality = geometry::CurveSamplingQuality::AdaptiveLow;
    }

    curve.sampleRange(samples, geometry::CurveSamplingParameters(quality));
    VGC_ASSERT(samples.length() > 0);

    return vacomplex::EdgeSampling(std::move(samples));
}

void FreehandEdgeGeometry::startEdit() {
    if (!isBeingEdited_) {
        points_ = sharedConstPoints_.get();
        widths_ = sharedConstWidths_.get();
        isBeingEdited_ = true;
    }
}

void FreehandEdgeGeometry::resetEdit() {
    if (isBeingEdited_) {
        points_ = sharedConstPoints_.get();
        widths_ = sharedConstWidths_.get();
        dirtyEdgeSampling();
    }
}

void FreehandEdgeGeometry::finishEdit() {
    if (isBeingEdited_) {
        sharedConstPoints_ = SharedConstPoints(std::move(points_));
        sharedConstWidths_ = SharedConstWidths(std::move(widths_));
        points_ = geometry::Vec2dArray();
        widths_ = core::DoubleArray();
        originalArclengths_.clear();
        originalArclengths_.shrinkToFit();
        dirtyEdgeSampling();
        isBeingEdited_ = false;
    }
}

void FreehandEdgeGeometry::abortEdit() {
    if (isBeingEdited_) {
        points_ = geometry::Vec2dArray();
        widths_ = core::DoubleArray();
        originalArclengths_.clear();
        originalArclengths_.shrinkToFit();
        dirtyEdgeSampling();
        isBeingEdited_ = false;
    }
}

namespace {

struct SculptPoint {
    SculptPoint(const geometry::Vec2d& pos, double width, double d, double s)
        : pos(pos)
        , width(width)
        , d(d)
        , s(s) {
    }

    geometry::Vec2d pos;
    double width = 0;
    double d = 0;
    double s = 0;
};

struct SculptSampling {
    core::Array<SculptPoint> sculptPoints;
    Int middleSculptPointIndex = -1;
    geometry::Vec2d radiusCaps = {};
    geometry::Vec2d intervalS = {};
    double ds = 0;
    bool isClosed = false;
    bool isOverlappingStart = false;
};

// Assumes:
// - radius > 0
// - mspS is in [samples.first.s(), samples.last.s()].
void computeSculptSampling(
    SculptSampling& outSampling,
    geometry::CurveSampleArray& samples,
    double mspS,
    double radius,
    double maxDs,
    bool isClosed) {

    core::Array<SculptPoint>& sculptPoints = outSampling.sculptPoints;

    Int numSamples = samples.length();
    Int numSamplingSegments = numSamples - 1;
    VGC_ASSERT(numSamples > 0);

    // Here we want to uniformly sample the curve over an arclength of `radius` on both
    // sides of the closest point.

    Int numSculptSamplesBeforeMsp = 0;
    Int numSculptSamplesAfterMsp = 0;
    double ds = 0;
    geometry::Vec2d radiusCaps = {};

    double totalS = samples.last().s();
    double halfS = totalS * 0.5;

    if (!isClosed) {
        double n = std::ceil(radius / maxDs);
        // guaranteed >= 1
        Int minSculptSamplesPerSide = static_cast<Int>(n);
        ds = radius / minSculptSamplesPerSide;
        double beforeMspS = mspS;
        if (radius <= beforeMspS) {
            numSculptSamplesBeforeMsp = minSculptSamplesPerSide;
            radiusCaps[0] = radius;
        }
        else {
            numSculptSamplesBeforeMsp = static_cast<Int>(beforeMspS / ds);
            radiusCaps[0] = beforeMspS;
        }
        double afterMspS = totalS - mspS;
        if (radius <= afterMspS) {
            numSculptSamplesAfterMsp = minSculptSamplesPerSide;
            radiusCaps[1] = radius;
        }
        else {
            numSculptSamplesAfterMsp = static_cast<Int>(afterMspS / ds);
            radiusCaps[1] = afterMspS;
        }
    }
    else { // isClosed
        if (halfS <= radius) {
            // If the sculpt interval encompasses the full curve and the curve is closed
            // then we want to produce a closed sculpt sampling.
            // To have the sculpt endpoints spaced by `ds` we have to adjust ds and
            // numSculptSamplesPerSide.
            //
            double n = std::ceil((halfS - maxDs * 0.5) / maxDs);
            numSculptSamplesBeforeMsp = static_cast<Int>(n);
            numSculptSamplesAfterMsp = static_cast<Int>(n);
            ds = halfS / (n + 0.5);
            outSampling.isClosed = true;
            radiusCaps[0] = halfS;
            radiusCaps[1] = halfS;
        }
        else {
            // If the curve is closed then we do not cap the radii to the input interval.
            //
            double n = std::ceil(radius / maxDs);
            numSculptSamplesBeforeMsp = static_cast<Int>(n);
            numSculptSamplesAfterMsp = static_cast<Int>(n);
            ds = radius / n;
            radiusCaps[0] = radius;
            radiusCaps[1] = radius;
        }
    }

    // CURRENT PROBLEM: if mspS == totalS then it returns 0
    // and this breaks doResamplingStep since it cannot create the last point.

    Int spEndIndex = numSculptSamplesAfterMsp + 1;
    Int spIndex = -numSculptSamplesBeforeMsp;

    const geometry::CurveSample* sa1 = &samples[0];
    double sculptPointSOffset = (mspS + spIndex * ds) < 0 ? totalS : 0;
    auto calcSculptPointS = [&](Int i) { return sculptPointSOffset + mspS + i * ds; };
    double nextSculpPointS = calcSculptPointS(spIndex);

    auto doResamplingStep = [&](Int sampleIndex2) -> bool {
        const geometry::CurveSample* sa2 = &samples[sampleIndex2];
        if (nextSculpPointS > sa2->s()) {
            sa1 = sa2;
            return false;
        }
        const double d = sa2->s() - sa1->s();
        if (d > 0) {
            while (1) {
                double t = (nextSculpPointS - sa1->s()) / d;
                if (t < 0.0 || t > 1.0) {
                    break;
                }
                double u = 1.0 - t;
                geometry::Vec2d p = u * sa1->position() + t * sa2->position();
                double w = (u * sa1->halfwidth(0) + t * sa2->halfwidth(0)) * 2.0;
                sculptPoints.emplaceLast(p, w, spIndex * ds, nextSculpPointS);
                ++spIndex;
                if (spIndex == spEndIndex) {
                    // all sculpt points have been sampled.
                    return true;
                }
                nextSculpPointS = calcSculptPointS(spIndex);
            }
        }
        sa1 = sa2;
        return false;
    };

    bool isDone = false;
    for (Int sampleIndex = 1; sampleIndex < numSamples && !isDone; ++sampleIndex) {
        isDone = doResamplingStep(sampleIndex);
    }
    if (isClosed && !isDone) {
        // loop and skip first point..
        sculptPointSOffset -= totalS;
        nextSculpPointS -= totalS;
        outSampling.isOverlappingStart = true;
        sa1 = &samples[0];
        for (Int sampleIndex = 1; sampleIndex < numSamples && !isDone; ++sampleIndex) {
            isDone = doResamplingStep(sampleIndex);
        }
    }
    VGC_ASSERT(isDone);

    outSampling.middleSculptPointIndex = numSculptSamplesBeforeMsp;
    outSampling.radiusCaps = radiusCaps;
    outSampling.intervalS = geometry::Vec2d(
        std::fmod(mspS - radiusCaps[0] + totalS, totalS),
        std::fmod(mspS + radiusCaps[1], totalS));
    outSampling.ds = ds;
}

[[maybe_unused]] Int filterSculptPointsWidthStep(
    core::Array<SculptPoint>& points,
    core::IntArray& indices,
    Int intervalStart,
    double tolerance) {

    Int i = intervalStart;
    Int endIndex = indices[i + 1];
    while (indices[i] != endIndex) {

        Int iA = indices[i];
        Int iB = indices[i + 1];
        const SculptPoint& a = points[iA];
        const SculptPoint& b = points[iB];
        double ds = b.s - a.s;

        // Compute which sample between A and B has an offset point
        // furthest from the offset line AB.
        double maxOffsetDiff = tolerance;
        Int maxOffsetDiffPointIndex = -1;
        for (Int j = iA + 1; j < iB; ++j) {
            const SculptPoint& p = points[j];
            double t = (p.s - a.s) / ds;
            double w = (1 - t) * a.width + t * b.width;
            double dist = (std::abs)(w - p.width);
            if (dist > maxOffsetDiff) {
                maxOffsetDiff = dist;
                maxOffsetDiffPointIndex = j;
            }
        }

        // If the distance exceeds the tolerance, then recurse.
        // Otherwise, stop the recursion and move one to the next segment.
        if (maxOffsetDiffPointIndex != -1) {
            // Add sample to the list of selected samples
            indices.insert(i + 1, maxOffsetDiffPointIndex);
        }
        else {
            ++i;
        }
    }
    return i;
}

Int filterSculptPointsStep(
    core::Array<SculptPoint>& points,
    core::IntArray& indices,
    Int intervalStart,
    double tolerance) {

    Int i = intervalStart;
    Int endIndex = indices[i + 1];
    while (indices[i] != endIndex) {

        // Get line AB. Fast continue if AB too small.
        Int iA = indices[i];
        Int iB = indices[i + 1];
        geometry::Vec2d a = points[iA].pos;
        geometry::Vec2d b = points[iB].pos;
        geometry::Vec2d ab = b - a;
        double abLen = ab.length();
        if (abLen < core::epsilon) {
            ++i;
            continue;
        }

        // Compute which sample between A and B has a position
        // furthest from the line AB.
        double maxDist = tolerance;
        Int maxDistPointIndex = -1;
        for (Int j = iA + 1; j < iB; ++j) {
            geometry::Vec2d p = points[j].pos;
            geometry::Vec2d ap = p - a;
            double dist = (std::abs)(ab.det(ap) / abLen);
            if (dist > maxDist) {
                maxDist = dist;
                maxDistPointIndex = j;
            }
        }

        // If the furthest point is too far from AB, then recurse.
        // Otherwise, stop the recursion and move one to the next segment.
        if (maxDistPointIndex != -1) {
            // Add sample to the list of selected samples
            indices.insert(i + 1, maxDistPointIndex);
        }
        else {
            //
            // Note: The width filtering step is disabled for now since it introduces
            // unwanted noisy widening of the curve.
            // This is probably caused by our use of Catmull-Rom to interpolate
            // widths: they can overshoot, then get sampled as sculpt points and
            // kept as new control points that overshoot further on the next grab.
            //
            /*
            Int i0 = indices[i];
            Int i1 = indices[i + 1];
            geometry::Vec2d previousPos = points[i0].pos;
            double s = points[i0].cumulativeS;
            for (Int j = i0 + 1; j < i1; ++j) {
            SculptPoint& sp = points[j];
            s += (sp.pos - previousPos).length();
            sp.cumulativeS = s;
            previousPos = sp.pos;
            }
            i = filterSculptPointsWidthStep(points, indices, i, tolerance);
            */
            ++i;
        }
    }
    return i;
}

} // namespace

geometry::Vec2d FreehandEdgeGeometry::sculptGrab(
    const geometry::Vec2d& startPosition,
    const geometry::Vec2d& endPosition,
    double radius,
    double /*strength*/,
    double tolerance,
    bool isClosed) {

    // Let's consider tolerance will be ~= pixelSize for now.
    //
    // sampleStep is screen-space-dependent.
    //   -> doesn't look like a good parameter..

    VGC_ASSERT(isBeingEdited_);

    Int numPoints = points_.length();
    if (numPoints == 0) {
        return endPosition;
    }

    // Note: We sample with widths even though we only need widths for samples in radius.
    // We could benefit from a two step sampling (sample centerline points, then sample
    // cross sections on an sub-interval).
    geometry::CurveSampleArray samples;
    geometry::Curve curve;
    curve.setPositions(points_);
    curve.setWidths(widths_);
    core::Array<double> pointsS(numPoints, core::noInit);
    pointsS[0] = 0;
    for (Int i = 1; i < numPoints; ++i) {
        if (i > 1) {
            samples.pop();
        }
        curve.sampleRange(
            samples, geometry::CurveSamplingQuality::AdaptiveLow, i - 1, i, true);
        pointsS[i] = samples.last().s();
    }

    // Note: we could have a distanceToCurve specialized for our geometry.
    // It could check each control polygon region first to skip sampling the ones
    // that are strictly farther than an other.
    geometry::DistanceToCurve d = geometry::distanceToCurve(samples, startPosition);
    if (d.distance() > radius) {
        return endPosition;
    }

    // Compute middle sculpt point info (closest point).
    Int mspSegmentIndex = d.segmentIndex();
    double mspSegmentParameter = d.segmentParameter();
    geometry::CurveSample mspSample = samples[mspSegmentIndex];
    if (mspSegmentParameter > 0 && mspSegmentIndex + 1 < samples.length()) {
        const geometry::CurveSample& s2 = samples[mspSegmentIndex + 1];
        mspSample = geometry::lerp(mspSample, s2, mspSegmentParameter);
    }
    double mspS = mspSample.s();

    const double maxDs = (tolerance * 2.0);

    SculptSampling sculptSampling = {};
    computeSculptSampling(sculptSampling, samples, mspS, radius, maxDs, isClosed);

    core::Array<SculptPoint>& sculptPoints = sculptSampling.sculptPoints;

    geometry::Vec2d delta = endPosition - startPosition;
    geometry::Vec2d uMins =
        geometry::Vec2d(1.0, 1.0) - sculptSampling.radiusCaps / radius;
    geometry::Vec2d wMins(cubicEaseInOut(uMins[0]), cubicEaseInOut(uMins[1]));

    for (Int i = 0; i < sculptPoints.length(); ++i) {
        SculptPoint& sp = sculptPoints[i];
        double u = 0;
        double wMin = 0;
        if (sp.d < 0) {
            u = 1.0 - (-sp.d / radius);
            wMin = wMins[0];
        }
        else if (sp.d > 0) {
            u = 1.0 - (sp.d / radius);
            wMin = wMins[1];
        }
        else {
            // when sculpt point is an endpoint
            u = 1.0;
            wMin = 0.0;
        }
        double w = cubicEaseInOut(u);
        double t = isClosed ? w : (w - wMin) / (1 - wMin);
        sp.pos += delta * t;
    }

    core::IntArray indices({0, sculptPoints.length() - 1});
    filterSculptPointsStep(sculptPoints, indices, 0, tolerance * 0.5);
    Int numPatchPoints = indices.length();

    constexpr bool keepAll = false;
    if (keepAll) {
        indices.clear();
        for (Int i = 0; i < sculptPoints.length(); ++i) {
            indices.append(i);
        }
        numPatchPoints = sculptPoints.length();
    }

    bool hasWidths = !widths_.isEmpty();

    double s0 = sculptSampling.sculptPoints.first().s;
    double sn = sculptSampling.sculptPoints.last().s;

    if (sculptSampling.isClosed) {
        points_.resize(numPatchPoints + 1);
        for (Int i = 0; i < numPatchPoints; ++i) {
            const SculptPoint& sp = sculptPoints[indices[i]];
            points_[i] = sp.pos;
        }
        points_.last() = points_.first();
        if (hasWidths) {
            widths_.resize(numPatchPoints + 1);
            for (Int i = 0; i < numPatchPoints; ++i) {
                const SculptPoint& sp = sculptPoints[indices[i]];
                widths_[i] = sp.width;
            }
            widths_.last() = widths_.first();
        }
    }
    else if (sculptSampling.isOverlappingStart) {
        if (sn > s0) {
            // only known case: s0 was totalS but modulo'd to 0
            s0 = samples.last().s();
        }

        // original points to keep are in the middle of the original array
        Int keepIndex = 0;
        for (Int i = 0; i < numPoints; ++i) {
            keepIndex = i;
            if (pointsS[i] >= sn) {
                break;
            }
        }
        Int keepIndexEnd = 0;
        for (Int i = keepIndex; i < numPoints; ++i) {
            keepIndexEnd = i;
            if (pointsS[i] > s0) {
                break;
            }
        }
        Int keepCount = keepIndexEnd - keepIndex;

        points_.erase(points_.begin(), points_.begin() + keepIndex);
        points_.resize(keepCount + numPatchPoints + 1);
        for (Int i = 0; i < numPatchPoints; ++i) {
            const SculptPoint& sp = sculptPoints[indices[i]];
            points_[keepCount + i] = sp.pos;
        }
        points_.last() = points_.first();
        if (hasWidths) {
            widths_.erase(widths_.begin(), widths_.begin() + keepIndex);
            widths_.resize(keepCount + numPatchPoints + 1);
            for (Int i = 0; i < numPatchPoints; ++i) {
                const SculptPoint& sp = sculptPoints[indices[i]];
                widths_[keepCount + i] = sp.width;
            }
            widths_.last() = widths_.first();
        }
    }
    else {
        if (s0 > sn) {
            // only known case: sn was totalS but modulo'd to 0
            sn = samples.last().s();
        }

        // original points to keep are at the beginning and end of the original array
        Int insertIndex = 0;
        for (Int i = 0; i < numPoints; ++i) {
            insertIndex = i;
            if (pointsS[i] >= s0) {
                break;
            }
        }
        // force keep first points
        if (insertIndex < 1) {
            insertIndex = 1;
        }
        Int insertIndexEnd = 0;
        for (Int i = insertIndex; i < numPoints; ++i) {
            insertIndexEnd = i;
            if (pointsS[i] > sn) {
                break;
            }
        }
        // force keep last points
        if (insertIndexEnd > numPoints - 1) {
            insertIndexEnd = numPoints - 1;
        }

        points_.erase(points_.begin() + insertIndex, points_.begin() + insertIndexEnd);
        points_.insert(insertIndex, numPatchPoints, {});
        for (Int i = 0; i < numPatchPoints; ++i) {
            const SculptPoint& sp = sculptPoints[indices[i]];
            points_[insertIndex + i] = sp.pos;
        }
        if (hasWidths) {
            widths_.erase(
                widths_.begin() + insertIndex, widths_.begin() + insertIndexEnd);
            widths_.insert(insertIndex, numPatchPoints, {});
            for (Int i = 0; i < numPatchPoints; ++i) {
                const SculptPoint& sp = sculptPoints[indices[i]];
                widths_[insertIndex + i] = sp.width;
            }
        }
    }

    dirtyEdgeSampling();

    return sculptPoints[sculptSampling.middleSculptPointIndex].pos;

    // Depending on the sculpt kernel we may have to duplicate the points
    // at the sculpt boundary to "extrude" properly.

    // problem: cannot reuse distanceToCurve.. samples don't have their segment index :(

    // In arclength mode, step is not supported so we have to do this only once.
    // In spatial mode, step is supported and we may have to do this at every step.
}

geometry::Vec2d FreehandEdgeGeometry::sculptSmooth(
    const geometry::Vec2d& position,
    double /*radius*/,
    double /*strength*/,
    double /*tolerance*/,
    bool /*isClosed*/) {

    return position;
}

void FreehandEdgeGeometry::snap(
    const geometry::Vec2d& snapStartPosition,
    const geometry::Vec2d& snapEndPosition,
    vacomplex::EdgeSnapTransformationMode /*mode*/) {

    const geometry::Vec2dArray& points = this->points();
    if (!points.isEmpty() && points.first() == snapStartPosition
        && points.last() == snapEndPosition) {
        // already snapped
        return;
    }

    if (isBeingEdited_) {
        computeSnappedLinearS_(
            points_, points, originalArclengths_, snapStartPosition, snapEndPosition);
    }
    else {
        computeSnappedLinearS_(
            points_, points, originalArclengths_, snapStartPosition, snapEndPosition);
        sharedConstPoints_ = SharedConstPoints(std::move(points_));
        points_ = geometry::Vec2dArray();
    }

    originalArclengths_.clear();
    dirtyEdgeSampling();
}

bool FreehandEdgeGeometry::updateFromDomEdge_(dom::Element* element) {
    namespace ds = dom::strings;

    bool changed = false;

    const auto& domPoints = element->getAttribute(ds::positions).getVec2dArray();
    if (sharedConstPoints_ != domPoints) {
        sharedConstPoints_ = domPoints;
        originalArclengths_.clear();
        dirtyEdgeSampling();
        changed = true;
    }

    const auto& domWidths = element->getAttribute(ds::widths).getDoubleArray();
    if (sharedConstWidths_ != domWidths) {
        sharedConstWidths_ = domWidths;
        dirtyEdgeSampling();
        changed = true;
    }

    return changed;
}

void FreehandEdgeGeometry::writeToDomEdge_(dom::Element* element) const {
    namespace ds = dom::strings;

    const auto& domPoints = element->getAttribute(ds::positions).getVec2dArray();
    if (sharedConstPoints_ != domPoints) {
        element->setAttribute(ds::positions, sharedConstPoints_);
    }

    const auto& domWidths = element->getAttribute(ds::widths).getDoubleArray();
    if (sharedConstWidths_ != domWidths) {
        element->setAttribute(ds::widths, sharedConstWidths_);
    }
}

void FreehandEdgeGeometry::removeFromDomEdge_(dom::Element* element) const {
    namespace ds = dom::strings;
    element->clearAttribute(ds::positions);
    element->clearAttribute(ds::widths);
}

void FreehandEdgeGeometry::computeSnappedLinearS_(
    geometry::Vec2dArray& outPoints,
    const geometry::Vec2dArray& srcPoints,
    core::DoubleArray& srcArclengths,
    const geometry::Vec2d& snapStartPosition,
    const geometry::Vec2d& snapEndPosition) {

    outPoints.resize(srcPoints.length());
    Int numPoints = outPoints.length();

    geometry::Vec2d a = snapStartPosition;
    geometry::Vec2d b = snapEndPosition;

    if (numPoints == 1) {
        // We would have to deal with "widths" if we want
        // to change the number of points.
        outPoints[0] = (a + b) * 0.5;
    }
    else if (numPoints == 2) {
        // We would have to deal with "widths" if we want
        // to change the number of points.
        outPoints[0] = a;
        outPoints[1] = b;
    }
    else {
        geometry::Vec2d d1 = a - srcPoints.first();
        geometry::Vec2d d2 = b - srcPoints.last();

        if (d1 == d2) {
            for (Int i = 0; i < numPoints; ++i) {
                outPoints[i] = srcPoints[i] + d1;
            }
        }
        else {
            if (srcArclengths.isEmpty()) {
                computeArclengths_(srcArclengths, srcPoints);
            }
            double totalS = srcArclengths.last();
            if (totalS > 0) {
                // linear deformation in rough "s"
                for (Int i = 0; i < numPoints; ++i) {
                    double t = srcArclengths[i] / totalS;
                    outPoints[i] = srcPoints[i] + (d1 + t * (d2 - d1));
                }
            }
            else {
                for (Int i = 0; i < numPoints; ++i) {
                    outPoints[i] = srcPoints[i] + d1;
                }
            }
        }
    }
}

void FreehandEdgeGeometry::computeArclengths_(
    core::DoubleArray& outArclengths,
    const geometry::Vec2dArray& srcPoints) {

    Int numPoints = srcPoints.length();
    outArclengths.resize(numPoints);
    if (numPoints == 0) {
        return;
    }

    outArclengths[0] = 0;
    geometry::Curve curve(0.0);
    geometry::CurveSampleArray sampling;
    geometry::CurveSamplingParameters sParams(
        geometry::CurveSamplingQuality::AdaptiveLow);
    curve.setPositions(srcPoints);
    double s = 0;
    for (Int i = 1; i < numPoints; ++i) {
        curve.sampleRange(sampling, sParams, i - 1, i, true);
        s += sampling.last().s();
        outArclengths[i] = s;
        sampling.clear();
    }
}

} // namespace vgc::workspace
