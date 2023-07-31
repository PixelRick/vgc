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

#include <vgc/geometry/catmullrom.h>

#include <vgc/geometry/bezier.h>
#include <vgc/geometry/curve.h>

namespace vgc::geometry {

Int CatmullRomSplineStroke2d::numKnots_() const {
    return positions_.length();
}

bool CatmullRomSplineStroke2d::isZeroLengthSegment_(Int segmentIndex) const {
    return chordLengths_[segmentIndex] == 0;
}

Vec2d CatmullRomSplineStroke2d::evalNonZeroCenterline(Int segmentIndex, double u) const {
    auto bezier = segmentToBezier(segmentIndex);
    return bezier.eval(u);
}

Vec2d CatmullRomSplineStroke2d::evalNonZeroCenterline(
    Int segmentIndex,
    double u,
    Vec2d& dp) const {

    auto bezier = segmentToBezier(segmentIndex);
    return bezier.eval(u, dp);
}

StrokeSampleEx2d CatmullRomSplineStroke2d::evalNonZero(Int segmentIndex, double u) const {
    if (isWidthConstant_) {
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex);
        double hw = 0.5 * widths_[0];
        Vec2d dp(core::noInit);
        Vec2d p = centerlineBezier.eval(u, dp);
        double speed = dp.length();
        return StrokeSampleEx2d(p, dp / speed, hw, speed, segmentIndex, u);
    }
    else {
        CubicBezier2d halfwidthsBezier(core::noInit);
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex, halfwidthsBezier);
        Vec2d dp(core::noInit);
        Vec2d p = centerlineBezier.eval(u, dp);
        double speed = dp.length();
        Vec2d hw = halfwidthsBezier.eval(u);
        return StrokeSampleEx2d(p, dp / speed, hw, speed, segmentIndex, u);
    }
}

void CatmullRomSplineStroke2d::sampleNonZeroSegment(
    StrokeSampleEx2dArray& out,
    Int segmentIndex,
    const CurveSamplingParameters& params) const {

    detail::AdaptiveStrokeSampler sampler = {};

    if (isWidthConstant_) {
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex);
        double hw = 0.5 * widths_[0];
        sampler.sample(
            [&, hw](double u) -> StrokeSampleEx2d {
                Vec2d dp(core::noInit);
                Vec2d p = centerlineBezier.eval(u, dp);
                double speed = dp.length();
                return StrokeSampleEx2d(p, dp / speed, hw, speed, segmentIndex, u);
            },
            params,
            out);
    }
    else {
        CubicBezier2d halfwidthsBezier(core::noInit);
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex, halfwidthsBezier);
        sampler.sample(
            [&](double u) -> StrokeSampleEx2d {
                Vec2d dp(core::noInit);
                Vec2d p = centerlineBezier.eval(u, dp);
                double speed = dp.length();
                Vec2d hw = halfwidthsBezier.eval(u);
                return StrokeSampleEx2d(p, dp / speed, hw, speed, segmentIndex, u);
            },
            params,
            out);
    }
}

StrokeSampleEx2d CatmullRomSplineStroke2d::zeroLengthStrokeSample() const {
    return StrokeSampleEx2d(
        positions().first(), Vec2d(0, 1), 0.5 /*constantHalfwidth_*/, 0, 0);
}

// Currently assumes first derivative at endpoint is non null.
// TODO: Support null derivatives (using limit analysis).
// TODO: Support different halfwidth on both sides.
namespace {

std::array<Vec2d, 2> computeOffsetLineTangents(
    const Vec2d& p,
    const Vec2d& dp,
    const Vec2d& ddp,
    const Vec2d& w,
    const Vec2d& dw,
    Int endpointIndex) {

    double dpl = dp.length();
    Vec2d n = dp.orthogonalized() / dpl;
    Vec2d dn = dp * (ddp.det(dp)) / (dpl * dpl * dpl);

    Vec2d offset0 = dn * w[0] + n * dw[0];
    Vec2d offset1 = -(dn * w[1] + n * dw[1]);
    return {(dp + offset0).normalized(), (dp + offset1).normalized()};
}

// For varying width strokes, cusps are were the derivative
// of offset line would become null if W'(u) were 0.
// O'(u) = P'(u) + N'(u) * W(u) + N(u) * W'(u)
// (offset line tangent becomes colinear with the normal).
//
void hasOffsetLineCusp(
    const std::array<Vec2d, 4>& positions,
    const std::array<Vec2d, 4>& halfwidths) {

    // We want to solve for u:
    // P'(u) + N'(u) * W(u) = (0, 0)
    //
    // P'(u) + N'(u) * W(u) = (0, 0)
    //
    // with
    //  Nx' = X' * (X"Y' - X'Y") / (X'² + Y'²)^(3/2)
    //  Ny' = Y' * (Y"X' - Y'X") / (Y'² + X'²)^(3/2)
    //
    // eq becomes
    //  K = (X"Y' - X'Y") / (X'² + Y'²)^(3/2)
    //  X' * (1 + [ K * W]) == 0
    //  Y' * (1 + [-K * W]) == 0
    //
    // since we want to check cusp on both sides,
    // we want K*W to equal 1 or -1
    //
    // W =     (1 - u)³      * w0
    //   + 3 * (1 - u)² * u  * w1
    //   + 3 * (1 - u)  * u² * w2
    //   +                u³ * w3;
    //
    // X' = 3 * (1 - u)²      * (x1 - x0)
    //    + 6 * (1 - u)  * u  * (x2 - x1)
    //    + 3            * u² * (x3 - x2);
    //
    // X" = 6 * (1 - u)     * (x2 - 2 * x1 + x0)
    //    + 6           * u * (x3 - 2 * x2 + x1);
    //

    //

    //return v3 * p0 + 3 * v2 * u * p1 + 3 * v * u2 * p2 + u3 * p3;
    //return //
    //    3 * v2 * (p1 - p0) + 6 * v * u * (p2 - p1) + 3 * u2 * (p3 - p2);
    //return 6 * (v * (p2 - 2 * p1 + p0) + u * (p3 - 2 * p2 + p1));
}

} // namespace

std::array<Vec2d, 2>
CatmullRomSplineStroke2d::computeOffsetLineTangentsAtSegmentEndpoint_(
    Int segmentIndex,
    Int endpointIndex) const {

    CubicBezier2d halfwidthBezier(core::noInit);
    CubicBezier2d positionsBezier = segmentToBezier(segmentIndex, halfwidthBezier);

    const std::array<Vec2d, 4>& positions = positionsBezier.controlPoints();
    const std::array<Vec2d, 4>& halfwidths = halfwidthBezier.controlPoints();

    Vec2d p = core::noInit;
    Vec2d dp = core::noInit;
    Vec2d ddp = core::noInit;
    Vec2d w = core::noInit;
    Vec2d dw = core::noInit;
    if (endpointIndex) {
        p = positions[3];
        dp = 3 * (positions[3] - positions[2]);
        ddp = 6 * (positions[3] - 2 * positions[2] + positions[1]);
        w = halfwidths[3];
        dw = 3 * (halfwidths[3] - halfwidths[2]);
    }
    else {
        p = positions[0];
        dp = 3 * (positions[1] - positions[0]);
        ddp = 6 * (positions[2] - 2 * positions[1] + positions[0]);
        w = halfwidths[0];
        dw = 3 * (halfwidths[1] - halfwidths[0]);
    }

    return computeOffsetLineTangents(p, dp, ddp, w, dw, endpointIndex);
}

namespace {

Vec2dArray computeChords(core::ConstSpan<Vec2d> knotPositions, bool isClosed) {
    const Vec2d* p = knotPositions.data();
    Int n = knotPositions.length();
    Vec2dArray chords(n, core::noInit);
    if (n > 0) {
        for (Int i = 0; i < n - 1; ++i) {
            chords[i] = p[i + 1] - p[i];
        }
        // Last chord is the closure.
        chords[n - 1] = p[n - 1] - p[0];
    }
}

void computeLengths(const Vec2dArray& vectors, core::DoubleArray& outLengths) {
    Int n = vectors.length();
    outLengths.resizeNoInit(n);
    for (Int i = 0; i < n; ++i) {
        outLengths[i] = vectors[i].length();
    }
}

Int calcNumSegments(Int numKnots, bool isClosed) {
    return isClosed ? numKnots : (numKnots ? numKnots - 1 : 0);
}

std::array<Int, 4> computeSegmentKnotIndices(
    Int numKnots,
    Int numSegments,
    bool isClosed,
    Int segmentIndex) {

    // Ensure we have a valid segmentIndex
    VGC_ASSERT(segmentIndex < numSegments);

    // Get indices of points used by the Catmull-Rom interpolation, handle
    // wrapping for closed curves and boundary for open curves.
    std::array<Int, 4> indices = {
        segmentIndex - 1, segmentIndex, segmentIndex + 1, segmentIndex + 2};
    if (isClosed) {
        if (indices[0] < 0) {
            indices[0] = numKnots - 1;
        }
        if (indices[2] > numKnots - 1) {
            indices[2] = 0;
            indices[3] = 1;
        }
        if (indices[3] > numKnots - 1) {
            indices[3] = 0;
        }
    }
    else {
        if (indices[0] < 0) {
            indices[0] = 0;
        }
        if (indices[2] > numKnots - 1) {
            indices[2] = numKnots - 1;
            indices[3] = numKnots - 1;
        }
        else if (indices[3] > numKnots - 1) {
            indices[3] = numKnots - 1;
        }
    }
    return indices;
}

std::array<Vec2d, 4> getSegmentKnotsUnchecked(
    core::ConstSpan<Vec2d> knotPositions,
    const std::array<Int, 4>& knotIndices) {

    return std::array<Vec2d, 4>{
        knotPositions.getUnchecked(knotIndices[0]),
        knotPositions.getUnchecked(knotIndices[1]),
        knotPositions.getUnchecked(knotIndices[2]),
        knotPositions.getUnchecked(knotIndices[3])};
}

std::array<Int, 3> computeSegmentChordIndices(
    const std::array<Int, 4>& knotIndices,
    Int numChords,
    bool isClosed) {

    std::array<Int, 3> indices;

    if (knotIndices[0] == knotIndices[1]) {
        indices[0] = numChords - 1;
    }
    else {
        indices[0] = knotIndices[0];
    }

    indices[1] = knotIndices[1];

    if (knotIndices[2] == knotIndices[3]) {
        indices[2] = numChords - 1;
    }
    else {
        indices[2] = knotIndices[2];
    }
}

std::array<Vec2d, 3> getSegmentChordsUnchecked(
    core::ConstSpan<Vec2d> chords,
    const std::array<Int, 3>& chordIndices) {

    return std::array<Vec2d, 3>{
        chords.getUnchecked(chordIndices[0]),
        chords.getUnchecked(chordIndices[1]),
        chords.getUnchecked(chordIndices[2])};
}

std::array<double, 3> getSegmentChordLengthsUnchecked(
    core::ConstSpan<double> chordLengths,
    const std::array<Int, 3>& chordIndices) {

    return std::array<double, 3>{
        chordLengths.getUnchecked(chordIndices[0]),
        chordLengths.getUnchecked(chordIndices[1]),
        chordLengths.getUnchecked(chordIndices[2])};
}

CurveSegmentType
computeSegmentTypeFromChordLengths(const std::array<double, 3>& segmentChordLengths) {
    if (segmentChordLengths[1] == 0) {
        return CurveSegmentType::Corner;
    }
    bool isAfterCorner = (segmentChordLengths[0] == 0);
    bool isBeforeCorner = (segmentChordLengths[2] == 0);
    if (isAfterCorner) {
        if (isBeforeCorner) {
            return CurveSegmentType::BetweenCorners;
        }
        return CurveSegmentType::AfterCorner;
    }
    else if (isBeforeCorner) {
        return CurveSegmentType::BeforeCorner;
    }
    else {
        return CurveSegmentType::Simple;
    }
}

std::array<double, 3> computeSegmentVirtualChordLengths(
    const std::array<double, 3>& chordLengths,
    CurveSegmentType segmentType) {

    std::array<double, 3> result;
    result[1] = chordLengths[1];

    // Note: computeSegmentCenterlineCubicBezier creates imaginary control points.
    // todo: better comment.

    if (segmentType == CurveSegmentType::AfterCorner) {
        result[0] = result[1];
        result[2] = chordLengths[2];
    }
    else if (segmentType == CurveSegmentType::BeforeCorner) {
        result[0] = chordLengths[0];
        result[2] = result[1];
    }

    return result;
}

CubicBezier2d computeSegmentCenterlineCubicBezier(
    CatmullRomSplineParameterization parameterization,
    const std::array<Vec2d, 4>& knots,
    const std::array<Vec2d, 3>& chords,
    const std::array<double, 3>& chordLengths,
    CurveSegmentType segmentType,
    std::array<Vec2d, 4>& outImaginaryKnots,
    std::array<double, 3>& outImaginaryChordLengths) {

    CubicBezier2d result(core::noInit);

    // Aliases
    const Vec2d p0p1 = chords[0];
    const Vec2d p1p2 = chords[1];
    const Vec2d p2p3 = chords[2];
    const double d01 = chordLengths[0];
    const double d12 = chordLengths[1];
    const double d23 = chordLengths[2];

    outImaginaryKnots = knots;
    outImaginaryChordLengths = chordLengths;

    // Handle "corner knots", defined as:
    // 1. Two consecutive equal points, or
    // 2. The first/last knot of an open curve
    //
    switch (segmentType) {
    case CurveSegmentType::Simple: {
        break;
    }
    case CurveSegmentType::Corner: {
        result = CubicBezier2d(knots);
        return result;
    }
    case CurveSegmentType::AfterCorner: {
        // (d01 == 0) && (d12 > 0) && (d23 > 0)
        //
        // Creates an imaginary control point p0 that would extrapolate the
        // curve, defined as:
        //
        //        p1    p2
        //         o----o         distance(p0, p1)  == distance(p1, p2)
        //        '      `        angle(p0, p1, p2) == angle(p1, p2, p3)
        //       o        `       w1 - w0           == w2 - w1
        //    p0           `
        //                  o p3
        //
        // Similarly to using "mirror tangents", this prevents ugly
        // inflexion points that would happen by keeping p0 = p1, as
        // illustrated here: https://github.com/vgc/vgc/pull/1341
        //
        Vec2d d = p2p3 / d23;                    // unit vector to reflect
        Vec2d n = (p1p2 / d12).orthogonalized(); // unit axis of reflexion
        Vec2d q = 2 * d.dot(n) * n - d;          // refection of d along n
        outImaginaryKnots[0] = knots[1] + d12 * q;
        outImaginaryChordLengths[0] = d12;
        break;
    }
    case CurveSegmentType::BeforeCorner: {
        // (d01 > 0) && (d12 > 0) && (d23 == 0)
        //
        // Similar as AfterCorner case above.
        Vec2d d = -p0p1 / d01;
        Vec2d n = (p1p2 / d12).orthogonalized();
        Vec2d q = 2 * d.dot(n) * n - d;
        outImaginaryKnots[3] = knots[2] + d12 * q;
        outImaginaryChordLengths[2] = d12;
        break;
    }
    case CurveSegmentType::BetweenCorners: {
        // (d01 == 0) && (d12 > 0) && (d23 == 0)
        //
        // Linear parameterization
        double u = 1.0 / 3;
        double v = (1 - u);
        result = CubicBezier2d(
            knots[1], v * knots[1] + u * knots[2], u * knots[1] + v * knots[2], knots[2]);
        return result;
    }
    }

    switch (parameterization) {
    case CatmullRomSplineParameterization::Uniform: {
        result = uniformCatmullRomToBezier<Vec2d>(outImaginaryKnots);
        break;
    }
    case CatmullRomSplineParameterization::Centripetal: {
        result = centripetalCatmullRomToBezier<Vec2d>(
            outImaginaryKnots, outImaginaryChordLengths);
        break;
    }
    case CatmullRomSplineParameterization::Chordal: {
        result =
            chordalCatmullRomToBezier<Vec2d>(outImaginaryKnots, outImaginaryChordLengths);
        break;
    }
    }

    // Test: fix tangent lengths using OGH
    // (Optimized Geometric Hermite)
    //
    const std::array<Vec2d, 4>& cps = result.controlPoints();
    Vec2d ab = cps[3] - cps[0];
    Vec2d v0 = (cps[1] - cps[0]).normalized();
    Vec2d v1 = (cps[3] - cps[2]).normalized();

    double abDotV0 = ab.dot(v0);
    double abDotV1 = ab.dot(v1);
    double v0v1 = v0.dot(v1);

    double den = 4 - v0v1 * v0v1;
    double a0 = std::abs((6 * abDotV0 - 3 * abDotV1 * v0v1) / den);
    double a1 = std::abs((6 * abDotV1 - 3 * abDotV0 * v0v1) / den);

    std::array<Vec2d, 4> newCps = {
        cps[0], cps[0] + v0 * a0 / 3, cps[3] - v1 * a1 / 3, cps[3]};

    result = CubicBezier2d(newCps);
    return result;
}

CubicBezier2d computeSegmentHalfwidthsCubicBezier(
    core::ConstSpan<double> knotWidths,
    const std::array<Int, 4>& knotIndices,
    const std::array<Vec2d, 4>& centerlineControlPoints,
    const std::array<double, 3>& virtualChordLengths,
    CurveSegmentType segmentType) {

    CubicBezier2d result(core::noInit);

    std::array<double, 4> hws = {
        0.5 * knotWidths.getUnchecked(knotIndices[0]),
        0.5 * knotWidths.getUnchecked(knotIndices[1]),
        0.5 * knotWidths.getUnchecked(knotIndices[2]),
        0.5 * knotWidths.getUnchecked(knotIndices[3])};

    std::array<Vec2d, 4> knots = {
        Vec2d(hws[0], hws[0]),
        Vec2d(hws[1], hws[1]),
        Vec2d(hws[2], hws[2]),
        Vec2d(hws[3], hws[3])};

    // Aliases
    const double d01 = virtualChordLengths[0];
    const double d12 = virtualChordLengths[1];
    const double d23 = virtualChordLengths[2];

    std::array<Vec2d, 2> fixedNeighborKnots;

    // Handle "corner knots", defined as:
    // 1. Two consecutive equal points, or
    // 2. The first/last knot of an open curve
    //
    switch (segmentType) {
    case CurveSegmentType::Simple: {
        fixedNeighborKnots[0] = knots[0];
        fixedNeighborKnots[1] = knots[3];
        break;
    }
    case CurveSegmentType::BetweenCorners:
    case CurveSegmentType::Corner: {
        double u = 1.0 / 3;
        double v = (1 - u);
        result = CubicBezier2d(
            knots[1], //
            v * knots[1] + u * knots[2],
            u * knots[1] + v * knots[2],
            knots[2]);
        // Fast return.
        return result;
    }
    case CurveSegmentType::AfterCorner: {
        // Imaginary control point, see `initPositions_()`.
        fixedNeighborKnots[0] = 2 * knots[1] - knots[2];
        fixedNeighborKnots[1] = knots[3];
        break;
    }
    case CurveSegmentType::BeforeCorner: {
        // Imaginary control point, see `initPositions_()`.
        fixedNeighborKnots[0] = knots[0];
        fixedNeighborKnots[1] = 2 * knots[2] - knots[1];
        break;
    }
    }

    // Compute Bézier control points for halfwidths such that on both sides of
    // each knot we have the same desired dw/ds.
    //
    double d012 = d01 + d12;
    double d123 = d12 + d23;
    // desired dw/ds at start/end
    Vec2d dhw_ds_1 = (knots[2] - fixedNeighborKnots[0]) / d012;
    Vec2d dhw_ds_2 = (fixedNeighborKnots[1] - knots[1]) / d123;
    // 1/3 of ds/du at start/end
    double ds_du_1 = (centerlineControlPoints[1] - centerlineControlPoints[0]).length();
    double ds_du_2 = (centerlineControlPoints[3] - centerlineControlPoints[2]).length();
    // w1 - w0 = 1/3 of dw/du at start; w3 - w2 = 1/3 of dw/du at end
    Vec2d hw1 = knots[1] + dhw_ds_1 * ds_du_1;
    Vec2d hw2 = knots[2] - dhw_ds_2 * ds_du_2;

    result = CubicBezier2d(knots[1], hw1, hw2, knots[2]);
    return result;
}

} // namespace

CubicBezier2d CatmullRomSplineStroke2d::segmentToBezier(Int segmentIndex) const {
    CubicBezier2d centerlineBezier(core::noInit);

    computeCache_();

    Int numSegments = this->numSegments();
    VGC_ASSERT(segmentIndex >= 0);
    VGC_ASSERT(segmentIndex < numSegments);

    Int numKnots = this->numKnots();
    if (segmentIndex == numKnots - 1) {
    }

    bool isClosed = this->isClosed();

    Int i0 = segmentIndex;
    Int i3 = segmentIndex + 1;
    if (i3 >= numKnots) {
        i3 = isClosed ? 0 : segmentIndex;
    }
    Int j = i0 * 2;

    const Vec2d* p = positions_.data();
    const Vec2d* cp = centerlineControlPoints_.data();
    return CubicBezier2d(p[i0], cp[j], cp[j + 1], p[i3]);
}

CubicBezier2d CatmullRomSplineStroke2d::segmentToBezier(
    Int segmentIndex,
    CubicBezier2d& halfwidths) const {

    computeCache_();

    Int numSegments = this->numSegments();
    VGC_ASSERT(segmentIndex >= 0);
    VGC_ASSERT(segmentIndex < numSegments);

    Int numKnots = this->numKnots();
    if (segmentIndex == numKnots - 1) {
    }

    bool isClosed = this->isClosed();

    Int i0 = segmentIndex;
    Int i3 = segmentIndex + 1;
    if (i3 >= numKnots) {
        i3 = isClosed ? 0 : segmentIndex;
    }
    Int j = i0 * 2;

    const double* w = widths_.data();
    const Vec2d* chw = halfwidthsControlPoints_.data();
    double hw0 = 0.5 * w[i0];
    double hw3 = 0.5 * w[i3];
    halfwidths = CubicBezier2d(Vec2d(hw0, hw0), chw[j], chw[j + 1], Vec2d(hw3, hw3));

    const Vec2d* p = positions_.data();
    const Vec2d* cp = centerlineControlPoints_.data();
    return CubicBezier2d(p[i0], cp[j], cp[j + 1], p[i3]);
}

void CatmullRomSplineStroke2d::computeCache_() const {

    if (!isCacheDirty_) {
        return;
    }

    const auto& positions = this->positions();
    const auto& widths = this->widths();
    Int numKnots = positions.length();
    Int numSegments = this->numSegments();
    bool isClosed = this->isClosed();

    bool computeChordLengthsCache = chordLengths_.isEmpty();

    Vec2dArray chords;

    if (computeChordLengthsCache) {
        chords = computeChords(positions, isClosed);
        if (chordLengths_.isEmpty()) {
            computeLengths(chords, chordLengths_);
        }
        segmentTypes_.resizeNoInit(numSegments);
    }

    centerlineControlPoints_.resizeNoInit(numSegments * 2);
    halfwidthsControlPoints_.resizeNoInit(numSegments * 2);

    Int numChords = chordLengths_.length();

    CubicBezier2d centerlineBezier(core::noInit);
    CubicBezier2d halfwidthsBezier(core::noInit);

    for (Int i = 0; i < numSegments; ++i) {

        std::array<Int, 4> knotIndices =
            computeSegmentKnotIndices(numKnots, numSegments, isClosed, i);

        std::array<Int, 3> chordIndices =
            computeSegmentChordIndices(knotIndices, numChords, isClosed);

        std::array<double, 3> segmentChordLengths =
            getSegmentChordLengthsUnchecked(chordLengths_, chordIndices);

        CurveSegmentType segmentType =
            computeSegmentTypeFromChordLengths(segmentChordLengths);

        std::array<Vec2d, 4> segmentKnots =
            getSegmentKnotsUnchecked(positions, knotIndices);
        std::array<Vec2d, 3> segmentChords =
            getSegmentChordsUnchecked(chords, chordIndices);

        std::array<Vec2d, 4> imaginarySegmentKnots;
        std::array<double, 3> imaginarySegmentChordLengths;

        centerlineBezier = computeSegmentCenterlineCubicBezier(
            parameterization_,
            segmentKnots,
            segmentChords,
            segmentChordLengths,
            segmentType,
            imaginarySegmentKnots,
            imaginarySegmentChordLengths);

        //if (isWidthConstant_) {
        //    double chw = 0.5 * constantWidth();
        //    Vec2d cp(chw, chw);
        //    halfwidthsControlPoints_[j] = cp;
        //    halfwidthsControlPoints_[j + 1] = cp;
        //}
        //else {
        halfwidthsBezier = computeSegmentHalfwidthsCubicBezier(
            widths,
            knotIndices,
            centerlineBezier.controlPoints(),
            imaginarySegmentChordLengths,
            segmentType);

        Int j = i * 2;
        centerlineControlPoints_[j] = centerlineBezier.controlPoints()[1];
        centerlineControlPoints_[j + 1] = centerlineBezier.controlPoints()[2];
        halfwidthsControlPoints_[j] = halfwidthsBezier.controlPoints()[1];
        halfwidthsControlPoints_[j + 1] = halfwidthsBezier.controlPoints()[2];
    }
}

} // namespace vgc::geometry
