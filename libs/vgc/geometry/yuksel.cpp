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

#include <vgc/geometry/yuksel.h>

#include <vgc/geometry/bezier.h>
#include <vgc/geometry/curve.h>

namespace vgc::geometry {

Int YukselSplineStroke2d::numKnots_() const {
    return positions_.length();
}

bool YukselSplineStroke2d::isZeroLengthSegment_(Int segmentIndex) const {
    return knotsData_[segmentIndex].chordLength == 0;
}

Vec2d YukselSplineStroke2d::evalNonZeroCenterline(Int segmentIndex, double u) const {
    auto centerlineSegment = segmentEvaluator(segmentIndex);
    return centerlineSegment.eval(u);
}

Vec2d YukselSplineStroke2d::evalNonZeroCenterline(Int segmentIndex, double u, Vec2d& dp)
    const {

    auto centerlineSegment = segmentEvaluator(segmentIndex);
    return centerlineSegment.eval(u, dp);
}

StrokeSampleEx2d YukselSplineStroke2d::evalNonZero(Int segmentIndex, double u) const {
    if (isWidthConstant_) {
        auto centerlineSegment = segmentEvaluator(segmentIndex);
        double hw = 0.5 * widths_[0];
        Vec2d dp(core::noInit);
        Vec2d p = centerlineSegment.eval(u, dp);
        return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
    }
    else {
        CubicBezier2d halfwidthsSegment(core::noInit);
        auto centerlineSegment = segmentEvaluator(segmentIndex, halfwidthsSegment);
        Vec2d dp(core::noInit);
        Vec2d p = centerlineSegment.eval(u, dp);
        Vec2d hw = halfwidthsSegment.eval(u);
        return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
    }
}

void YukselSplineStroke2d::sampleNonZeroSegment(
    StrokeSampleEx2dArray& out,
    Int segmentIndex,
    const CurveSamplingParameters& params) const {

    detail::AdaptiveStrokeSampler sampler = {};

    if (isWidthConstant_) {
        auto centerlineSegment = segmentEvaluator(segmentIndex);
        double hw = 0.5 * widths_[0];
        sampler.sample(
            [&, hw](double u) -> StrokeSampleEx2d {
                Vec2d dp(core::noInit);
                Vec2d p = centerlineSegment.eval(u, dp);
                return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
            },
            params,
            out);
    }
    else {
        CubicBezier2d halfwidthsSegment(core::noInit);
        auto centerlineSegment = segmentEvaluator(segmentIndex, halfwidthsSegment);
        sampler.sample(
            [&](double u) -> StrokeSampleEx2d {
                Vec2d dp(core::noInit);
                Vec2d p = centerlineSegment.eval(u, dp);
                Vec2d hw = halfwidthsSegment.eval(u);
                return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
            },
            params,
            out);
    }
}

StrokeSampleEx2d YukselSplineStroke2d::zeroLengthStrokeSample() const {
    return StrokeSampleEx2d(
        positions().first(), Vec2d(0, 1), 0.5 /*constantHalfwidth_*/, 0, 0);
}

std::array<Vec2d, 2> YukselSplineStroke2d::computeOffsetLineTangentsAtSegmentEndpoint_(
    Int /*segmentIndex*/,
    Int /*endpointIndex*/) const {

    return {Vec2d(1, 0), Vec2d(1, 0)};
}

namespace {

std::array<Int, 4> computeKnotIndices_(bool isClosed, Int numKnots, Int segmentIndex) {
    // Ensure we have a valid segment between two control points
    const Int numSegments = isClosed ? numKnots : (numKnots ? numKnots - 1 : 0);
    VGC_ASSERT(segmentIndex >= 0);
    VGC_ASSERT(segmentIndex < numSegments);

    // Get indices of points used by the interpolation, handle
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

void computeSegmentCenterlineYukselSegment_(
    YukselBezierSegment2d& segment,
    core::ConstSpan<Vec2d> knotPositions,
    core::ConstSpan<detail::YukselKnotData> knotsData,
    const std::array<Int, 4>& knotIndices,
    std::array<double, 3>& chordLengths) {

    std::array<Vec2d, 4> knots = {
        knotPositions.getUnchecked(knotIndices[0]),
        knotPositions.getUnchecked(knotIndices[1]),
        knotPositions.getUnchecked(knotIndices[2]),
        knotPositions.getUnchecked(knotIndices[3])};

    const detail::YukselKnotData& kd0 = knotsData.getUnchecked(knotIndices[0]);
    const detail::YukselKnotData& kd1 = knotsData.getUnchecked(knotIndices[1]);
    const detail::YukselKnotData& kd2 = knotsData.getUnchecked(knotIndices[2]);

    chordLengths = {kd0.chordLength, kd1.chordLength, kd2.chordLength};

    segment = YukselBezierSegment2d(knots, kd1.bi, kd1.ti, kd2.bi, kd2.ti);
}

void computeSegmentHalfwidthsCubicBezier_(
    CubicBezier2d& bezier,
    core::ConstSpan<double> knotWidths,
    const std::array<Int, 4>& knotIndices,
    const YukselBezierSegment2d& centerlineSegment,
    const std::array<double, 3>& chordLengths) {

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

    // Compute Bézier control points for halfwidths such that on both sides of
    // each knot we have the same desired dw/ds.
    //
    // desired dw/ds at start/end
    Vec2d dhw_ds_1 = (knots[2] - knots[0]) / (chordLengths[0] + chordLengths[1]);
    Vec2d dhw_ds_2 = (knots[3] - knots[1]) / (chordLengths[1] + chordLengths[2]);
    // 1/3 of ds/du at start/end
    double ds_du_1 = (1.0 / 3) * (centerlineSegment.startDerivative()).length();
    double ds_du_2 = (1.0 / 3) * (centerlineSegment.endDerivative()).length();
    // w1 - w0 = 1/3 of dw/du at start; w3 - w2 = 1/3 of dw/du at end
    Vec2d hw1 = knots[1] + dhw_ds_1 * ds_du_1;
    Vec2d hw2 = knots[2] - dhw_ds_2 * ds_du_2;

    bezier = CubicBezier2d(knots[1], hw1, hw2, knots[2]);
}

} // namespace

YukselBezierSegment2d YukselSplineStroke2d::segmentEvaluator(Int segmentIndex) const {
    YukselBezierSegment2d centerlineSegment(core::noInit);

    std::array<Int, 4> knotIndices =
        computeKnotIndices_(isClosed(), positions().length(), segmentIndex);

    std::array<double, 3> chordLengths;
    computeSegmentCenterlineYukselSegment_(
        centerlineSegment, positions(), knotsData_, knotIndices, chordLengths);

    return centerlineSegment;
}

YukselBezierSegment2d YukselSplineStroke2d::segmentEvaluator(
    Int segmentIndex,
    CubicBezier2d& halfwidths) const {

    YukselBezierSegment2d centerlineSegment(core::noInit);

    std::array<Int, 4> knotIndices =
        computeKnotIndices_(isClosed(), positions().length(), segmentIndex);

    std::array<double, 3> chordLengths;
    computeSegmentCenterlineYukselSegment_(
        centerlineSegment, positions(), knotsData_, knotIndices, chordLengths);

    if (isWidthConstant_) {
        double chw = 0.5 * constantWidth();
        Vec2d cp(chw, chw);
        halfwidths = CubicBezier2d(cp, cp, cp, cp);
    }
    else {
        computeSegmentHalfwidthsCubicBezier_(
            halfwidths, widths(), knotIndices, centerlineSegment, chordLengths);
    }

    return centerlineSegment;
}

namespace {

double
computeTiMaxCurvature_(const Vec2d& knot0, const Vec2d& knot1, const Vec2d& knot2) {
    // For now we use the exact formula but a numeric method may be as precise and faster.
    // See "High-Performance Polynomial Root Finding for Graphics" by Cem Yuksel.

    Vec2d v02 = knot2 - knot0;
    Vec2d v10 = knot0 - knot1;
    double a = v02.dot(v02);
    double b = 3 * v02.dot(v10);
    double c = (3 * knot0 - 2 * knot1 - knot2).dot(v10);
    double d = -v10.dot(v10);
    // Solving `ax³ + bx² + cx + d = 0` in [0, 1]
    // https://en.wikipedia.org/wiki/Cubic_equation
    // TODO: If a==0, solve a quadratic or linear equation.

    double p = (3 * a * c - b * b) / 3 / a / a;
    double q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / 27 / a / a / a;
    double discriminant = 4 * p * p * p + 27 * q * q;
    double t = 0;
    if (discriminant >= 0) {
        // Single real root
        return std::cbrt(-q / 2 + std::sqrt(q * q / 4 + p * p * p / 27))
               + std::cbrt(-q / 2 - std::sqrt(q * q / 4 + p * p * p / 27)) - b / 3 / a;
    }
    else {
        // Three real roots
        for (int k = 0; k < 3; ++k) {
            t = 2 * std::sqrt(-p / 3)
                    * std::cos(
                        1.0 / 3 * std::acos(3.0 * q / 2 / p * std::sqrt(-3.0 / p))
                        - 2.0 * core::pi * k / 3)
                - b / 3 / a;
            if (0 <= t && t <= 1) {
                return t;
            }
        }
    }
    // error
    //throw core::RuntimeError("fkjf");
    return 0.5;
}

Vec2d computeBi_(const Vec2d& knot0, const Vec2d& knot1, const Vec2d& knot2, double ti) {
    if (ti <= 0) {
        return knot0;
    }
    if (ti >= 1) {
        return knot1;
    }
    double qi = 1 - ti;
    double c = 1 / (2.0 * qi * ti);
    return c * (knot1 - qi * qi * knot0 - ti * ti * knot2);
}

} // namespace

void YukselSplineStroke2d::computeCache_() {
    const auto& positions = this->positions();
    const Vec2d* p = positions.data();
    Int n = positions.length();
    knotsData_.resizeNoInit(n);
    detail::YukselKnotData* knotsData = knotsData_.data();
    if (n > 0) {
        for (Int i = 0; i < n - 1; ++i) {
            knotsData[i].chordLength = (p[i + 1] - p[i]).length();
        }
        // We compute the closure even if the spline is not closed.
        knotsData[n - 1].chordLength = (p[n - 1] - p[0]).length();

        // compute bi and ti
        if (n > 1) {
            if (isClosed()) {
                knotsData[0].ti = computeTiMaxCurvature_(p[n - 1], p[0], p[1]);
                knotsData[0].bi = computeBi_(p[n - 1], p[0], p[1], knotsData[0].ti);
            }
            else {
                knotsData[0].ti = computeTiMaxCurvature_(p[0], p[0], p[1]);
                knotsData[0].bi = computeBi_(p[0], p[0], p[1], knotsData[0].ti);
            }
            for (Int i = 1; i < n - 1; ++i) {
                knotsData[i].ti = computeTiMaxCurvature_(p[i - 1], p[i], p[i + 1]);
                knotsData[i].bi = computeBi_(p[i - 1], p[i], p[i + 1], knotsData[i].ti);
            }
            if (isClosed()) {
                knotsData[n - 1].ti = computeTiMaxCurvature_(p[n - 2], p[n - 1], p[0]);
                knotsData[n - 1].bi =
                    computeBi_(p[n - 2], p[n - 1], p[0], knotsData[n - 1].ti);
            }
            else {
                knotsData[n - 1].ti =
                    computeTiMaxCurvature_(p[n - 2], p[n - 1], p[n - 1]);
                knotsData[n - 1].bi =
                    computeBi_(p[n - 2], p[n - 1], p[n - 1], knotsData[n - 1].ti);
            }
        }
        else {
            knotsData[0].ti = 0.5;
            knotsData[0].bi = p[0];
        }
    }
}

} // namespace vgc::geometry
