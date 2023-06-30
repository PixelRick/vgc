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
    return chordLengths_[segmentIndex] == 0;
}

Vec2d YukselSplineStroke2d::evalNonZeroCenterline(Int segmentIndex, double u) const {
    /*auto bezier = segmentToBezier(segmentIndex);
    return bezier.eval(u);*/
    return {};
}

Vec2d YukselSplineStroke2d::evalNonZeroCenterlineWithDerivative(
    Int segmentIndex,
    double u,
    Vec2d& dp) const {

    /*auto bezier = segmentToBezier(segmentIndex);
    return bezier.evalWithDerivative(u, dp);*/
    return {};
}

StrokeSampleEx2d YukselSplineStroke2d::evalNonZero(Int segmentIndex, double u) const {
    /*if (isWidthConstant_) {
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex);
        double hw = 0.5 * widths_[0];
        Vec2d dp(core::noInit);
        Vec2d p = centerlineBezier.evalWithDerivative(u, dp);
        return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
    }
    else {
        CubicBezier2d halfwidthsBezier(core::noInit);
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex, halfwidthsBezier);
        Vec2d dp(core::noInit);
        Vec2d p = centerlineBezier.evalWithDerivative(u, dp);
        Vec2d hw = halfwidthsBezier.eval(u);
        return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
    }*/
    return zeroLengthStrokeSample();
}

void YukselSplineStroke2d::sampleNonZeroSegment(
    StrokeSampleEx2dArray& out,
    Int segmentIndex,
    const CurveSamplingParameters& params) const {

    /*detail::AdaptiveStrokeSampler sampler = {};

    if (isWidthConstant_) {
        CubicBezier2d centerlineBezier = segmentToBezier(segmentIndex);
        double hw = 0.5 * widths_[0];
        sampler.sample(
            [&, hw](double u) -> StrokeSampleEx2d {
                Vec2d dp(core::noInit);
                Vec2d p = centerlineBezier.evalWithDerivative(u, dp);
                return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
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
                Vec2d p = centerlineBezier.evalWithDerivative(u, dp);
                Vec2d hw = halfwidthsBezier.eval(u);
                return StrokeSampleEx2d(p, dp, hw, segmentIndex, u);
            },
            params,
            out);
    }*/
}

StrokeSampleEx2d YukselSplineStroke2d::zeroLengthStrokeSample() const {
    return StrokeSampleEx2d(
        positions().first(), Vec2d(0, 1), 0.5 /*constantHalfwidth_*/, 0, 0);
}

std::array<Vec2d, 2> YukselSplineStroke2d::computeOffsetLineTangentsAtSegmentEndpoint_(
    Int segmentIndex,
    Int endpointIndex) const {

    return {Vec2d(1, 0), Vec2d(1, 0)};
}

void YukselSplineStroke2d::computeChordLengths_() {
    const auto& positions = this->positions();
    const Vec2d* p = positions.data();
    Int n = positions.length();
    chordLengths_.resizeNoInit(n);
    double* chordLengths = chordLengths_.data();
    if (n > 0) {
        for (Int i = 0; i < n - 1; ++i) {
            chordLengths[i] = (p[i + 1] - p[i]).length();
        }
        // We compute the closure even if the spline is not closed.
        chordLengths[n - 1] = (p[n - 1] - p[0]).length();
    }
}

} // namespace vgc::geometry
