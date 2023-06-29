// Copyright 2021 The VGC Developers
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
    Vec2d p(core::noInit);
    p = bezier.evalPosition(u);
    return p;
}

Vec2d CatmullRomSplineStroke2d::evalNonZeroCenterlineWithDerivative(
    Int segmentIndex,
    double u,
    Vec2d& dp) const {

    auto bezier = segmentToBezier(segmentIndex);
    Vec2d p(core::noInit);
    bezier.evalPositionAndDerivative(u, p, dp);
    return p;
}

StrokeSample2d CatmullRomSplineStroke2d::evalNonZero(Int segmentIndex, double u) const {
    auto bezier = segmentToBezier(segmentIndex);
    Vec2d p(core::noInit), dp(core::noInit);
    bezier.evalPositionAndDerivative(u, p, dp);
    double hw = bezier.evalHalfwidths(u);
    return StrokeSample2d(p, dp, hw, segmentIndex, u);
}

void CatmullRomSplineStroke2d::sampleNonZeroSegment(
    Int segmentIndex,
    const CurveSamplingParameters& params,
    StrokeSample2dArray& out) const {

    detail::AdaptiveStrokeSampler sampler = {};
    auto bezier = segmentToBezier(segmentIndex);
    sampler.sample(
        [&](double u) -> detail::StrokeSampleEx2d {
            Vec2d p(core::noInit), dp(core::noInit);
            bezier.evalPositionAndDerivative(u, p, dp);
            double hw = bezier.evalHalfwidths(u);
            return {StrokeSample2d(p, dp, hw, segmentIndex, u)};
        },
        params,
        out);
}

StrokeSample2d CatmullRomSplineStroke2d::zeroLengthStrokeSample() const {
    return StrokeSample2d(positions().first(), Vec2d(0, 1), 0.0, 0, 0);
}

detail::CubicBezierStroke
CatmullRomSplineStroke2d::segmentToBezier(Int segmentIndex) const {
    if (isWidthConstant_) {
        return detail::CubicBezierStroke::fromCatmullRomSpline(
            parametrization_, positions(), widths()[0], isClosed(), segmentIndex);
    }
    else {
        return detail::CubicBezierStroke::fromCatmullRomSpline(
            parametrization_, positions(), widths(), isClosed(), segmentIndex);
    }
}

void CatmullRomSplineStroke2d::computeChordLengths_() {
    const auto& positions = this->positions();
    Int n = positions.length();
    chordLengths_.resize(n);
    if (n > 0) {
        for (Int i = 0; i < n - 1; ++i) {
            chordLengths_[i] = (positions[i + 1] - positions[i]).length();
        }
        chordLengths_[n - 1] = (positions[n - 1] - positions[0]).length();
    }
}

} // namespace vgc::geometry
