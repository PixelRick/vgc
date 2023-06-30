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

#ifndef VGC_GEOMETRY_YUKSEL_H
#define VGC_GEOMETRY_YUKSEL_H

#include <vgc/core/span.h>
#include <vgc/geometry/api.h>
#include <vgc/geometry/bezier.h>
#include <vgc/geometry/curve.h>
#include <vgc/geometry/vec2d.h>

namespace vgc::geometry {

class YukselSplineStroke2d : public AbstractStroke2d {
public:
    YukselSplineStroke2d(bool isClosed)
        : AbstractStroke2d(isClosed) {
    }

    YukselSplineStroke2d(bool isClosed, double constantWidth)
        : AbstractStroke2d(isClosed)
        , widths_(1, constantWidth)
        , isWidthConstant_(true) {
    }

    template<typename TRangePositions, typename TRangeWidths>
    YukselSplineStroke2d(
        bool isClosed,
        bool isWidthConstant,
        TRangePositions&& positions,
        TRangeWidths&& widths)

        : AbstractStroke2d(isClosed)
        , positions_(std::forward<TRangePositions>(positions))
        , widths_(std::forward<TRangeWidths>(widths))
        , isWidthConstant_(isWidthConstant) {

        computeChordLengths_();
    }

    const core::Array<Vec2d>& positions() const {
        return positions_;
    }

    core::Array<Vec2d>&& movePositions() {
        return std::move(positions_);
    }

    template<typename TRange>
    void setPositions(TRange&& positions) {
        positions_ = std::forward<TRange>(positions);
        computeChordLengths_();
    }

    const core::Array<double>& widths() const {
        return widths_;
    }

    // TODO: make data class and startEdit() endEdit()
    core::Array<double>&& moveWidths() {
        return std::move(widths_);
    }

    template<typename TRange>
    void setWidths(TRange&& widths) {
        widths_ = std::forward<TRange>(widths);
    }

    void setConstantWidth(double width) {
        isWidthConstant_ = true;
        widths_.resize(1);
        widths_[0] = width;
    }

    bool isWidthConstant() const {
        return isWidthConstant_;
    }

    const core::Array<double>& chordLengths() const {
        return chordLengths_;
    }

protected:
    Int numKnots_() const override;

    bool isZeroLengthSegment_(Int segmentIndex) const override;

    Vec2d evalNonZeroCenterline(Int segmentIndex, double u) const override;

    Vec2d evalNonZeroCenterlineWithDerivative(Int segmentIndex, double u, Vec2d& dp)
        const override;

    StrokeSampleEx2d evalNonZero(Int segmentIndex, double u) const override;

    void sampleNonZeroSegment(
        StrokeSampleEx2dArray& out,
        Int segmentIndex,
        const CurveSamplingParameters& params) const override;

    StrokeSampleEx2d zeroLengthStrokeSample() const override;

    std::array<Vec2d, 2> computeOffsetLineTangentsAtSegmentEndpoint_(
        Int segmentIndex,
        Int endpointIndex) const override;

    //CubicBezier2d segmentToBezier(Int segmentIndex) const;
    //CubicBezier2d segmentToBezier(Int segmentIndex, CubicBezier2d& halfwidths) const;

    double constantWidth() const {
        return widths_[0];
    }

private:
    core::Array<Vec2d> positions_;
    core::Array<double> widths_;
    core::Array<double> chordLengths_;
    bool isWidthConstant_ = false;

    void computeChordLengths_(); // TODO: rename computeCache_() ?
};

} // namespace vgc::geometry

#endif // VGC_GEOMETRY_CATMULLROM_H
