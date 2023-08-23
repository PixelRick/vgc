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

#ifndef VGC_GEOMETRY_INTERPOLATINGSTROKE_H
#define VGC_GEOMETRY_INTERPOLATINGSTROKE_H

#include <vgc/core/arithmetic.h>
#include <vgc/core/span.h>
#include <vgc/geometry/api.h>
#include <vgc/geometry/curve.h>
#include <vgc/geometry/vec2d.h>

namespace vgc::geometry {

enum class CurveSegmentType : UInt8 {
    Simple,
    Corner,
    AfterCorner,
    BeforeCorner,
    BetweenCorners,
};

class VGC_GEOMETRY_API AbstractInterpolatingStroke2d : public AbstractStroke2d {
protected:
    AbstractInterpolatingStroke2d(core::StringId implName, bool isClosed)
        : AbstractStroke2d(implName, isClosed) {
    }

    AbstractInterpolatingStroke2d(
        core::StringId implName,
        bool isClosed,
        double constantWidth)

        : AbstractStroke2d(implName, isClosed)
        , widths_(1, constantWidth)
        , isWidthConstant_(true) {
    }

    template<typename TRangePositions, typename TRangeWidths>
    AbstractInterpolatingStroke2d(
        core::StringId implName,
        bool isClosed,
        bool isWidthConstant,
        TRangePositions&& positions,
        TRangeWidths&& widths)

        : AbstractStroke2d(implName, isClosed)
        , positions_(std::forward<TRangePositions>(positions))
        , widths_(std::forward<TRangeWidths>(widths))
        , isWidthConstant_(isWidthConstant) {
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
        chordLengths_.clear();
        segmentTypes_.clear();
        onPositionsChanged_();
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
        isWidthConstant_ = false;
        onWidthsChanged_();
    }

    void setConstantWidth(double width) {
        isWidthConstant_ = true;
        widths_.resize(1);
        widths_[0] = width;
        onWidthsChanged_();
    }

    bool isWidthConstant() const {
        return isWidthConstant_;
    }

    double constantWidth() const {
        return widths_.isEmpty() ? 0. : widths_.getUnchecked(0);
    }

protected:
    struct SegmentComputeData {
        std::array<Int, 4> knotIndices;
        std::array<Vec2d, 3> chords;
        std::array<double, 3> chordLengths;
    };

    const core::DoubleArray& chordLengths() const {
        return chordLengths_;
    }

    const core::Array<CurveSegmentType>& segmentTypes() const {
        return segmentTypes_;
    }

    core::Array<SegmentComputeData> computeCache_() const;

private:
    Vec2dArray positions_;
    core::DoubleArray widths_;

    // It has the same number of elements as of positions_.
    // Last chord is the closure if closed, zero otherwise.
    mutable core::DoubleArray chordLengths_;
    mutable core::Array<CurveSegmentType> segmentTypes_;

    bool isWidthConstant_ = false;

    virtual void onPositionsChanged_() = 0;
    virtual void onWidthsChanged_() = 0;

    std::array<Vec2d, 2> computeOffsetLineTangentsAtSegmentEndpoint_(
        Int segmentIndex,
        Int endpointIndex) const override = 0;

    std::unique_ptr<AbstractStroke2d> clone_() const override = 0;
    bool copyAssign_(const AbstractStroke2d* other) override = 0;
    bool moveAssign_(AbstractStroke2d* other) override = 0;

    Int numKnots_() const override;

    bool isZeroLengthSegment_(Int segmentIndex) const override;

    StrokeSampling2d computeSampling_(
        const CurveSamplingParameters& params,
        const Vec2d& snapStartPosition,
        const Vec2d& snapEndPosition,
        CurveSnapTransformationMode mode =
            CurveSnapTransformationMode::LinearInArclength) const override;

    StrokeSampling2d
    computeSampling_(const geometry::CurveSamplingParameters& params) const override;

    void translate_(const geometry::Vec2d& delta) override;

    void transform_(const geometry::Mat3d& transformation) override;

    void snap_(
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition,
        CurveSnapTransformationMode mode) override;

    Vec2d sculptGrab_(
        const Vec2d& startPosition,
        const Vec2d& endPosition,
        double radius,
        double strength,
        double tolerance,
        bool isClosed) override;

    Vec2d sculptWidth_(
        const Vec2d& position,
        double delta,
        double radius,
        double tolerance,
        bool isClosed) override;

    Vec2d sculptSmooth_(
        const Vec2d& position,
        double radius,
        double strength,
        double tolerance,
        bool isClosed) override;
};

namespace detail {

void checkSegmentIndexIsValid(Int segmentIndex, Int numSegments) {
    VGC_ASSERT(segmentIndex >= 0);
    VGC_ASSERT(segmentIndex < numSegments);
}

template<std::size_t I, typename... T, std::size_t... Is>
constexpr std::tuple<std::tuple_element_t<I + Is, std::tuple<T...>>...>
SubPackAsTuple_(std::index_sequence<Is...>) {
}

template<typename T, size_t n, size_t... Is>
std::array<T, n> getElementsUnchecked_(
    const core::Array<T>& arr,
    const std::array<Int, n>& indices,
    std::index_sequence<Is...>) {

    return std::array<T, n>{arr.getUnchecked(indices[Is])...};
}

template<typename T, size_t n>
std::array<T, n>
getElementsUnchecked(const core::Array<T>& arr, const std::array<Int, n>& indices) {
    return getElementsUnchecked_(arr, indices, std::make_index_sequence<n>());
}

} // namespace detail

} // namespace vgc::geometry

#endif // VGC_GEOMETRY_CATMULLROM_H
