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

#include <vgc/geometry/interpolatingstroke.h>

namespace vgc::geometry {

namespace {

Vec2dArray computeChords(core::ConstSpan<Vec2d> knotPositions) {
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
    return chords;
}

void computeLengths(const Vec2dArray& vectors, core::DoubleArray& outLengths) {
    Int n = vectors.length();
    outLengths.resizeNoInit(n);
    for (Int i = 0; i < n; ++i) {
        outLengths[i] = vectors[i].length();
    }
}

// Assumes segmentIndex is valid.
void computeSegmentKnotAndChordIndices(
    Int numKnots,
    bool isClosed,
    Int segmentIndex,
    std::array<Int, 4>& outKnotIndices,
    std::array<Int, 3>& outChordIndices) {

    // Get indices of points used by the Catmull-Rom interpolation, handle
    // wrapping for closed curves and boundary for open curves.
    outKnotIndices = {segmentIndex - 1, segmentIndex, segmentIndex + 1, segmentIndex + 2};
    outChordIndices = {segmentIndex - 1, segmentIndex, segmentIndex + 1};

    if (isClosed) {
        if (outKnotIndices[0] < 0) {
            outKnotIndices[0] = numKnots - 1;
            outChordIndices[0] = numKnots - 1;
        }
        if (outKnotIndices[2] > numKnots - 1) {
            outKnotIndices[2] = 0;
            outChordIndices[2] = 0;
            outKnotIndices[3] = 1;
        }
        if (outKnotIndices[3] > numKnots - 1) {
            outKnotIndices[3] = 0;
        }
    }
    else {
        Int zeroLengthChordIndex = numKnots - 1;
        if (outKnotIndices[0] < 0) {
            outKnotIndices[0] = 0;
            outChordIndices[0] = zeroLengthChordIndex;
        }
        if (outKnotIndices[2] > numKnots - 1) {
            outKnotIndices[2] = numKnots - 1;
            outChordIndices[2] = zeroLengthChordIndex;
            outKnotIndices[3] = numKnots - 1;
        }
        else if (outKnotIndices[3] > numKnots - 1) {
            outKnotIndices[3] = numKnots - 1;
        }
    }
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

} // namespace

core::Array<AbstractInterpolatingStroke2d::SegmentComputeData>
AbstractInterpolatingStroke2d::computeCache_() const {

    core::Array<SegmentComputeData> computeDataArray;

    Int numKnots = positions_.length();
    Int numSegments = this->numSegments();
    bool isClosed = this->isClosed();

    Vec2dArray chords = computeChords(positions_);
    if (!isClosed) {
        chords.last() = Vec2d();
    }

    bool updateSegmentTypes = false;
    if (chordLengths_.isEmpty()) {
        computeLengths(chords, chordLengths_);
        segmentTypes_.resizeNoInit(numSegments);
        updateSegmentTypes = true;
    }

    computeDataArray.resizeNoInit(numSegments);

    for (Int i = 0; i < numSegments; ++i) {
        SegmentComputeData& computeData = computeDataArray[i];

        std::array<Int, 4> knotIndices;
        std::array<Int, 3> chordIndices;
        computeSegmentKnotAndChordIndices(
            numKnots, isClosed, i, knotIndices, chordIndices);

        computeData.knotIndices = knotIndices;
        computeData.chords = getElementsUnchecked(chords, chordIndices);
        computeData.chordLengths = getElementsUnchecked(chordLengths_, chordIndices);

        if (updateSegmentTypes) {
            CurveSegmentType segmentType =
                computeSegmentTypeFromChordLengths(computeData.chordLengths);
            segmentTypes_.getUnchecked(i) = segmentType;
        }
    }

    return computeDataArray;
}

Int AbstractInterpolatingStroke2d::numKnots_() const {
    return positions_.length();
}

bool AbstractInterpolatingStroke2d::isZeroLengthSegment_(Int segmentIndex) const {
    computeCache_();
    return chordLengths_[segmentIndex] == 0;
}

StrokeSampling2d AbstractInterpolatingStroke2d::computeSampling_(
    const CurveSamplingParameters& params,
    const Vec2d& snapStartPosition,
    const Vec2d& snapEndPosition,
    CurveSnapTransformationMode mode =
        CurveSnapTransformationMode::LinearInArclength) const {
}

StrokeSampling2d AbstractInterpolatingStroke2d::computeSampling_(
    const geometry::CurveSamplingParameters& params) const {
}

void AbstractInterpolatingStroke2d::translate_(const geometry::Vec2d& delta) {
}

void AbstractInterpolatingStroke2d::transform_(const geometry::Mat3d& transformation) {
}

void AbstractInterpolatingStroke2d::snap_(
    const geometry::Vec2d& snapStartPosition,
    const geometry::Vec2d& snapEndPosition,
    CurveSnapTransformationMode mode) {
}

Vec2d AbstractInterpolatingStroke2d::sculptGrab_(
    const Vec2d& startPosition,
    const Vec2d& endPosition,
    double radius,
    double strength,
    double tolerance,
    bool isClosed) {
}

Vec2d AbstractInterpolatingStroke2d::sculptWidth_(
    const Vec2d& position,
    double delta,
    double radius,
    double tolerance,
    bool isClosed) {
}

Vec2d AbstractInterpolatingStroke2d::sculptSmooth_(
    const Vec2d& position,
    double radius,
    double strength,
    double tolerance,
    bool isClosed) {
}

} // namespace vgc::geometry
