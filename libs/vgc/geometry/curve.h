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

#ifndef VGC_GEOMETRY_CURVE_H
#define VGC_GEOMETRY_CURVE_H

#include <array>

#include <vgc/core/arithmetic.h>
#include <vgc/core/array.h>
#include <vgc/core/color.h>
#include <vgc/core/enum.h>
#include <vgc/core/object.h>
#include <vgc/core/span.h>
#include <vgc/geometry/api.h>
#include <vgc/geometry/mat3d.h>
#include <vgc/geometry/vec2d.h>

namespace vgc::geometry {

class StrokeView2d;

enum class CurveSamplingQuality {
    Disabled,
    UniformVeryLow,
    AdaptiveLow,
    UniformHigh,
    AdaptiveHigh,
    UniformVeryHigh,
    Max_ = UniformVeryHigh
};

VGC_GEOMETRY_API
VGC_DECLARE_ENUM(CurveSamplingQuality)

class StrokeSample2d {
public:
    constexpr StrokeSample2d() noexcept
        : derivativeInvLength_(1)
        , s_(0)
        , segmentIndex_(-1)
        , u_(-1)
        , isCornerStart_(false) {
    }

    VGC_WARNING_PUSH
    VGC_WARNING_MSVC_DISABLE(26495) // member variable uninitialized
    StrokeSample2d(core::NoInit) noexcept
        : position_(core::noInit)
        , derivative_(core::noInit)
        , halfwidths_(core::noInit) {
    }
    VGC_WARNING_POP

    StrokeSample2d(
        const Vec2d& position,
        const Vec2d& derivative,
        double halfwidth,
        Int segmentIndex = -1,
        double u = 0) noexcept

        : position_(position)
        , derivative_(derivative)
        , halfwidths_(halfwidth, halfwidth)
        , derivativeInvLength_(1.0 / derivative.length())
        , s_(0)
        , segmentIndex_(segmentIndex)
        , u_(u)
        , isCornerStart_(false) {
    }

    StrokeSample2d(
        const Vec2d& position,
        const Vec2d& derivative,
        const Vec2d& halfwidths,
        Int segmentIndex = -1,
        double u = 0) noexcept

        : position_(position)
        , derivative_(derivative)
        , halfwidths_(halfwidths)
        , derivativeInvLength_(1.0 / derivative.length())
        , s_(0)
        , segmentIndex_(segmentIndex)
        , u_(u)
        , isCornerStart_(false) {
    }

    const Vec2d& position() const {
        return position_;
    }

    void setPosition(const Vec2d& position) {
        position_ = position;
    }

    const Vec2d& derivative() const {
        return derivative_;
    }

    void setDerivative(const Vec2d& derivative) {
        derivative_ = derivative;
        derivativeInvLength_ = 1.0 / derivative.length();
    }

    Vec2d tangent() const {
        return derivative_ * derivativeInvLength_;
    }

    // ┌─── x
    // │ ─segment─→
    // y  ↓ normal
    //
    Vec2d normal() const {
        return tangent().orthogonalized();
    }

    // ┌─── x
    // │ ─segment─→
    // │  ↓ right
    // y
    //
    double rightHalfwidth() const {
        return halfwidths_[0];
    }

    // ┌─── x
    // │  ↑ left
    // │ ─segment─→
    // y
    //
    double leftHalfwidth() const {
        return halfwidths_[1];
    }

    // ┌─── x
    // │  ↑ halfwidths[1]
    // │ ─segment─→
    // y  ↓ halfwidths[0]
    //
    const Vec2d& halfwidths() const {
        return halfwidths_;
    }

    // ┌─── x
    // │  ↑ halfwidth(1)
    // │ ─segment─→
    // y  ↓ halfwidth(0)
    //
    double halfwidth(Int side) const {
        return halfwidths_[side];
    }

    // ┌─── x
    // │  ↑ halfwidths[1]
    // │ ─segment─→
    // y  ↓ halfwidths[0]
    //
    void setHalfwidths(const Vec2d& halfwidths) {
        halfwidths_ = halfwidths;
    }

    void setWidth(double rightHalfwidth, double leftHalfwidth) {
        halfwidths_[0] = rightHalfwidth;
        halfwidths_[1] = leftHalfwidth;
    }

    // ┌─── x
    // │ ─segment─→
    // │  ↓ right
    // y
    //
    Vec2d rightPoint() const {
        return position_ + normal() * halfwidths_[0];
    }

    // ┌─── x
    // │  ↑ left
    // │ ─segment─→
    // y
    //
    Vec2d leftPoint() const {
        return position_ - normal() * halfwidths_[1];
    }

    // ┌─── x
    // │  ↑ side 1
    // │ ─segment─→
    // y  ↓ side 0
    //
    Vec2d sidePoint(Int side) const {
        return side ? leftPoint() : rightPoint();
    }

    // ┌─── x
    // │  ↑ side 1
    // │ ─segment─→
    // y  ↓ side 0
    //
    std::array<Vec2d, 2> sidePoints() const {
        Vec2d normal = this->normal();
        return {position_ + normal * halfwidths_[0], position_ - normal * halfwidths_[1]};
    }

    // TODO: remove `s` from stroke sample class
    //       since it is computed after sampling it makes sense to externalize it.

    double s() const {
        return s_;
    }

    void setS(double s) {
        s_ = s;
    }

    Int segmentIndex() const {
        return segmentIndex_;
    }

    void setSegmentIndex(Int segmentIndex) {
        segmentIndex_ = segmentIndex;
    }

    double u() const {
        return u_;
    }

    void setU(double u) {
        u_ = u;
    }

    bool isCornerStart() const {
        return isCornerStart_;
    }

    void setCornerStart(bool isCornerStart) {
        isCornerStart_ = isCornerStart;
    }

private:
    Vec2d position_;
    Vec2d derivative_;
    Vec2d halfwidths_;
    double derivativeInvLength_;
    double s_; // arclength from stroke start point.
    Int segmentIndex_;
    double u_; // parameter in stroke segment.

    // isCornerStart_ is true only for the first sample of the two that makes
    // a corner (hard turn).
    bool isCornerStart_;

    // TODO: add enum/flags for corner kind ? knot corner, centerline cusp, offsetline cusp..
};

namespace detail {

enum CatmullRomSplineParameterization {
    Uniform,
    Centripetal
};

class VGC_GEOMETRY_API CubicBezierStroke {
public:
    // Uninitialized
    //
    CubicBezierStroke() noexcept = default;

    // Returns the CubicBezierStroke corresponding to the segment at
    // index [`i`, `i`+1] in the given `curve`.
    //
    static CubicBezierStroke fromStroke(const StrokeView2d* stroke, Int i);

    // Returns the CubicBezierStroke corresponding to the segment at
    // index [`i`, `i`+1] in the Catmull-Rom spline stroke defined by the
    // given `parameterization`, `knots`, `knotWidths`, and `isClosed`.
    //
    static CubicBezierStroke fromCatmullRomSpline(
        CatmullRomSplineParameterization parameterization,
        core::ConstSpan<Vec2d> knotPositions,
        core::ConstSpan<double> knotWidths,
        bool isClosed,
        Int i);

    // Constructs the CubicBezierStroke corresponding to the segment at
    // index [`i`, `i`+1] in the Catmull-Rom spline stroke defined by the
    // given `parameterization`, `knots`, `width`, and `isClosed`.
    //
    static CubicBezierStroke fromCatmullRomSpline(
        CatmullRomSplineParameterization parameterization,
        core::ConstSpan<Vec2d> knotPositions,
        double width,
        bool isClosed,
        Int i);

    const std::array<Vec2d, 4>& positions() const {
        return positions_;
    }

    const std::array<double, 4>& halfwidths() const {
        return halfwidths_;
    }

    bool isWidthUniform() const {
        return isWidthUniform_;
    }

    Vec2d evalPosition(double u) const;

    void evalPositionAndDerivative(double u, Vec2d& position, Vec2d& derivative) const;

    double evalHalfwidths(double u) const;

    void
    evalHalfwidthsAndDerivative(double u, double& halfwidth, double& derivative) const;

private:
    std::array<Vec2d, 4> positions_;
    std::array<double, 4> halfwidths_;
    bool isWidthUniform_ = false;

    enum class CornerCase {
        None = 0,
        Corner = 1,
        AfterCorner = 2,
        BeforeCorner = 3,
        BetweenCorners = 4,
    };

    // It also computes `knotSegments` and `knotSegmentLengths`.
    CornerCase initPositions_(
        CatmullRomSplineParameterization parameterization,
        core::ConstSpan<Vec2d> splineKnots,
        const std::array<Int, 4>& knotIndices,
        std::array<Vec2d, 3>& knotSegments,
        std::array<double, 3>& knotSegmentLengths);

    void initHalfwidths_(
        core::ConstSpan<double> splineKnots,
        const std::array<Int, 4>& knotIndices,
        const std::array<double, 3>& knotSegmentLengths,
        CornerCase cornerCase);

    static std::array<Int, 4> computeKnotIndices_(bool isClosed, Int numKnots, Int i);
};

} // namespace detail

/// Returns a new sample with each attribute linearly interpolated.
///
inline geometry::StrokeSample2d
lerp(const geometry::StrokeSample2d& a, const geometry::StrokeSample2d& b, double t) {
    const double ot = (1 - t);
    geometry::StrokeSample2d result(
        a.position() * ot + b.position() * t,
        a.derivative() * ot + b.derivative() * t,
        a.halfwidths() * ot + b.halfwidths() * t);
    result.setS(a.s() * ot + b.s() * t);
    return result;
}

/// Alias for `vgc::core::Array<vgc::geometry::StrokeSample2d>`.
///
using StrokeSample2dArray = core::Array<StrokeSample2d>;

class VGC_GEOMETRY_API DistanceToCurve {
public:
    DistanceToCurve() noexcept = default;

    DistanceToCurve(
        double distance,
        double angleFromTangent,
        Int segmentIndex,
        double segmentParameter) noexcept

        : distance_(distance)
        , angleFromTangent_(angleFromTangent)
        , segmentParameter_(segmentParameter)
        , segmentIndex_(segmentIndex) {
    }

    double distance() const {
        return distance_;
    }

    void setDistance(double distance) {
        distance_ = distance;
    }

    double angleFromTangent() const {
        return angleFromTangent_;
    }

    /// Returns the index of:
    /// - a segment containing a closest point, or
    /// - the index of the last sample.
    ///
    Int segmentIndex() const {
        return segmentIndex_;
    }

    /// Returns the parameter t between 0 and 1 such that
    /// `lerp(samples[segmentIndex], samples[segmentIndex + 1], t)`
    /// is a closest point.
    ///
    double segmentParameter() const {
        return segmentParameter_;
    }

private:
    double distance_ = 0;
    double angleFromTangent_ = 0;
    double segmentParameter_ = 0;
    Int segmentIndex_ = 0;
};

VGC_GEOMETRY_API
DistanceToCurve
distanceToCurve(const StrokeSample2dArray& samples, const Vec2d& position);

/// Alias for `vgc::core::SharedConstArray<vgc::geometry::StrokeSample2d>`.
///
using SharedConstStrokeSample2dArray = core::SharedConstArray<StrokeSample2d>;

/// \class vgc::geometry::CurveSamplingParameters
/// \brief Parameters for the sampling.
///
/// Using the default parameters, the sampling is "adaptive". This means that
/// the number of samples generated between two control points depends on the
/// curvature of the curve: the higher the curvature, the more samples are
/// generated. This ensures that consecutive segments never have an angle more
/// than `maxAngle` (expressed in radian), which ensures that the curve looks
/// "smooth" at any zoom level.
///
/// \verbatim
///                    _o p3
///                 _-`
///              _-` } maxAngle
/// o----------o`- - - - - - - - -
/// p1         p2
/// \endverbatim
///
/// Note that some algorithms may only take into account the angle between
/// consecutive segments of the centerline, while other may be width-aware,
/// that is, also ensure that the angle between consecutive segments of the
/// offset curves is less than `maxAngle`.
///
/// When the curve is a straight line between two control points, no intra
/// segment samples are needed. However, you can use `minIntraSegmentSamples`
/// if you wish to have at least a certain number of samples uniformly
/// generated between any two control points. Also, you can use
/// `maxIntraSegmentSamples` to limit how many samples are generated between
/// any two control points. This is necessary to break infinite loops in case
/// the curve contains a cusp between two control points.
///
/// If you wish to uniformly generate a fixed number of samples between control
/// points, simply set `maxAngle` to any value, and set
/// `minIntraSegmentSamples` and `maxIntraSegmentSamples` to the same value.
///
class VGC_GEOMETRY_API CurveSamplingParameters {
public:
    CurveSamplingParameters() = default;

    CurveSamplingParameters(CurveSamplingQuality quality);

    CurveSamplingParameters(
        double maxAngle,
        Int minIntraSegmentSamples,
        Int maxIntraSegmentSamples)

        : maxAngle_(maxAngle)
        , cosMaxAngle_(std::cos(maxAngle))
        , minIntraSegmentSamples_(minIntraSegmentSamples)
        , maxIntraSegmentSamples_(maxIntraSegmentSamples) {
    }

    double maxDs() const {
        return maxDs_;
    }

    void setMaxDs(double maxDs) {
        maxDs_ = maxDs;
    }

    double maxAngle() const {
        return maxAngle_;
    }

    void setMaxAngle(double maxAngle) {
        maxAngle_ = maxAngle;
        cosMaxAngle_ = std::cos(maxAngle);
    }

    double cosMaxAngle() const {
        return cosMaxAngle_;
    }

    Int minIntraSegmentSamples() const {
        return minIntraSegmentSamples_;
    }

    void setMinIntraSegmentSamples(Int minIntraSegmentSamples) {
        minIntraSegmentSamples_ = minIntraSegmentSamples;
    }

    Int maxIntraSegmentSamples() const {
        return maxIntraSegmentSamples_;
    }

    void setMaxIntraSegmentSamples(Int maxIntraSegmentSamples) {
        maxIntraSegmentSamples_ = maxIntraSegmentSamples;
    }

    friend bool
    operator==(const CurveSamplingParameters& a, const CurveSamplingParameters& b) {
        return (a.maxAngle_ == b.maxAngle_)
               && (a.minIntraSegmentSamples_ == b.minIntraSegmentSamples_)
               && (a.maxIntraSegmentSamples_ == b.maxIntraSegmentSamples_);
    }

    friend bool
    operator!=(const CurveSamplingParameters& a, const CurveSamplingParameters& b) {
        return !(a == b);
    }

private:
    double maxDs_ = core::DoubleInfinity;
    double maxAngle_ = 0.05; // 2PI / 0.05 ~= 125.66
    double cosMaxAngle_;
    Int minIntraSegmentSamples_ = 0;
    Int maxIntraSegmentSamples_ = 63;
    bool isScreenspace_ = false;
};

namespace detail {

using AdaptiveSamplingParameters = CurveSamplingParameters;

template<typename TSample>
class AdaptiveSampler {
private:
    struct Node {
    public:
        Node() = default;

        TSample sample;
        double u;
        Node* previous = nullptr;
        Node* next = nullptr;
    };

public:
    // Samples the segment [data.segmentIndex, data.segmentIndex + 1], and appends the
    // result to outAppend.
    //
    // KeepPredicate signature must match:
    //   `bool(const TSample& previousSample,
    //         const TSample& sample,
    //         const TSample& nextSample)`
    //
    // The first sample of the segment is appended only if the cache `data` is new.
    // The last sample is always appended.
    //
    template<typename USample, typename Evaluator, typename KeepPredicate>
    void sample(
        Evaluator evaluator,
        KeepPredicate keepPredicate,
        const AdaptiveSamplingParameters& params,
        core::Array<USample>& outAppend) {

        const Int minISS = params.minIntraSegmentSamples(); // 0 -> 2 samples minimum
        const Int maxISS = params.maxIntraSegmentSamples(); // 1 -> 3 samples maximum
        const Int minSamples = std::max<Int>(0, minISS) + 2;
        const Int maxSamples = std::max<Int>(minSamples, maxISS + 2);

        resetSampleTree_(maxSamples);

        // Setup first and last sample nodes of segment.
        Node* s0 = &sampleTree_[0];
        s0->sample = evaluator(0);
        s0->u = 0;
        Node* sN = &sampleTree_[1];
        sN->sample = evaluator(1);
        sN->u = 1;
        s0->previous = nullptr;
        s0->next = sN;
        sN->previous = s0;
        sN->next = nullptr;

        Int nextNodeIndex = 2;

        // Compute `minIntraSegmentSamples` uniform samples.
        Node* previousNode = s0;
        for (Int i = 1; i < minISS; ++i) {
            Node* node = &sampleTree_[nextNodeIndex];
            double u = static_cast<double>(i) / minISS;
            node->sample = evaluator(u);
            node->u = u;
            ++nextNodeIndex;
            linkNode_(node, previousNode);
            previousNode = node;
        }

        const Int sampleTreeLength = sampleTree_.length();
        Int previousLevelStartIndex = 2;
        Int previousLevelEndIndex = nextNodeIndex;

        // Fallback to using the last sample as previous level sample
        // when we added no uniform samples.
        if (previousLevelStartIndex == previousLevelEndIndex) {
            previousLevelStartIndex = 1;
        }

        while (nextNodeIndex < sampleTreeLength) {
            // Since we create a candidate on the left and right of each previous level node,
            // each pass can add as much as twice the amount of nodes of the previous level.
            for (Int i = previousLevelStartIndex; i < previousLevelEndIndex; ++i) {
                Node* previousLevelNode = &sampleTree_[i];
                // Try subdivide left.
                if (trySubdivide_(
                        evaluator,
                        keepPredicate,
                        nextNodeIndex,
                        previousLevelNode->previous,
                        previousLevelNode)
                    && nextNodeIndex == sampleTreeLength) {
                    break;
                }
                // We subdivide right only if it is not the last point.
                if (!previousLevelNode->next) {
                    continue;
                }
                // Try subdivide right.
                if (trySubdivide_(
                        evaluator,
                        keepPredicate,
                        nextNodeIndex,
                        previousLevelNode,
                        previousLevelNode->next)
                    && nextNodeIndex == sampleTreeLength) {
                    break;
                }
            }
            if (nextNodeIndex == previousLevelEndIndex) {
                // No new candidate, let's stop here.
                break;
            }
            previousLevelStartIndex = previousLevelEndIndex;
            previousLevelEndIndex = nextNodeIndex;
        }

        Node* node = &sampleTree_[0];
        while (node) {
            outAppend.emplaceLast(node->sample);
            node = node->next;
        }
    }

private:
    core::Span<Node> sampleTree_;
    std::unique_ptr<Node[]> sampleTreeStorage_;

    void resetSampleTree_(Int newStorageLength) {
        if (newStorageLength > sampleTree_.length()) {
            sampleTreeStorage_ = std::make_unique<Node[]>(newStorageLength);
            sampleTree_ = core::Span<Node>(sampleTreeStorage_.get(), newStorageLength);
        }
    }

    template<typename Evaluator, typename KeepPredicate>
    bool trySubdivide_(
        Evaluator evaluator,
        KeepPredicate keepPredicate,
        Int& nodeIndex,
        Node* n0,
        Node* n1) {

        Node* node = &sampleTree_[nodeIndex];
        node->sample = evaluator(0.5 * (n0->u + n1->u));
        if (keepPredicate(n0->sample, node->sample, n1->sample)) {
            ++nodeIndex;
            linkNode_(node, n0);
            return true;
        }
        return false;
    };

    static void linkNode_(Node* node, Node* previous) {
        Node* next = previous->next;
        next->previous = node;
        previous->next = node;
        node->previous = previous;
        node->next = next;
    };
};

class StrokeSampleEx2d : public StrokeSample2d {
public:
    VGC_WARNING_PUSH
    VGC_WARNING_MSVC_DISABLE(26495) // member variable uninitialized
    StrokeSampleEx2d() noexcept
        : StrokeSample2d(core::noInit) {
    }
    VGC_WARNING_POP

    StrokeSampleEx2d(const StrokeSample2d& sample)
        : StrokeSample2d(sample) {
        sidePoints_ = sidePoints();
    }

    StrokeSampleEx2d& operator=(const StrokeSample2d& sample) {
        StrokeSample2d::operator=(sample);
        sidePoints_ = sidePoints();
    }

    std::array<Vec2d, 2> sidePoints_;
};

bool isCenterlineSegmentUnderTolerance(
    const StrokeSampleEx2d& s0,
    const StrokeSampleEx2d& s1,
    double cosMaxAngle);

bool areOffsetLinesAnglesUnderTolerance(
    const StrokeSampleEx2d& s0,
    const StrokeSampleEx2d& s1,
    const StrokeSampleEx2d& s2,
    double cosMaxAngle);

bool shouldKeepNewSample(
    const StrokeSampleEx2d& previousSample,
    const StrokeSampleEx2d& sample,
    const StrokeSampleEx2d& nextSample,
    const CurveSamplingParameters& params);

class AdaptiveStrokeSampler : public AdaptiveSampler<StrokeSampleEx2d> {
public:
    template<typename USample, typename Evaluator>
    void sample(
        Evaluator&& evaluator,
        const AdaptiveSamplingParameters& params,
        core::Array<USample>& out) {

        AdaptiveSampler<StrokeSampleEx2d>::sample(
            std::forward<Evaluator>(evaluator),
            [&params](
                const StrokeSampleEx2d& previousSample,
                const StrokeSampleEx2d& sample,
                const StrokeSampleEx2d& nextSample) {
                return shouldKeepNewSample(previousSample, sample, nextSample, params);
            },
            params,
            out);
    }
};

} // namespace detail

/// \class vgc::geometry::WidthProfile
/// \brief A widths profile to apply on curves.
///
class WidthProfile {
public:
    WidthProfile() = default;

    // XXX todo

private:
    core::Array<Vec2d> values_;
};

namespace detail {

// TODO: We may want to have a lean version of AbstractStroke2d dedicated to
//       curves only (without thickness nor varying attributes). This class
//       is a draft of such interface.
//
/// \class vgc::geometry::AbstractCurve2d
/// \brief An abstract model of 2D curve (no thickness or other attributes).
///
class AbstractCurve2d {
public:
    virtual ~AbstractCurve2d() = default;

    /// Returns whether the curve is closed.
    ///
    virtual bool isClosed() const = 0;

    /// Returns the number of segments of the curve.
    ///
    /// \sa `eval()`.
    ///
    virtual Int numSegments() const = 0;

    /// Returns the position of the curve point from segment `segmentIndex` at
    /// parameter `u`.
    ///
    virtual Vec2d eval(Int segmentIndex, double u) const = 0;

    /// Returns the position of the curve point from segment `segmentIndex` at
    /// parameter `u`. It additionally sets the value of `derivative` as the
    /// position derivative at `u` with respect to the parameter u.
    ///
    virtual Vec2d
    evalWithDerivative(Int segmentIndex, double u, Vec2d& derivative) const = 0;
};

} // namespace detail

/// \class vgc::geometry::AbstractStroke2d
/// \brief An abstract model of 2D stroke.
///
class VGC_GEOMETRY_API AbstractStroke2d {
public:
    AbstractStroke2d(bool isClosed)
        : isClosed_(isClosed) {
    }

    virtual ~AbstractStroke2d() = default;

    /// Returns whether the stroke is closed.
    ///
    bool isClosed() const {
        return isClosed_;
    }

    /// Returns the number of knots of the stroke.
    ///
    Int numKnots() const {
        return numKnots_();
    }

    /// Returns the number of segments of the stroke.
    ///
    /// \sa `eval()`.
    ///
    Int numSegments() const {
        Int n = numKnots_();
        return (isClosed_ || n == 0) ? n : n - 1;
    }

    /// Returns whether the stroke segment at `segmentIndex` has a length of 0.
    ///
    bool isZeroLengthSegment(Int segmentIndex) const {
        return isZeroLengthSegment_(segmentIndex);
    }

    /// Returns the position of the centerline point from segment `segmentIndex` at
    /// parameter `u`.
    ///
    Vec2d evalCenterline(Int segmentIndex, double u) const;

    /// Returns the position of the centerline point from segment `segmentIndex` at
    /// parameter `u`. It additionally sets the value of `derivative` as the
    /// position derivative at `u` with respect to the parameter u.
    ///
    Vec2d
    evalCenterlineWithDerivative(Int segmentIndex, double u, Vec2d& derivative) const;

    /// Returns a `StrokeSample` from the segment `segmentIndex` at
    /// parameter `u`. The attribute `s` of the sample is left to 0.
    ///
    StrokeSample2d eval(Int segmentIndex, double u) const;

    // TODO: add variants of sampleSegment() and sampleRange() for CurveSample2d ?

    void sampleSegment(
        Int segmentIndex,
        const CurveSamplingParameters& params,
        StrokeSample2dArray& out) const;

    void sampleRange(
        StrokeSample2dArray& out,
        const CurveSamplingParameters& params,
        Int startKnotIndex = 0,
        Int numSegments = -1,
        bool computeArcLengths = true) const;

protected:
    virtual Int numKnots_() const = 0;

    virtual bool isZeroLengthSegment_(Int segmentIndex) const = 0;

    virtual Vec2d evalNonZeroCenterline(Int segmentIndex, double u) const = 0;

    virtual Vec2d
    evalNonZeroCenterlineWithDerivative(Int segmentIndex, double u, Vec2d& dp) const = 0;

    virtual StrokeSample2d evalNonZero(Int segmentIndex, double u) const = 0;

    virtual void sampleNonZeroSegment(
        Int segmentIndex,
        const CurveSamplingParameters& params,
        StrokeSample2dArray& out) const = 0;

    virtual StrokeSample2d zeroLengthStrokeSample() const = 0;

private:
    const bool isClosed_;

    StrokeSample2d sampleKnot_(Int knotIndex) const;
    bool fixEvalLocation_(Int& segmentIndex, double& u) const;
};

/// Specifies the type of the curve, that is, how the
/// position of its centerline is represented.
///
enum class CurveType {

    /// Represents an open uniform Catmull-Rom spline.
    ///
    /// With this curve type, we have `numSegment() == numKnots() - 1`.
    ///
    /// Each position p in `positions()` represent a control points (= knot
    /// in this case) of the spline. The curve starts at the first control
    /// point, ends at the last control point, and go through all control
    /// points.
    ///
    /// Each curve segment between two control points p[i] and p[i+1] is a
    /// cubic curve P_i(u) parameterized by u in [0, 1]. The derivative
    /// P_i'(u) at each control point (except end points) is automatically
    /// determined from its two adjacent control points:
    ///
    /// P_{i-1}'(1) = P_i'(0) = (p[i+1] - p[i-1]) / 2
    ///
    /// At end control points, we use a tangent mirrored from the adjacent control
    /// point, see: https://github.com/vgc/vgc/pull/1341.
    ///
    /// In addition, tangents are capped based on the distance between control points
    /// to avoid loops.
    ///
    /// Note: "uniform" refers here to the fact that this corresponds to a
    /// Catmull-Rom spline with knot values uniformly spaced, that is: [0,
    /// 1, 2, ..., n-1]. There exist other types of Catmull-Rom splines
    /// using different knot values, such as the Cardinal or Chordal
    /// Catmull-Rom, that uses the distance between control points to
    /// determine more suitable knot values to avoid loops, see:
    ///
    /// https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
    ///
    OpenUniformCatmullRom,

    /// Represents an open centripetal Catmull-Rom spline.
    ///
    /// Similar to `OpenUniformCatmullRom` but using centripetal
    /// parametrization. This prevents cusps and loops.
    ///
    OpenCentripetalCatmullRom,

    /// Represents a closed uniform Catmull-Rom spline.
    ///
    /// This is similar to `OpenUniformCatmullRom` except that it forms a loop.
    ///
    /// With this curve type, we have `numSegment() == numKnots()`: the last segment
    /// goes from the knot at index `numKnots() - 1` (the last knot) back to the knot
    /// at index `0` (the first knot).
    ///
    /// Unlike `OpenUniformCatmullRom`, there is no special handling of tangents
    /// for the first/last segment, since all knots have adjacent knots. In particular,
    /// the tangent at the first/last control point is determined by:
    ///
    /// P' = (p[1] - p[n-1]) / 2
    ///
    /// where n = numKnots().
    ///
    ClosedUniformCatmullRom,

    /// Represents a closed centripetal Catmull-Rom spline.
    ///
    /// Similar to `ClosedUniformCatmullRom` but using centripetal
    /// parametrization. This prevents cusps and loops.
    ///
    ClosedCentripetalCatmullRom
};

// Note: StrokeView2d might accept additional optional
// attributes (e.g., color) interpolated along the curve, but that do not
// affect how many samples are computed. This may or may not be templated
// (non-templated version could be an array of DoubleArrays)

/// \class vgc::geometry::StrokeView2d
/// \brief A helper class to sample a 2D stroke given external (non-owned) data.
///
/// This class can be used to sample a 2D stroke given external (non-owned) data.
///
/// Note that this class does not own the data provided, for example via the
/// `setPositions(positions)` and `setWidths(widths)` function. It is the
/// responsability of the programmer to ensure that the data referred to by the
/// given `Span` outlives the `StrokeView2d`.
///
/// Currently, only Catmull-Rom curves are supported, but more curve types are
/// expected in future versions.
///
class VGC_GEOMETRY_API StrokeView2d {
public:
    /// Specifies the type of a variable attribute along the curve, that is,
    /// how it is represented.
    ///
    enum class AttributeVariability {

        /// Represents a constant attribute. For example, a 2D attribute
        /// would be formatted as [ u, v ].
        ///
        Constant,

        /// Represents a varying attribute specified per control point. For
        /// example, a 2D attribute would be formatted as [ u0, v0, u1, v1,
        /// ..., un, vn ].
        ///
        PerControlPoint
    };

    /// Creates an empty stroke of the given \p type and PerControlPoint width
    /// variability.
    ///
    StrokeView2d(CurveType type = CurveType::OpenUniformCatmullRom);

    /// Creates an empty stroke of the given \p type and the given constant
    /// width variability.
    ///
    StrokeView2d(double constantWidth, CurveType type = CurveType::OpenUniformCatmullRom);

    /// Returns the CurveType of the centerline.
    ///
    CurveType type() const {
        return type_;
    }

    /// Returns whether the stroke is closed.
    ///
    bool isClosed() const {
        return isClosed_;
    }

    /// Returns the number of knots of the stroke.
    ///
    Int numKnots() const {
        return positions_.length();
    }

    /// Returns the number of segments of the stroke.
    ///
    Int numSegments() const;

    /// Returns the position data of the curve.
    ///
    core::ConstSpan<Vec2d> positions() const {
        return positions_;
    }

    /// Sets the position data of the centerline.
    ///
    /// Note that this `StrokeView2d` does not make a copy of the data.
    /// It is your responsibility to ensure that the data outlives this `StrokeView2d`.
    ///
    void setPositions(core::ConstSpan<Vec2d> positions);

    /// Returns the position of the start knot of the given segment.
    ///
    /// Throws `IndexError` if `segmentIndex` is not in the range `[0,
    /// numSegments() - 1]`.
    ///
    Vec2d segmentStartPosition(Int segmentIndex) const;

    /// Returns the position of the end knot of the given segment.
    ///
    /// Throws `IndexError` if `segmentIndex` is not in the range `[0,
    /// numSegments() - 1]`.
    ///
    Vec2d segmentEndPosition(Int segmentIndex) const;

    /// Returns the line-length of each segment, that is, the distance between
    /// the start knot and the end knot of the segment.
    ///
    /// This is different from the arclength of the segment, which is
    /// the length along the curve.
    ///
    core::ConstSpan<double> segmentLineLengths() const {
        return segmentLineLengths_;
    }

    /// Returns whether the given segment is a "segment corner", that
    /// is, a segment of length zero that behaves like a corner for the
    /// adjacent segments.
    ///
    /// Throws `IndexError` if `segmentIndex` is not in the range `[0,
    /// numSegments() - 1]`.
    ///
    bool isSegmentCorner(Int segmentIndex) const;

    /// Returns the AttributeVariability of the width attribute.
    ///
    AttributeVariability widthVariability() const {
        return widthVariability_;
    }

    /// Returns the width data of the curve.
    ///
    core::ConstSpan<double> widths() const {
        return widths_;
    }

    /// Sets the width data of the stroke.
    ///
    /// Note that this `StrokeView2d` does not make a copy of the data.
    /// It is your responsibility to ensure that the data outlives this `StrokeView2d`.
    ///
    void setWidths(core::ConstSpan<double> widths) {
        widths_ = widths;
        onWidthsChanged_();
    }

    /// Returns the width of the curve. If width is varying, then returns
    /// the average width;
    ///
    double width() const;

    /// Computes and returns a triangulation of this curve as a triangle strip
    /// [ p0, p1, ..., p_{2n}, p_{2n+1} ]. The even indices are on the "left"
    /// of the centerline, while the odd indices are on the "right", assuming a
    /// right-handed 2D coordinate system.
    ///
    /// Representing pairs of triangles as a single quad, it looks like this:
    ///
    /// \verbatim
    /// Y ^
    ///   |
    ///   o---> X
    ///
    /// p0  p2                                p_{2n}
    /// o---o-- ............................ --o
    /// |   |                                  |
    /// |   |    --- curve direction --->      |
    /// |   |                                  |
    /// o---o-- ............................ --o
    /// p1  p3                                p_{2n+1}
    /// \endverbatim
    ///
    /// Using the default parameters, the triangulation is "adaptive". This
    /// means that the number of quads generated between two control points
    /// depends on the curvature of the curve. The higher the curvature, the
    /// more quads are generated to ensure that consecutive quads never have an
    /// angle more than \p maxAngle (expressed in radian). This is what makes
    /// sure that the curve looks "smooth" at any zoom level.
    ///
    /// \verbatim
    ///                    _o p4
    ///                 _-` |
    /// p0        p2 _-`    |
    /// o----------o`       |
    /// |          |       _o p5
    /// |          |    _-`
    /// |          | _-` } maxAngle
    /// o----------o`- - - - - - - - -
    /// p1         p3
    /// \endverbatim
    ///
    /// In the case where the curve is a straight line between two control
    /// points, a single quad is enough. However, you can use use \p minQuads
    /// if you wish to have at least a certain number of quads uniformly
    /// generated between any two control points. Also, you can use \p maxQuads
    /// to limit how many quads are generated between any two control points.
    /// This is necessary to break infinite loops in case the curve contains a
    /// cusp between two control points.
    ///
    /// If you wish to uniformly generate a fixed number of quads between
    /// control points, simply set maxAngle to any value, and set minQuads =
    /// maxQuads = number of desired quads.
    ///
    Vec2dArray
    triangulate(double maxAngle = 0.05, Int minQuads = 1, Int maxQuads = 64) const;

    /// Computes a sampling of the subset of this curve consisting of
    /// `numSegments` segments starting at the knot at index `startKnot`.
    ///
    /// \verbatim
    /// INPUT
    /// -----
    /// startKnot   = 1
    /// numSegments = 2
    /// knots       = 0------1-----------2---------3---------4--------5
    ///                      |                     |
    ///                      |                     |
    ///                      |                     |
    ///                      |                     |
    /// OUTPUT               |                     |
    /// ------               v                     v
    /// samples     =        x-x-x-x-x-x-x-x-x-x-x-x
    /// \endverbatim
    ///
    /// The result is appended to the output parameter `outAppend`.
    ///
    /// The value of `startKnot` must be in the range [-m, m-1] with `m =
    /// numKnots()`. Negative values can be used for indexing from the end:
    /// `-1` represents the last knot, and `-m` represents the first knot.
    ///
    /// The value of `numSegments` must be in the range [-n-1, n] with `n =
    /// numSegments()`. Negative values can be used for specifying "all except
    /// k segments": `-1` represents all segments, and `-n-1` represents zero
    /// segments.
    ///
    /// This function throws `IndexError` if:
    /// - the curve is empty (`numKnots() == 0`), or
    /// - `startKnot` is not in the range [-m, m-1], or
    /// - `numSegments` is not in the range [-n-1, n], or
    /// - the curve is open and the requested number of segments (after wrapping
    /// negative values) is larger than the remaining number of segments when
    /// starting at `startKnot`. For example, if the curve has 4 knots and
    /// `startKnot == 1`, then the maximum value for `numSegments` is 2
    /// (segments from knot index 1 to knot index 3 which is the last knot).
    ///
    /// The start and end samples of the range are both included. This means
    /// that if this function does not throw, it is guaranteed to return a
    /// non-empty sampling (i.e., with at least one sample), even when
    /// the given `numSegments` is equal to zero.
    ///
    /// This also means that calling `sampleRange(out, params, 0, 1)` followed
    /// by `sampleRange(out, params, 1, 1)` would result in having two times
    /// the sample corresponding to knot index `1`. If you wish to do such chaining
    /// meaningfully, you have to manually discard the last point:
    ///
    /// ```cpp
    /// sampleRange(out, params, 0, 1);
    /// out.removeLast();
    /// sampleRange(out, params, 1, 1);
    /// ```
    ///
    /// If `withArclengths = true` (the default), then arclengths are computed
    /// starting from `s = 0` (if `outAppend` is initially empty) or `s =
    /// outAppend.last().s()` (if `outAppend` is not initially empty).
    ///
    /// If `withArclengths = false` (the default), then all arclengths of the
    /// computed samples are left uninitialized.
    ///
    /// If the curve is open and `numKnot() == 1`, this function returns a
    /// unique sample with a normal set to zero.
    ///
    void sampleRange(
        core::Array<StrokeSample2d>& outAppend,
        const CurveSamplingParameters& parameters,
        Int startKnot = 0,
        Int numSegments = -1,
        bool withArclengths = true) const;

    /// Returns the normalized tangents of the two offset lines at the given
    /// segment endpoint, given by its segment index and endpoint index (0 for
    /// the start of the segment, and 1 for the end of the segment).
    ///
    /// Note that the tangents just before and just after a knot are not
    /// necessarily equal in case of "corner" knots. Therefore,
    /// `getOffsetLineTangentsAtSegmentEndpoint(i - 1, 1)` and
    /// `getOffsetLineTangentsAtSegmentEndpoint(i, 0)` may not be equal.
    ///
    /// Throws `IndexError` if the given `segmentIndex` is not in the range
    /// `[0, numSegments() - 1]`.
    ///
    /// Throws `IndexError` if the given `endpointIndex` is neither `0` or `1`.
    ///
    std::array<geometry::Vec2d, 2>
    getOffsetLineTangentsAtSegmentEndpoint(Int segmentIndex, Int endpointIndex) const;

    /// Sets the color of the curve.
    ///
    // XXX Think aboutvariability for colors too. Does it make sense
    // to also have PerControlPoint variability? or only "SpatialLinear"?
    // Does it make sense to allow SpatialLinear for width too? and Circular?
    // It is a tradeoff between flexibility, supporting typical use case,
    // performance, and cognitive complexity (don't confuse users...). It
    // makes it complex for implementers too if too many features are supported.
    // For now, we only support constant colors, and postpone this discussion.
    //
    void setColor(const core::Color& color) {
        color_ = color;
    }

    /// Returns the color of the curve.
    ///
    core::Color color() const {
        return color_;
    }

private:
    // Representation of the centerline of the curve
    CurveType type_;
    core::ConstSpan<Vec2d> positions_ = {};

    bool isClosed_ = false;

    // Representation of the width of the curve
    AttributeVariability widthVariability_;
    core::ConstSpan<double> widths_ = {};
    double widthConstant_ = 0;
    double averageWidth_ = 0;
    //double maxWidth_ = 0;

    // Color of the curve
    core::Color color_;

    // Cached euclidean distances between knot positions.
    //
    // This should be recomputed either when the positions change,
    // or when the curve type changes (e.g., from open to close).
    //
    // Note: this assumes that the non-owned positions haven't changed
    // since the last call of setPositions, since this class cannot be
    // aware of changes of positions.
    //
    core::DoubleArray segmentLineLengths_;
    void computeSegmentLineLengths_();

    // Whether all segments are corner segments (includes the case
    // where numSegments() == 0).
    //
    bool areAllSegmentsCorners_ = false;

    void onWidthsChanged_();
};

} // namespace vgc::geometry

#endif // VGC_GEOMETRY_CURVE_H
