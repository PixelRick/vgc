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

#ifndef VGC_VACOMPLEX_EDGEGEOMETRY_H
#define VGC_VACOMPLEX_EDGEGEOMETRY_H

#include <array>
#include <memory>

#include <vgc/core/arithmetic.h>
#include <vgc/core/array.h>
#include <vgc/core/enum.h>
#include <vgc/core/flags.h>
#include <vgc/geometry/curve.h>
#include <vgc/geometry/mat3d.h>
#include <vgc/geometry/range1d.h>
#include <vgc/geometry/rect2d.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/vacomplex/api.h>
#include <vgc/vacomplex/edgesampling.h>
#include <vgc/vacomplex/cellgeometry.h>

// how to share edge shape correctly ?
// an inbetween edge that doesn't change should have the same shape for all times
// we also need edge shape source/def, which can be different curve types
// -> EdgeParameters ?

namespace vgc::vacomplex {

class KeyEdge;
class KeyEdgeGeometry;

class VGC_VACOMPLEX_API KeyHalfedgeGeometry {
public:
    KeyHalfedgeGeometry() noexcept = default;

    KeyHalfedgeGeometry(KeyEdgeGeometry* edgeGeometry, bool direction) noexcept
        : edgeGeometry_(edgeGeometry)
        , direction_(direction) {
    }

    KeyEdgeGeometry* edgeGeometry() const {
        return edgeGeometry_;
    }

    bool direction() const {
        return direction_;
    }

private:
    KeyEdgeGeometry* edgeGeometry_ = nullptr;
    bool direction_ = false;
};

/// \class vgc::vacomplex::KeyEdgeGeometry
/// \brief Authored model of the edge geometry.
///
/// It can be translated from dom or set manually.
///
// Dev Notes:
// Edge geometry is relative to end vertices position.
// We want to snap the source geometry in its own space when:
//    - releasing a dragged end vertex
//    - right before sculpting
//    - right before control point dragging
// We have to snap output geometry (sampling) when the source
// geometry is not already snapped (happens in many cases).
//
// In which space do we sample ?
// inbetweening -> common ancestor for best identification of interest points
//
class VGC_VACOMPLEX_API KeyEdgeGeometry final : public CellGeometry {
private:
    friend detail::Operations;
    friend KeyEdge;

public:
    KeyEdgeGeometry(bool isClosed)
        : isClosed_(isClosed) {
    }

    ~KeyEdgeGeometry() override = default;

    std::shared_ptr<KeyEdgeGeometry> clone() const {
        return std::static_pointer_cast<KeyEdgeGeometry>(clone_());
    }

    std::shared_ptr<KeyEdgeGeometry> createDefault() const {
        return std::static_pointer_cast<KeyEdgeGeometry>(createDefault_());
    }

    bool isClosed() const {
        return isClosed_;
    }

    //std::shared_ptr<KeyEdgeGeometry>
    //concat(bool direction, KeyEdgeGeometry* other, bool otherDirection) const;

    // IDEA: do conversion to common best stroke geometry to merge
    //       then match cell properties by pairs (use null if not present)

    /// Expects positions in object space.
    ///
    virtual geometry::StrokeSampling2d computeSampling(
        const geometry::CurveSamplingParameters& params,
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition,
        geometry::CurveSnapTransformationMode mode =
        geometry::CurveSnapTransformationMode::LinearInArclength) const = 0;

    virtual geometry::StrokeSampling2d
    computeSampling(const geometry::CurveSamplingParameters& params) const = 0;

    /// Expects delta in object space.
    ///
    virtual void translate(const geometry::Vec2d& delta) = 0;

    /// Expects transformation in object space.
    ///
    virtual void transform(const geometry::Mat3d& transformation) = 0;

    /// Expects positions in object space.
    ///
    virtual void snap(
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition,
        geometry::CurveSnapTransformationMode mode =
        geometry::CurveSnapTransformationMode::LinearInArclength) = 0;

    // We will later need a variant of computeSampling() that accepts a target
    // view matrix.
    // Ideally, for inbetweening we would like a sampling that is good in 2 spaces:
    // - the common ancestor group space for best morphing.
    // - the canvas space for best rendering.

    /// Returns the new position of the grabbed point (center of deformation falloff).
    ///
    // Note: choose properly between tolerance/samplingDelta/quality.
    // Todo: later add falloff kind, arclength/spatial, keep vertices.
    //
    virtual geometry::Vec2d sculptGrab(
        const geometry::Vec2d& startPosition,
        const geometry::Vec2d& endPosition,
        double radius,
        double strength,
        double tolerance,
        bool isClosed = false) = 0;

    /// Returns the position of the grabbed point (center of deformation falloff).
    ///
    // Note: choose properly between tolerance/samplingDelta/quality.
    // Todo: later add falloff kind, arclength/spatial, keep vertices.
    //
    virtual geometry::Vec2d sculptWidth(
        const geometry::Vec2d& position,
        double delta,
        double radius,
        double tolerance,
        bool isClosed = false) = 0;

    /// Returns the new position of the smooth point.
    ///
    // Todo: later add falloff kind, arclength/spatial.
    //
    virtual geometry::Vec2d sculptSmooth(
        const geometry::Vec2d& position,
        double radius,
        double strength,
        double tolerance,
        bool isClosed = false) = 0;

private:
    std::unique_ptr<geometry::AbstractStroke2d> stroke_;
    std::unique_ptr<geometry::AbstractStroke2d> editStroke_;

    bool isBeingEdited_() const {
        return editStroke_ != nullptr;
    }
    
    const bool isClosed_;

    //virtual std::shared_ptr<KeyEdgeGeometry>
    //concat_(const KeyHalfedgeGeometry& khg1, const KeyHalfedgeGeometry& khg2) const = 0;
    //
    //virtual void
    //assignAverageProperties_(core::Array<KeyHalfedgeGeometry> khgs) const = 0;
};

//std::shared_ptr<const EdgeSampling> snappedSampling_;
//virtual EdgeSampling computeSampling() = 0;

// key edge
//   geometry as pointer or type, but if it's a type it could be integrated to key edge...
//   if pointer then poly or inner pointer again ? poly is more efficient..

} // namespace vgc::vacomplex

#endif // VGC_VACOMPLEX_EDGEGEOMETRY_H
