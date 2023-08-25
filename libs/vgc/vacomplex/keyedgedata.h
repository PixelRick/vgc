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
#include <vgc/geometry/mat3d.h>
#include <vgc/geometry/stroke.h> // AbstractStroke2d
#include <vgc/vacomplex/api.h>
#include <vgc/vacomplex/celldata.h>

// how to share edge shape correctly ?
// an inbetween edge that doesn't change should have the same shape for all times
// we also need edge shape source/def, which can be different curve types
// -> EdgeParameters ?

namespace vgc::vacomplex {

class KeyEdge;
class KeyEdgeData;

namespace detail {

class Operations;

}

class VGC_VACOMPLEX_API KeyHalfedgeGeometry {
public:
    KeyHalfedgeGeometry() noexcept = default;

    KeyHalfedgeGeometry(KeyEdgeData* edgeGeometry, bool direction) noexcept
        : edgeGeometry_(edgeGeometry)
        , direction_(direction) {
    }

    KeyEdgeData* edgeGeometry() const {
        return edgeGeometry_;
    }

    bool direction() const {
        return direction_;
    }

private:
    KeyEdgeData* edgeGeometry_ = nullptr;
    bool direction_ = false;
};

/// \class vgc::vacomplex::KeyEdgeData
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
class VGC_VACOMPLEX_API KeyEdgeData final : public CellData {
private:
    friend detail::Operations;
    friend KeyEdge;

public:
    KeyEdgeData(bool isClosed)
        : isClosed_(isClosed) {
    }

    ~KeyEdgeData() override = default;

    std::shared_ptr<KeyEdgeData> clone() const {
        return std::static_pointer_cast<KeyEdgeData>(clone_());
    }

    std::shared_ptr<KeyEdgeData> createDefault() const {
        return std::static_pointer_cast<KeyEdgeData>(createDefault_());
    }

    bool isClosed() const {
        return isClosed_;
    }

    /// Expects delta in object space.
    ///
    void translate(const geometry::Vec2d& delta);

    /// Expects transformation in object space.
    ///
    void transform(const geometry::Mat3d& transformation);

    /// Expects positions in object space.
    ///
    void snap(
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition,
        geometry::CurveSnapTransformationMode mode =
            geometry::CurveSnapTransformationMode::LinearInArclength);

    const geometry::AbstractStroke2d* stroke() const;

    void setStroke(geometry::AbstractStroke2d* stroke);

    //std::shared_ptr<KeyEdgeData>
    //concat(bool direction, KeyEdgeData* other, bool otherDirection) const;
    //
    //virtual std::shared_ptr<KeyEdgeData>
    //concat_(const KeyHalfedgeGeometry& khg1, const KeyHalfedgeGeometry& khg2) const = 0;
    //
    //virtual void
    //assignAverageProperties_(core::Array<KeyHalfedgeGeometry> khgs) const = 0;
    //
    // IDEA: do conversion to common best stroke geometry to merge
    //       then match cell properties by pairs (use null if not present)

private:
    std::unique_ptr<geometry::AbstractStroke2d> stroke_;
    std::unique_ptr<geometry::AbstractStroke2d> oldStroke_;

    bool isBeingEdited_() const {
        return oldStroke_ != nullptr;
    }

    const bool isClosed_;
};

//std::shared_ptr<const EdgeSampling> snappedSampling_;
//virtual EdgeSampling computeSampling() = 0;

// key edge
//   geometry as pointer or type, but if it's a type it could be integrated to key edge...
//   if pointer then poly or inner pointer again ? poly is more efficient..

} // namespace vgc::vacomplex

#endif // VGC_VACOMPLEX_EDGEGEOMETRY_H
