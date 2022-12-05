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

#ifndef VGC_TOPOLOGY_CELL_H
#define VGC_TOPOLOGY_CELL_H

#include <optional>

#include <vgc/core/animtime.h>
#include <vgc/core/id.h>
#include <vgc/geometry/curve.h>
#include <vgc/geometry/mat2d.h>
#include <vgc/geometry/mat3d.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/topology/api.h>
#include <vgc/topology/edgegeometry.h>

namespace vgc::topology {

namespace detail {

class Operations;

} // namespace detail

VGC_DECLARE_OBJECT(Vgc);

class VgcCell;

class Vertex;
class Edge;
class Face;

VGC_DECLARE_OBJECT(Vac);

class VacNode;
class VacGroup;
class VacCell;

class VertexCell;
class EdgeCell;
class FaceCell;

class KeyCell;
class KeyVertex;
class KeyEdge;
class KeyFace;

class InbetweenCell;
class InbetweenVertex;
class InbetweenEdge;
class InbetweenFace;

class EdgeGeometry;
class FaceGeometry;

/// \enum vgc::graph::CellSpatialType
/// \brief Specifies the spatial type of a Cell.
// clang-format off
enum class CellSpatialType : UInt8 {
    Vertex = 0,
    Edge   = 1,
    Face   = 2,
};
// clang-format on

/// \enum vgc::graph::VacCellType
/// \brief Specifies the type of a Vac Cell.
// clang-format off
enum class VacCellType : UInt8 {
    KeyVertex       = 0,
    KeyEdge         = 1,
    KeyFace         = 2,
    // 3 is skipped for bit masking
    InbetweenVertex = 4,
    InbetweenEdge   = 5,
    InbetweenFace   = 6,
};
// clang-format on

namespace detail {

inline constexpr CellSpatialType vacCellTypeToSpatialType(VacCellType x) {
    return static_cast<CellSpatialType>(core::toUnderlying(x) & 3);
}

inline constexpr bool isVacKeyCellType(VacCellType x) {
    return (core::toUnderlying(x) >> 2) == 0;
}

// Affine 2D transform for Vec2d.
// XXX move to geometry
class Transform2d {
    geometry::Vec2d operator*(const geometry::Vec2d& v) const {
        return rotationScale_ * v + translation_;
    }

private:
    geometry::Vec2d translation_ = {};
    geometry::Mat2d rotationScale_ = geometry::Mat2d::identity;
};

} // namespace detail

class VGC_TOPOLOGY_API VacNode {
private:
    friend detail::Operations;

protected:
    VacNode(core::Id id) noexcept
        : id_(id)
        , cellType_(std::nullopt) {
    }

    VacNode(core::Id id, VacCellType cellType) noexcept
        : id_(id)
        , cellType_(cellType) {
    }

public:
    virtual ~VacNode() = default;

    VacNode(const VacNode&) = delete;
    VacNode& operator=(const VacNode&) = delete;

    VacGroup* parentGroup() const {
        return parentGroup_;
    }

    core::Id id() const {
        return id_;
    }

    bool isCell() const {
        return cellType_.has_value();
    }

    bool isGroup() const {
        return !isCell();
    }

protected:
    VacCellType cellTypeUnchecked() const {
        return *cellType_;
    }

private:
    friend Vac;

    VacGroup* parentGroup_ = nullptr;
    core::Id id_ = -1;
    const std::optional<VacCellType> cellType_;
};

class VGC_TOPOLOGY_API VacGroup : public VacNode {
private:
    friend detail::Operations;

public:
    ~VacGroup() override = default;

    explicit VacGroup(core::Id id) noexcept
        : VacNode(id) {
    }

    // Root Node constructor.
    explicit VacGroup(Vac* vac) noexcept
        : VacNode(0)
        , vac_(vac) {
    }

    Vac* vac() const {
        return vac_;
    }

    /// Returns children in depth order from bottom to top.
    ///
    const core::Array<VacNode*>& children() const {
        return children_;
    }

    Int numChildren() const {
        return children_.length();
    }

    const geometry::Mat3d& transform() const {
        return transform_;
    }

    const geometry::Mat3d& inverseTransform() const {
        return inverseTransform_;
    }

    const geometry::Mat3d& transformFromRoot() const {
        return transformFromRoot_;
    }

    geometry::Mat3d computeInverseTransformTo(VacGroup* ancestor) const;

    geometry::Mat3d computeInverseTransformToRoot() const {
        return computeInverseTransformTo(nullptr);
    }

protected:
    void updateTransformFromRoot();

    void setTransform(const geometry::Mat3d& transform);

private:
    friend Vac;
    friend detail::Operations;

    Vac* vac_ = nullptr;
    core::Array<VacNode*> children_;
    geometry::Mat3d transform_;
    // to speed-up working with cells connected from different groups
    geometry::Mat3d inverseTransform_;
    geometry::Mat3d transformFromRoot_;
};

// boundaries:
//  key vertex  -> none
//  key edge    -> 2 key vertices
//  key face    -> N key vertices, key edges
//  ib vertex   -> 2 key vertices
//  ib edge     -> N key vertices, ib vertices, key edges
//  ib face     -> N key faces, ib edges
//
// stars:
//  key vertex  -> ib vertices, key edges, ib edges, key faces, ib faces
//  key edge    ->
//  key face    ->
//  ib vertex   ->
//  ib edge     ->
//  ib face     ->
//
// additional repr:
//  key face    -> cycle    = half key edges
//  ib edge     -> path     = half key edges
//              -> animvtx  = ib vertices
//  ib face     -> animcycl = planar graph with key verts, ib verts, key edges, ib edges
//

class VGC_TOPOLOGY_API VacCell : public VacNode {
private:
    friend detail::Operations;

    friend KeyCell;
    friend InbetweenCell;

    friend VertexCell;
    friend EdgeCell;
    friend FaceCell;

    VacCell()
        : VacNode(-1, VacCellType::KeyVertex) {

        throw core::RuntimeError(
            "This constructor should not be called. It is only defined to be used "
            "as unused default constructor in the context of virtual inheritance.");
    }

protected:
    VacCell(core::Id id, VacCellType cellType) noexcept
        : VacNode(id, cellType) {
    }

public:
    ~VacCell() override = default;

    // XXX move to cpp
    Vac* vac() const {
        VacGroup* p = parentGroup();
        return p ? p->vac() : nullptr;
    }

    /// Returns the cell type of this `VacCell`.
    ///
    VacCellType cellType() const {
        return cellTypeUnchecked();
    }

    CellSpatialType spatialType() const {
        return detail::vacCellTypeToSpatialType(cellType());
    }

    bool isKeyCell() const {
        return detail::isVacKeyCellType(cellType());
    }

    bool isInbetweenCell() const {
        return !isKeyCell();
    }

    virtual bool existsAt(core::AnimTime t) const = 0;

private:
    core::Array<VacCell*> star_;
};

class VGC_TOPOLOGY_API KeyCell : virtual public VacCell {
private:
    friend detail::Operations;

protected:
    KeyCell() = default;

    explicit KeyCell(const core::AnimTime& time) noexcept
        : time_(time) {
    }

public:
    constexpr core::AnimTime time() const {
        return time_;
    }

    bool existsAt(core::AnimTime t) const override {
        return t == time_;
    }

private:
    core::AnimTime time_;
};

class VGC_TOPOLOGY_API InbetweenCell : virtual public VacCell {
private:
    friend detail::Operations;

protected:
    InbetweenCell() = default;

public:
    // virtual api

    bool existsAt(core::AnimTime /*t*/) const override {
        // XXX todo
        return false;
    }
};

class VGC_TOPOLOGY_API VertexCell : virtual public VacCell {
private:
    friend detail::Operations;

protected:
    VertexCell() = default;

public:
    virtual geometry::Vec2d position(core::AnimTime t) const = 0;
};

class VGC_TOPOLOGY_API EdgeCell : virtual public VacCell {
private:
    friend detail::Operations;

protected:
    EdgeCell() = default;

public:
    // virtual api

    // note: Looks best to return an object so that we can change its impl
    //       if we want to share the data. The straight forward implementation
    //       is to not cache this result in the cell, otherwise we'd have to manage
    //       a cache array in inbetween cells.
    //virtual EdgeGeometry computeSamplingAt(core::AnimTime /*t*/) = 0;
};

class VGC_TOPOLOGY_API FaceCell : virtual public VacCell {
private:
    friend detail::Operations;

protected:
    FaceCell() = default;

public:
    // virtual api
};

class VGC_TOPOLOGY_API VertexUsage {
    // def depends on how we'll represent keyface boundaries
};

} // namespace vgc::topology

#endif // VGC_TOPOLOGY_CELL_H
