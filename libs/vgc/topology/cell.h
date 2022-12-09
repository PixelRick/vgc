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
#include <type_traits>

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

template<typename Derived, typename Child>
class TreeParentBase;

template<typename Derived, typename Parent>
class TreeChildBase;

template<typename T>
class TreeChildrenIterator {
public:
    using difference_type = Int;
    using value_type = T;
    using reference = T&;
    using pointer = T*;
    using iterator_category = std::forward_iterator_tag;

    constexpr TreeChildrenIterator() noexcept = default;

    explicit TreeChildrenIterator(T* p)
        : p_(p) {
    }

    TreeChildrenIterator& operator++() {
        p_ = p_->next();
        return *this;
    }

    TreeChildrenIterator operator++(int) {
        TreeChildrenIterator cpy(*this);
        operator++();
        return cpy;
    }

    T& operator*() const {
        return *p_;
    }

    T* operator->() const {
        return p_;
    }

    bool operator==(const TreeChildrenIterator& other) const {
        return p_ == other.p_;
    }

    bool operator!=(const TreeChildrenIterator& other) const {
        return p_ != other.p_;
    }

private:
    T* p_ = nullptr;
};

// non-owning-tree child
// nothing is done automatically on destruction
template<typename Child, typename Parent = Child>
class TreeChildBase {
private:
    friend TreeParentBase<Parent, Child>;
    friend TreeChildrenIterator<TreeChildBase>;

protected:
    Child* previous() const {
        return previous_;
    }

    Child* next() const {
        return next_;
    }

    Parent* parent() const {
        return parent_;
    }

    void unlink();

private:
    Child* previous_ = nullptr;
    Child* next_ = nullptr;
    Parent* parent_ = nullptr;
};

// non-owning-tree parent
// nothing is done automatically on destruction
template<typename Parent, typename Child = Parent>
class TreeParentBase {
private:
    friend TreeChildBase<Child, Parent>;

    using ChildBase = TreeChildBase<Child, Parent>;

protected:
    using Iterator = TreeChildrenIterator<Child>;

    constexpr TreeParentBase() noexcept {
        static_assert(std::is_base_of_v<ChildBase, Child>);
        static_assert(std::is_base_of_v<ChildBase, Parent>);
    }

    Child* firstChild() const {
        return firstChild_;
    }

    Child* lastChild() const {
        return lastChild_;
    }

    Iterator begin() const {
        return Iterator(firstChild_);
    }

    Iterator end() const {
        return Iterator();
    }

    Int numChildren() const {
        return numChildren_;
    }

    void resetNoUnlink() {
        numChildren_ = 0;
        firstChild_ = nullptr;
        lastChild_ = nullptr;
    }

    void appendChild(Child* x) {
        insertChildUnchecked(nullptr, x);
    }

    // assumes nextSibling is nullptr or a child of this
    void insertChildUnchecked(Child* nextSibling, Child* x) {

        Child* const newNext = nextSibling;
        if (x == newNext) {
            return;
        }

        Child* const newPrevious = newNext ? newNext->previous_ : lastChild_;
        if (x == newPrevious) {
            return;
        }

        Parent* const oldParent = x->parent_;
        Child* const oldPrevious = x->previous_;
        Child* const oldNext = x->next_;

        if (oldPrevious) {
            oldPrevious->next_ = oldNext;
        }
        else if (oldParent) {
            oldParent->firstChild_ = oldNext;
        }

        if (oldNext) {
            oldNext->previous_ = oldPrevious;
        }
        else if (oldParent) {
            oldParent->lastChild_ = oldPrevious;
        }

        if (newPrevious) {
            newPrevious->next_ = x;
        }
        else {
            firstChild_ = x;
        }

        if (newNext) {
            newNext->previous_ = x;
        }
        else {
            lastChild_ = x;
        }

        x->previous_ = newPrevious;
        x->next_ = newNext;

        if (oldParent != this) {
            x->parent_ = static_cast<Parent*>(this);
            ++numChildren_;
            if (oldParent) {
                --(oldParent->numChildren_);
            }
        }
    }

private:
    Child* firstChild_ = nullptr;
    Child* lastChild_ = nullptr;
    Int numChildren_ = 0;
};

template<typename Derived, typename Parent>
void TreeChildBase<Derived, Parent>::unlink() {

    using ChildBase = TreeChildBase<Derived, Parent>;

    Parent* const oldParent = parent_;
    Derived* const oldPrevious = previous_;
    Derived* const oldNext = next_;

    if (oldPrevious) {
        oldPrevious->next_ = oldNext;
        previous_ = nullptr;
    }
    else if (oldParent) {
        oldParent->firstChild_ = oldNext;
    }

    if (oldNext) {
        oldNext->previous_ = oldPrevious;
        next_ = nullptr;
    }
    else if (oldParent) {
        oldParent->lastChild_ = oldPrevious;
    }

    if (oldParent) {
        --(oldParent->numChildren_);
    }
}

} // namespace detail

class VGC_TOPOLOGY_API VacNode : public detail::TreeChildBase<VacNode, VacGroup> {
private:
    friend detail::Operations;

    using TreeChildBase = detail::TreeChildBase<VacNode, VacGroup>;
    using Iterator = detail::TreeChildrenIterator<VacNode>;

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

    VacNode* previous() const {
        return TreeChildBase::previous();
    }

    VacNode* next() const {
        return TreeChildBase::next();
    }

    VacGroup* parentGroup() const {
        return TreeChildBase::parent();
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

    core::Id id_ = -1;
    const std::optional<VacCellType> cellType_;
};

class VGC_TOPOLOGY_API VacGroup : public VacNode,
                                  public detail::TreeParentBase<VacGroup, VacNode> {
private:
    friend detail::Operations;

    using TreeParentBase = detail::TreeParentBase<VacGroup, VacNode>;

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

    /// Returns bottom-most child in depth order.
    ///
    VacNode* firstChild() const {
        return TreeParentBase::firstChild();
    }

    /// Returns top-most child in depth order.
    ///
    VacNode* lastChild() const {
        return TreeParentBase::lastChild();
    }

    Int numChildren() const {
        return TreeParentBase::numChildren();
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

private:
    friend Vac;
    friend detail::Operations;

    Vac* vac_ = nullptr;

    geometry::Mat3d transform_;
    // to speed-up working with cells connected from different groups
    geometry::Mat3d inverseTransform_;
    geometry::Mat3d transformFromRoot_;

    void onChildrenDestroyed() {
        TreeParentBase::resetNoUnlink();
    }

    void setTransform_(const geometry::Mat3d& transform);
    void updateTransformFromRoot_();
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
