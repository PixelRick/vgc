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

#ifndef VGC_VACOMPLEX_CELLPROPERTY_H
#define VGC_VACOMPLEX_CELLPROPERTY_H

#include <memory> // std::unique_ptr, std::shared_ptr

#include <vgc/core/arithmetic.h>
#include <vgc/core/stringid.h>
#include <vgc/geometry/stroke.h> // geometry::AbstractStroke2d
#include <vgc/vacomplex/api.h>
#include <vgc/vacomplex/dataobject.h>

namespace vgc::vacomplex {

class CellProperty;
class CellData;
class KeyEdgeData;

class VGC_VACOMPLEX_API DirectedCellProperty {
public:
    DirectedCellProperty() noexcept = default;

    DirectedCellProperty(CellProperty* property, bool direction) noexcept
        : property_(property)
        , direction_(direction) {
    }

    CellProperty* property() const {
        return property_;
    }

    bool direction() const {
        return direction_;
    }

private:
    CellProperty* property_ = nullptr;
    bool direction_ = false;
};

/// \class vgc::vacomplex::CellProperty
/// \brief Abstract authored property of a cell geometry.
///
class VGC_VACOMPLEX_API CellProperty {
protected:
    explicit CellProperty(core::StringId name)
        : name_(name) {
    }
    virtual ~CellProperty() = default;

public:
    enum class OpResult : UInt8 {
        Unsupported,
        Unchanged,
        Success
    };

    core::StringId name() const {
        return name_;
    }

    std::unique_ptr<CellProperty> clone() const {
        return clone_();
    }

private:
    core::StringId name_;

    friend CellData;
    friend KeyEdgeData;

    virtual std::unique_ptr<CellProperty> clone_() const = 0;

    // Returns OpResult::Unchanged by default.
    virtual OpResult onTranslate_(const geometry::Vec2d& delta);

    // Returns OpResult::Unchanged by default.
    virtual OpResult onTransform_(const geometry::Mat3d& transformation);

    // Returns a null pointer by default.
    virtual std::unique_ptr<CellProperty>
    onKeyEdgeConcat_(DirectedCellProperty hep1, DirectedCellProperty hep2) const;

    // Returns a null pointer by default.
    virtual std::unique_ptr<CellProperty>
    onKeyEdgeGlue_(core::Span<DirectedCellProperty> heps) const;

    // Returns OpResult::Unchanged by default.
    virtual OpResult onKeyEdgeSnap_(
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition);

    // Returns OpResult::Unchanged by default.
    virtual OpResult onKeyEdgeStrokeChanged_(
        geometry::AbstractStroke2d* oldStroke,
        geometry::AbstractStroke2d* newStroke);

    // Returns a null pointer by default.
    virtual std::unique_ptr<CellProperty>
    onKeyFaceGlue_(core::Span<CellProperty> heps) const;

    // Returns OpResult::Unchanged by default.
    virtual OpResult onOperationEnd_();
};

} // namespace vgc::vacomplex

#endif // VGC_VACOMPLEX_CELLPROPERTY_H
