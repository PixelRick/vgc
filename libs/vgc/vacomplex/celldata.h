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

#ifndef VGC_VACOMPLEX_CELLDATA_H
#define VGC_VACOMPLEX_CELLDATA_H

#include <map>
#include <memory> // std::unique_ptr, std::shared_ptr

#include <vgc/core/arithmetic.h>
#include <vgc/core/stringid.h>
#include <vgc/vacomplex/api.h>
#include <vgc/vacomplex/cellproperty.h>
#include <vgc/vacomplex/dataobject.h>

namespace vgc::vacomplex {

class Cell;
class CellData;
using CellDataPtr = std::shared_ptr<CellData>;

/// \class vgc::vacomplex::CellData
/// \brief Abstract authored data of a cell (geometry and properties).
///
class VGC_VACOMPLEX_API CellData : public DataObject,
                                   public std::enable_shared_from_this<CellData> {
protected:
    CellData() noexcept = default;

    virtual ~CellData() = default;

    CellData(const CellData& other);
    CellData(CellData&& other) noexcept;
    CellData& operator=(const CellData& rhs);
    CellData& operator=(CellData&& rhs) noexcept;

public:
    using PropertyMap = std::map<core::StringId, std::unique_ptr<CellProperty>>;

    const PropertyMap& properties() const {
        return properties_;
    }

    const CellProperty* findProperty(core::StringId name) const;

protected:
    void setProperty(std::unique_ptr<CellProperty>&& value);
    void removeProperty(core::StringId name);

    // XXX: additional argument when it is only an affine transformation ?
    void emitGeometryChanged() const;
    void emitPropertyChanged(core::StringId name) const;

    void assignClonedProperties(const CellData* other);

    void translateProperties(const geometry::Vec2d& delta);
    void transformProperties(const geometry::Mat3d& transformation);
    void updateProperties(const geometry::AbstractStroke2d* newStroke);
    void notifyPropertiesOfOperationEnd();

    void onCellDestroyed() {
        cell_ = nullptr;
    }

private:
    std::map<core::StringId, std::unique_ptr<CellProperty>> properties_;
    Cell* cell_ = nullptr;

    template<typename Op>
    bool doPropertyOperation(const Op& op) {
        bool changed = false;
        core::Array<core::StringId> toRemove;
        for (const auto& p : properties()) {
            switch (op(p.second.get())) {
            case CellProperty::OpResult::Success:
                emitPropertyChanged(p.first);
                changed = true;
                break;
            case CellProperty::OpResult::Unchanged:
                break;
            case CellProperty::OpResult::Unsupported:
                toRemove.append(p.first);
                changed = true;
                break;
            }
        }
        for (const core::StringId& name : toRemove) {
            removeProperty(name);
            emitPropertyChanged(name);
        }
        return changed;
    }
};

} // namespace vgc::vacomplex

#endif // VGC_VACOMPLEX_CELLDATA_H
