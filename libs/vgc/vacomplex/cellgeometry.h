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

#ifndef VGC_VACOMPLEX_CELLGEOMETRY_H
#define VGC_VACOMPLEX_CELLGEOMETRY_H

#include <memory> // std::shared_ptr
#include <map>

#include <vgc/core/stringid.h>
#include <vgc/vacomplex/api.h>
#include <vgc/vacomplex/dataobject.h>

namespace vgc::vacomplex {

class Cell;

class VGC_VACOMPLEX_API CellProperty {
protected:
    explicit CellProperty(core::StringId name) : name_(name) {}
    virtual ~CellProperty() = default;

private:
    const core::StringId name_;
};

/// \class vgc::vacomplex::CellGeometry
/// \brief Abstract authored model of a cell geometry.
///
class VGC_VACOMPLEX_API CellGeometry : public DataObject,
                                       public std::enable_shared_from_this<CellGeometry> {
protected:
    CellGeometry() noexcept = default;

    virtual ~CellGeometry() = default;

    // non-copyable
    CellGeometry(const CellGeometry&) = delete;
    CellGeometry& operator=(const CellGeometry&) = delete;

public:
    // XXX: Concurrent use of the edit mode is bug-prone, we could replace this set
    //      of methods with a `startEdit()` that returns a non-copyable `EditGuard`.
    //      The latter would have `reset()` and `abort()` methods and do the equivalent of
    //      `finish()` on destruction. And at most one would be allowed to exist at all times.
    //
    void startEdit();
    void resetEdit();
    void finishEdit();
    void abortEdit();

    CellProperty* findProperty(core::StringId name) const;

protected:
    void setProperty(core::StringId name, std::unique_ptr<CellProperty>&& value);

    // XXX: additional argument when it is only an affine transformation ?
    void emitGeometryChanged() const;
    void emitPropertyChanged(core::StringId name) const;

    virtual std::shared_ptr<CellGeometry> clone_() const = 0;
    virtual std::shared_ptr<CellGeometry> createDefault_() const = 0;

private:
    std::map<core::StringId, std::unique_ptr<CellProperty>> properties_;
    Cell* cell_ = nullptr;

    virtual void startEdit_() = 0;
    virtual void resetEdit_() = 0;
    virtual void finishEdit_() = 0;
    virtual void abortEdit_() = 0;
};

} // namespace vgc::vacomplex

#endif // VGC_VACOMPLEX_CELLGEOMETRY_H
