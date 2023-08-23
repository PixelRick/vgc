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

#include <vgc/vacomplex/cellgeometry.h>

#include <vgc/vacomplex/complex.h>
#include <vgc/vacomplex/detail/operationsimpl.h>

namespace vgc::vacomplex {

void CellGeometry::startEdit() {
    startEdit_();
}

void CellGeometry::resetEdit() {
    resetEdit_();
}

void CellGeometry::finishEdit() {
    finishEdit_();
}

void CellGeometry::abortEdit() {
    abortEdit_();
}

CellProperty* CellGeometry::findProperty(core::StringId name) const {
    auto it = properties_.find(name);
    return it != properties_.end() ? it->second.get() : nullptr;
}

void CellGeometry::setProperty(
    core::StringId name,
    std::unique_ptr<CellProperty>&& value) {

    properties_[name] = std::move(value);
}

void CellGeometry::emitGeometryChanged() const {
    if (cell_) {
        Complex* complex = cell_->complex();
        detail::Operations ops(complex);
        ops.onGeometryChanged_(cell_);
    }
}

void CellGeometry::emitPropertyChanged(core::StringId name) const {
    if (cell_) {
        Complex* complex = cell_->complex();
        detail::Operations ops(complex);
        ops.onPropertyChanged_(cell_, name);
    }
}

} // namespace vgc::vacomplex
