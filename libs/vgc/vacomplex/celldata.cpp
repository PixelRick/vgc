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

#include <vgc/vacomplex/celldata.h>

#include <vgc/vacomplex/complex.h>
#include <vgc/vacomplex/detail/operationsimpl.h>

namespace vgc::vacomplex {

const CellProperty* CellData::findProperty(core::StringId name) const {
    auto it = properties_.find(name);
    return it != properties_.end() ? it->second.get() : nullptr;
}

void CellData::setProperty(std::unique_ptr<CellProperty>&& value) {
    // XXX: skip if equal ?
    core::StringId name = value->name();
    properties_[name] = std::move(value);
    emitPropertyChanged(name);
}

void CellData::removeProperty(core::StringId name) {
    properties_.erase(name);
    emitPropertyChanged(name);
}

void CellData::emitGeometryChanged() const {
    if (cell_) {
        Complex* complex = cell_->complex();
        detail::Operations ops(complex);
        ops.onGeometryChanged_(cell_);
    }
}

void CellData::emitPropertyChanged(core::StringId name) const {
    if (cell_) {
        Complex* complex = cell_->complex();
        detail::Operations ops(complex);
        ops.onPropertyChanged_(cell_, name);
    }
}

void CellData::assignClonedProperties(const CellData* other) {
    if (other == this) {
        return;
    }
    properties_.clear();
    for (const auto& [name, property] : other->properties_) {
        properties_[name] = property->clone();
    }
}

void CellData::translate(const geometry::Vec2d& delta) {
    for (const auto& p : properties_) {
        p.second->onTranslate_(delta);
    }
}

void CellData::transform(const geometry::Mat3d& transformation) {
    for (const auto& p : properties_) {
        p.second->onTransform_(transformation);
    }
}

void CellData::onOperationEnd() {
    for (const auto& p : properties_) {
        p.second->onOperationEnd_();
    }
}

} // namespace vgc::vacomplex
