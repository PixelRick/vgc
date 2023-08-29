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

#include <vgc/vacomplex/cellproperty.h>

#include <vgc/vacomplex/complex.h>
#include <vgc/vacomplex/detail/operationsimpl.h>

namespace vgc::vacomplex {

std::unique_ptr<CellProperty> CellProperty::concat_(
    const KeyHalfedgeData& /*khd1*/,
    const KeyHalfedgeData& /*khd2*/) const {

    return nullptr;
}

std::unique_ptr<CellProperty> CellProperty::glue_(
    core::ConstSpan<KeyHalfedgeData> /*khds*/,
    const geometry::AbstractStroke2d* /*gluedStroke*/) const {

    return nullptr;
}

std::unique_ptr<CellProperty>
CellProperty::glue_(core::ConstSpan<const KeyFaceData*> /*kfds*/) const {
    return nullptr;
}

CellProperty::OpResult CellProperty::onTranslate_(const geometry::Vec2d& /*delta*/) {
    return OpResult::Unchanged;
}

CellProperty::OpResult
CellProperty::onTransform_(const geometry::Mat3d& /*transformation*/) {
    return OpResult::Unchanged;
}

CellProperty::OpResult
CellProperty::onGeometryUpdate_(const geometry::AbstractStroke2d* /*newStroke*/) {
    return OpResult::Unchanged;
}

CellProperty::OpResult CellProperty::finalizeCombinedOperations_() {
    return OpResult::Unchanged;
}

CellProperties::CellProperties(const CellProperties& other) {
    assignClonedProperties_(other);
}

CellProperties::CellProperties(CellProperties&& other) noexcept
    : map_(std::move(other.map_)) {
}

CellProperties& CellProperties::operator=(const CellProperties& other) {
    if (&other != this) {
        assignClonedProperties_(other);
    }
    return *this;
}

CellProperties& CellProperties::operator=(CellProperties&& other) noexcept {
    if (&other != this) {
        map_ = std::move(other.map_);
    }
    return *this;
}

const CellProperty* CellProperties::find(core::StringId name) const {
    auto it = map_.find(name);
    return it != map_.end() ? it->second.get() : nullptr;
}

void CellProperties::insert(std::unique_ptr<CellProperty>&& value) {
    // XXX: skip if equal ?
    core::StringId name = value->name();
    map_[name] = std::move(value);
    emitPropertyChanged_(name);
}

void CellProperties::remove(core::StringId name) {
    if (map_.erase(name) > 0) {
        emitPropertyChanged_(name);
    }
}

void CellProperties::clear() {
    PropertyMap tmp = std::move(map_);
    for (const auto& it : tmp) {
        emitPropertyChanged_(it.first);
    }
}

namespace {

struct PropertyTemplate {
    core::StringId id;
    const CellProperty* prop;
};

} // namespace

void CellProperties::concat(
    CellProperties& result,
    const KeyHalfedgeData& khd1,
    const KeyHalfedgeData& khd2) const {

    result.clear();

    KeyEdgeData* ked1 = khd1.edgeData();
    KeyEdgeData* ked2 = khd2.edgeData();
    VGC_ASSERT(ked1 != nullptr && ked2 != nullptr);

    core::Array<PropertyTemplate> templates;
    for (const auto& p : ked1->properties().map()) {
        core::StringId id = p.first;
        if (!templates.search([id](const PropertyTemplate& p) { return p.id == id; })) {
            templates.append(PropertyTemplate{id, p.second.get()});
        }
    }
    for (const auto& p : ked2->properties().map()) {
        core::StringId id = p.first;
        if (!templates.search([id](const PropertyTemplate& p) { return p.id == id; })) {
            templates.append(PropertyTemplate{id, p.second.get()});
        }
    }

    for (const PropertyTemplate& p : templates) {
        std::unique_ptr<CellProperty> newProp = p.prop->concat_(khd1, khd2);
        if (newProp) {
            result.insert(std::move(newProp));
        }
    }
}

// Returns a null pointer by default.
void CellProperties::glue(
    CellProperties& result,
    core::ConstSpan<KeyHalfedgeData> khds,
    const geometry::AbstractStroke2d* gluedStroke) const {

    result.clear();

    core::Array<PropertyTemplate> templates;
    for (const KeyHalfedgeData& khd : khds) {
        KeyEdgeData* ked = khd.edgeData();
        for (const auto& p : ked->properties().map()) {
            core::StringId id = p.first;
            if (!templates.search(
                    [id](const PropertyTemplate& p) { return p.id == id; })) {
                templates.append(PropertyTemplate{id, p.second.get()});
            }
        }
    }

    for (const PropertyTemplate& p : templates) {
        std::unique_ptr<CellProperty> newProp = p.prop->glue_(khds, gluedStroke);
        if (newProp) {
            result.insert(std::move(newProp));
        }
    }
}

// Returns a null pointer by default.
void CellProperties::glue(
    CellProperties& result,
    core::ConstSpan<const KeyFaceData*> kfds) const {

    result.clear();

    // TODO
}

void CellProperties::finalizeCombinedOperations() {
    doOperation_([](CellProperty* p) { return p->finalizeCombinedOperations_(); });
}

void CellProperties::onTranslateGeometry(const geometry::Vec2d& delta) {
    doOperation_([&](CellProperty* p) { return p->onTranslate_(delta); });
}

void CellProperties::onTransformGeometry(const geometry::Mat3d& transformation) {
    doOperation_([&](CellProperty* p) { return p->onTransform_(transformation); });
}

void CellProperties::onUpdateGeometry(const geometry::AbstractStroke2d* newStroke) {
    doOperation_([&](CellProperty* p) { return p->onGeometryUpdate_(newStroke); });
}

void CellProperties::assignClonedProperties_(const CellProperties& other) {
    if (&other != this) {
        map_.clear();
        for (const auto& [name, prop] : other.map_) {
            map_[name] = prop->clone();
        }
    }
}

/* static */
void CellProperties::emitPropertyChanged_(core::StringId name) {
    if (cell_) {
        Complex* complex = cell_->complex();
        detail::Operations ops(complex);
        ops.onPropertyChanged_(cell_, name);
    }
}

} // namespace vgc::vacomplex
