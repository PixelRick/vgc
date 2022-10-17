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

#ifndef VGC_DOM_PATH_H
#define VGC_DOM_PATH_H

#include <vgc/core/object.h>
#include <vgc/core/stringid.h>
#include <vgc/dom/api.h>
#include <vgc/dom/document.h>
#include <vgc/dom/element.h>

namespace vgc::dom {

namespace detail {

VGC_DOM_API extern const core::StringId relRoot;

} // namespace detail

/// \class vgc::dom::Path
/// \brief Represents a path to a node or attribute.
///
class Path {
public:
    Path(Element* element, core::StringId attributeName);
    Path(Document* document, std::string_view path);

    size_t hash() const noexcept {
        size_t res = 'PATH';
        std::hash<core::StringId> hasher = {};
        for (const core::StringId& sid : path_) {
            res ^= hasher(sid);
        }
    }

    std::string string() const noexcept;

    bool isRelative() const noexcept {
        return !path_.isEmpty() && path_.first() == detail::relRoot;
    }

private:
    core::Array<core::StringId> path_;
};

} // namespace vgc::dom

#endif // VGC_DOM_PATH_H
