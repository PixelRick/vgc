// Copyright 2021 The VGC Developers
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

#include <vgc/dom/path.h>

#include <vgc/dom/strings.h>

namespace vgc::dom {

namespace detail {

const core::StringId RootPath("relRoot");

} // namespace detail

namespace {

const core::StringId nameAttrName("name");

}

Path::Path(Element* element, core::StringId attributeName) {
    path_.append(attributeName);
    while (element) {
        Element* parentElement = Element::cast(element->parent());
        if (!parentElement) {
            // root element "vgc"
            break;
        }
        const Value& v = element->getAuthoredAttribute(nameAttrName);
        if (v.type() == ValueType::String) {
            path_.append(core::StringId(v.getString()));
        }
        else {
            Element* e = parentElement->firstChildElement();
            Int i = 0;
            while (e && e != element) {
                e = e->nextSiblingElement();
                ++i;
            }
            path_.append(core::StringId(std::to_string(i)));
        }
        element = parentElement;
    }
    std::reverse(path_.begin(), path_.end());
}

Path::Path(Document* document, std::string_view path) {
    // parse
}

size_t Path::hash() const noexcept {
    size_t res = 'PATH';
    std::hash<core::StringId> hasher = {};
    for (const core::StringId& sid : path_) {
        res ^= hasher(sid);
    }
}

std::string Path::string() const noexcept {

}

} // namespace vgc::dom
