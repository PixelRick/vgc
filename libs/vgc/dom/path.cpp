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

#include <array>
#include <charconv>

#include <vgc/dom/logcategories.h>
#include <vgc/dom/strings.h>

namespace vgc::dom {

namespace detail {} // namespace detail

namespace {

const core::StringId nameAttrName("name");

}

explicit PathSegment::PathSegment(
    core::StringId name,
    PathSegmentFlags flags = PathSegmentFlag::Element,
    ArrayIndexType arrayIndex = 0)

    : name_(name)
    , arrayIndex_(arrayIndex)
    , flags_(flags) {
}

namespace {

bool appendElementReversePath(core::Array<PathSegment>& segments, Element* element) {
    while (element) {
        Element* parentElement = Element::cast(element->parent());
        if (!parentElement) {
            // root element
            break;
        }
        const Value& v = element->getAuthoredAttribute(nameAttrName);
        if (v.type() == ValueType::String) {
            segments.emplaceLast(core::StringId(v.getString()));
        }
        else {
            VGC_ERROR(
                LogVgcDom,
                "Failed to create dom::Path with element <{}> as segment since it has no "
                "attribute \"name\" and thus cannot be uniquely identified.",
                element->name());
            return false;
            //
            // possible fallback with tagName[index]:
            //
            //    Element* e = parentElement->firstChildElement();
            //    Int i = 0;
            //    while (e && e != element) {
            //        e = e->nextSiblingElement();
            //        ++i;
            //    }
            //    segments.append(core::StringId(std::to_string(i)));
        }
        element = parentElement;
    }
}

bool isReservedChar(char c) {
    constexpr std::string_view reserved = "/.#[]";
    return reserved.find(c) != std::string::npos;
}

size_t findReservedChar(std::string_view path, size_t start) {
    constexpr std::string_view reserved = "/.#[]";
    const size_t n = path.size();
    size_t i = start;
    while (i < n && !isReservedChar(path[i])) {
        ++i;
    }
    return i;
}

} // namespace

Path::Path(Element* element) {
    if (!appendElementReversePath(segments_, element)) {
        segments_.clear();
        return;
    }
    std::reverse(segments_.begin(), segments_.end());
}

Path::Path(Element* element, core::StringId attributeName)
    : Path(element) {

    segments_.emplaceLast(attributeName, PathSegmentFlag::Attribute);
}

Path::Path(
    Element* element,
    core::StringId attributeName,
    PathSegment::ArrayIndexType arrayIndex)

    : Path(element) {

    segments_.emplaceLast(attributeName, PathSegmentFlag::Attribute);
}

Path::Path(std::string_view path) {
    const size_t n = path.size();
    size_t i = 0;
    size_t j = 0;

    // parse first element
    if (n == 0) {
        segments_.emplaceLast(core::StringId(), PathSegmentFlag::Root);
        return;
    }
    else if (path[0] == '/') {
        // full path
        segments_.emplaceLast(core::StringId(), PathSegmentFlag::Root);
        // don't increment i here, elements are expected to be preceeded by '/' and '/.attr' is no valid.
    }
    else if (path[0] == '#') {
        // based path
        ++i;
        j = findReservedChar(path, i);
        if (j == i) {
            VGC_ERROR(
                LogVgcDom, "Empty unique name (starts with '#') in path: \"{}\".", path);
            segments_.clear();
            return;
        }
        segments_.emplaceLast(
            core::StringId(path.substr(i, j - i)), PathSegmentFlag::Element);
        i = j;
    }
    else if (!isReservedChar(path[0])) {
        // relative path
        segments_.emplaceLast(core::StringId(), PathSegmentFlag::Dot);
        j = findReservedChar(path, i + 1);
        segments_.emplaceLast(
            core::StringId(path.substr(i, j - i)), PathSegmentFlag::Element);
        i = j;
    }

    while (i < n && path[i] == '/') {
        ++i;
        j = findReservedChar(path, i);
        if (j == i) {
            VGC_ERROR(LogVgcDom, "Empty element name in path: \"{}\".", path);
            segments_.clear();
            return;
        }
        segments_.emplaceLast(
            core::StringId(path.substr(i, j - i)), PathSegmentFlag::Element);
        i = j;
    }

    if (i < n && path[i] == '.') {
        ++i;
        j = findReservedChar(path, i);
        if (j == i) {
            VGC_ERROR(LogVgcDom, "Empty attribute name in path: \"{}\".", path);
            segments_.clear();
            return;
        }
        std::string_view attrName = path.substr(i, j - i);
        i = j;

        if (i < n && path[i] == '[') {
            ++i;
            j = findReservedChar(path, i);
            if (j == n || path[i] != ']') {
                VGC_ERROR(LogVgcDom, "Expected ']' after index in path: \"{}\".", path);
                segments_.clear();
                return;
            }
            if (j == i) {
                VGC_ERROR(LogVgcDom, "Empty index in path: \"{}\".", path);
                segments_.clear();
                return;
            }

            size_t index = 0;
            auto result = std::from_chars(&path[i], &path[j], index);
            if (result.ec == std::errc::invalid_argument) {
                VGC_ERROR(LogVgcDom, "Invalid index format in path: \"{}\".", path);
                segments_.clear();
                return;
            }

            segments_.emplaceLast(
                core::StringId(attrName), PathSegmentFlag::Attribute, index);
            i = j;
        }
        else {
            segments_.emplaceLast(core::StringId(attrName), PathSegmentFlag::Attribute);
        }
    }

    if (i != n) {
        VGC_ERROR(
            LogVgcDom,
            "Unexpected extra characters after end of attribute identifier in path: "
            "\"{}\".",
            path);
        segments_.clear();
        return;
    }
}

std::string Path::string() const noexcept {
    if (segments_.isEmpty()) {
        return "";
    }

    std::string ret = {};

    bool skipSlash = true;
    Int i = 0;

    const PathSegment& seg0 = segments_[0];
    PathSegmentFlags flags0 = seg0.flags_;
    if (flags0.has(PathSegmentFlag::Dot)) {
        ++i;
    }
    else if (flags0.has(PathSegmentFlag::Root)) {
        ret.append(1, '/');
        ++i;
    }

    const Int n = segments_.length();
    for (; i < n; ++i) {
        const PathSegment& seg = segments_[i];
        if (flags0.has(PathSegmentFlag::Element)) {
            if (skipSlash) {
                skipSlash = false;
            }
            else {
                ret.append(1, '/');
            }
            ret.append(seg.name());
        }
        else if (flags0.has(PathSegmentFlag::Attribute)) {
            ret.append(1, '.');
            ret.append(seg.name());
            if (flags0.has(PathSegmentFlag::Indexed)) {
                ret.append(core::format("[{}]", seg.arrayIndex()));
            }
        }
        else {
            VGC_ERROR(
                LogVgcDom,
                "Could not convert dom::Path to string, it contains unexpected "
                "segments.");
            return "";
        }
    }

    return ret;
}

Path Path::elementPath() const {
    Path ret = {};
    for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
        if (!it->flags().has(PathSegmentFlag::Attribute)) {
            ret.segments_.assign(segments_.begin(), it.base());
        }
    }
    return ret;
}

Path Path::elementRelativeAttributePath() const {
    Path ret = {};
    for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
        if (!it->flags().has(PathSegmentFlag::Attribute)) {
            ret.segments_.assign(it.base(), segments_.end());
        }
    }
    return ret;
}

Path& Path::removeAttribute() {
    for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
        if (!it->flags().has(PathSegmentFlag::Attribute)) {
            segments_.erase(segments_.begin(), it.base());
        }
    }
    return *this;
}

Path& Path::removeIndex() {
    if (!segments_.isEmpty()) {
        const PathSegment& seg = segments_.last();
        seg.flags().unset(PathSegmentFlag::Indexed);
    }
}

} // namespace vgc::dom
