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

#include <vgc/core/flags.h>
#include <vgc/core/object.h>
#include <vgc/core/stringid.h>
#include <vgc/dom/api.h>
#include <vgc/dom/document.h>
#include <vgc/dom/element.h>

namespace vgc::dom {

class Path;

namespace detail {

template<typename T>
void hashCombine(std::size_t& res, const T& v) {
    res ^= std::hash<T>()(v) + 0x9E3779B9 + (res << 6) + (res >> 2);
}

template<typename... Ts>
std::enable_if_t<(sizeof...(Ts) > 1), void> hashCombine(std::size_t& res, Ts... values) {
    (hashCombine(res, values), ...);
}

} // namespace detail

/*
path examples:
/layer/rect.v[0] = "/layer/rect/v0" if real
    element: /layer/rect
    attribute: v
    isIndexed: true
    arrayIndex: 0
/layer/curve.startVertex
    element: /layer/curve
    attribute: startVertex
    isIndexed: false
    arrayIndex: 0
*/

/// \enum vgc::dom::PathSegmentType
/// \brief Specifies the type of a path segment.
///
// clang-format off
enum class PathSegmentFlag : UInt16 {
    None        = 0x00,
    Root        = 0x01, // only allowed as first segment
    UniqueId    = 0x02, // only allowed as first segment
    Dot         = 0x04, // only allowed as first segment
    Element     = 0x08,
    Attribute   = 0x10,
    Indexed     = 0x20, // only allowed for attributes atm
};
// clang-format on

VGC_DEFINE_FLAGS(PathSegmentFlags, PathSegmentFlag)

/// \class vgc::dom::PathSegment
/// \brief Represents a path segment.
///
/// It can be an `Element` name or an attribute name with an optional index.
///
class VGC_DOM_API PathSegment {
private:
    friend Path;

public:
    using ArrayIndexType = Int;

    // Constructs a segment representing the root element.
    constexpr PathSegment() noexcept = default;

    explicit PathSegment(
        core::StringId name,
        PathSegmentFlags flags = PathSegmentFlag::Element,
        ArrayIndexType arrayIndex = 0) noexcept;

    core::StringId name() const noexcept {
        return name_;
    }

    PathSegmentFlags flags() const noexcept {
        return flags_;
    }

    bool isRoot() const noexcept {
        return flags_.has(PathSegmentFlag::Root);
    }

    bool isUniqueId() const noexcept {
        return flags_.has(PathSegmentFlag::UniqueId);
    }

    bool isDot() const noexcept {
        return flags_.has(PathSegmentFlag::Dot);
    }

    bool isElement() const noexcept {
        return flags_.has(PathSegmentFlag::Element);
    }

    bool isAttribute() const noexcept {
        return flags_.has(PathSegmentFlag::Attribute);
    }

    bool isIndexed() const noexcept {
        return flags_.has(PathSegmentFlag::Indexed);
    }

    ArrayIndexType arrayIndex() const noexcept {
        return arrayIndex_;
    }

    size_t hash() const noexcept {
        size_t res = 'PSEG';
        detail::hashCombine(res, name_, flags_);
        if (flags_.has(PathSegmentFlag::Indexed)) {
            detail::hashCombine(res, arrayIndex_);
        }
        return res;
    }

private:
    core::StringId name_;
    PathSegmentFlags flags_ = PathSegmentFlag::None;
    ArrayIndexType arrayIndex_ = 0;
};

/// \class vgc::dom::Path
/// \brief Represents a path to a node or attribute.
///
class VGC_DOM_API Path {
public:
    using ArrayIndexType = PathSegment::ArrayIndexType;

    /// Constructs a null path.
    Path() noexcept = default;
    Path(Element* element);
    Path(Element* element, core::StringId attributeName);
    Path(Element* element, core::StringId attributeName, ArrayIndexType arrayIndex);

    Path(std::string_view path);
    //Path(Element* element, std::string_view relativePath);

    size_t hash() const noexcept {
        size_t res = 'PATH';
        for (const PathSegment& s : segments_) {
            detail::hashCombine(res, s.hash());
        }
        return res;
    }

    std::string string() const noexcept;

    bool isEmpty() const noexcept {
        return segments_.isEmpty();
    }

    bool isAbsolute() const noexcept {
        if (!segments_.isEmpty()) {
            const PathSegment& seg = segments_.getUnchecked(0);
            return seg.isRoot() || seg.isUniqueId();
        }
        return false;
    }

    bool isBased() const noexcept {
        return !segments_.isEmpty() && segments_.getUnchecked(0).isElement();
    }

    bool isElement() const noexcept {
        return !segments_.isEmpty() && segments_.getUnchecked(0).isElement();
    }

    bool isAttribute() const noexcept {
        return !segments_.isEmpty() && segments_.getUnchecked(0).isAttribute();
    }

    Path elementPath() const;
    Path elementRelativeAttributePath() const;

    Path& removeAttribute();
    Path& removeIndex();

private:
    core::Array<PathSegment> segments_;
};

} // namespace vgc::dom

#endif // VGC_DOM_PATH_H
