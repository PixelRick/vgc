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

namespace vgc::dom {

VGC_DECLARE_OBJECT(Document);
VGC_DECLARE_OBJECT(Element);
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
enum class PathSegmentType : UInt8 {
    Root = 0,
    UniqueId,
    Dot,
    Element,
    Attribute,
};
// clang-format on

/// \enum vgc::dom::PathSegmentFlag
/// \brief Specifies special properties of a path segment.
///
// clang-format off
enum class PathSegmentFlag : UInt8 {
    None        = 0x00,
    Indexed     = 0x01, // only allowed for attributes atm
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
    using ArrayIndex = Int;

    // Constructs a segment representing the root element.
    constexpr PathSegment() noexcept = default;

    explicit PathSegment(
        core::StringId name,
        PathSegmentType type = PathSegmentType::Element,
        PathSegmentFlags flags = PathSegmentFlag::None,
        Int arrayIndex = 0) noexcept;

    core::StringId name() const noexcept {
        return name_;
    }

    PathSegmentType type() const noexcept {
        return type_;
    }

    PathSegmentFlags flags() const noexcept {
        return flags_;
    }

    bool isIndexed() const noexcept {
        return flags_.has(PathSegmentFlag::Indexed);
    }

    Int arrayIndex() const noexcept {
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
    PathSegmentType type_ = PathSegmentType::Element;
    PathSegmentFlags flags_ = PathSegmentFlag::None;
    Int arrayIndex_ = 0;
};

/// \class vgc::dom::Path
/// \brief Represents a path to a node or attribute.
///
class VGC_DOM_API Path {
public:
    /// Constructs a null path.
    Path() noexcept = default;
    Path(Element* element);
    Path(Element* element, core::StringId attributeName);
    Path(Element* element, core::StringId attributeName, Int arrayIndex);

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
            PathSegmentType type = segments_.getUnchecked(0).type();
            return (type == PathSegmentType::Root) || (type == PathSegmentType::UniqueId);
        }
        return false;
    }

    bool isRelative() const noexcept {
        return !isAbsolute();
    }

    bool isBased() const noexcept {
        return !segments_.isEmpty()
               && segments_.getUnchecked(0).type() == PathSegmentType::UniqueId;
    }

    bool isElement() const noexcept {
        return !segments_.isEmpty()
               && segments_.getUnchecked(0).type() == PathSegmentType::Element;
    }

    bool isAttribute() const noexcept {
        return !segments_.isEmpty()
               && segments_.getUnchecked(0).type() == PathSegmentType::Attribute;
    }

    const core::Array<PathSegment>& segments() const {
        return segments_;
    }

    Path elementPath() const;
    Path elementRelativeAttributePath() const;

    Path& removeAttribute();
    Path& removeIndex();

private:
    core::Array<PathSegment> segments_;
};

/// Reads a `Path` from the input stream, and stores it in the given output
/// parameter `v`. Leading whitespaces are allowed. Raises `ParseError` if the
/// stream does not start with a `Path`.
///
template<typename IStream>
void readTo(Path& v, IStream& in) {
    skipWhitespaceCharacters(in);
    skipExpectedCharacter(in, '@');
    skipExpectedCharacter(in, '\'');
    std::string s = readStringUntilExpectedCharacter(in, '\'');
    v = Path(s);
}

} // namespace vgc::dom

#endif // VGC_DOM_PATH_H
