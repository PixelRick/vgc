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

#ifndef VGC_UI_MARGINS_H
#define VGC_UI_MARGINS_H

#include <vgc/geometry/rect2f.h>
#include <vgc/geometry/vec4f.h>
#include <vgc/ui/api.h>

namespace vgc::ui {

/// \class vgc::ui::Margins
/// \brief Represents a set of 4 margins for the four sides of a UI element.
///
class VGC_UI_API Margins {
private:
    enum Indices_ {
        Top,
        Right,
        Bottom,
        Left
    };

public:
    /// Constructs a `Margins` with all margins set to 0.
    ///
    constexpr Margins()
        : v_() {
    }

    /// Constructs a `Margins` with the given margins.
    ///
    constexpr Margins(float top, float right, float bottom, float left)
        : v_(top, right, bottom, left) {
    }

    /// Returns the top margin.
    ///
    constexpr float top() const {
        return v_[Top];
    }

    /// Sets the top margin to `margin`.
    ///
    void setTop(float margin) {
        v_[Top] = margin;
    }

    /// Returns the right margin.
    ///
    constexpr float right() const {
        return v_[Right];
    }

    /// Sets the right margin to `margin`.
    ///
    void setRight(float margin) {
        v_[Right] = margin;
    }

    /// Returns the bottom margin.
    ///
    constexpr float bottom() const {
        return v_[Bottom];
    }

    /// Sets the bottom margin to `margin`.
    ///
    void setBottom(float margin) {
        v_[Bottom] = margin;
    }

    /// Returns the left margin.
    ///
    constexpr float left() const {
        return v_[Left];
    }

    /// Sets the left margin to `margin`.
    ///
    void setLeft(float margin) {
        v_[Left] = margin;
    }

    /// Rounds the value of each margin.
    ///
    void round() {
        v_[Top] = std::roundf(v_[Top]);
        v_[Right] = std::roundf(v_[Right]);
        v_[Bottom] = std::roundf(v_[Bottom]);
        v_[Left] = std::roundf(v_[Left]);
    }

    /// Returns a copy with rounded margin values.
    ///
    Margins rounded() {
        Margins copy = *this;
        copy.round();
        return copy;
    }

    /// Adds `offset` to each margin.
    ///
    Margins& operator+=(float offset) {
        v_[Top] += offset;
        v_[Right] += offset;
        v_[Bottom] += offset;
        v_[Left] += offset;
        return *this;
    }

    /// Returns a copy of `this` with `offset` added to each margin.
    ///
    Margins operator+(float offset) const {
        Margins ret = *this;
        ret += offset;
        return ret;
    }

    /// Returns a copy of `margins` with `offset` added to each margin.
    ///
    friend Margins operator+(float offset, const Margins& margins) {
        return margins + offset;
    }

    /// Adds each margin of `other` to the respective margin of `this`.
    /// Returns a reference to `this`.
    ///
    Margins& operator+=(const Margins& other) {
        v_ += other.v_;
        return *this;
    }

    /// Returns a `Margins` with margins set to the sum of the respective
    /// margins of `other` and `this`.
    ///
    Margins operator+(const Margins& other) {
        Margins ret = *this;
        ret += other;
        return ret;
    }

    /// Returns a copy of `this` with each margin negated.
    ///
    Margins& operator-() {
        v_ = -v_;
        return *this;
    }

    /// Subtracts `offset` from each margin.
    ///
    Margins& operator-=(float offset) {
        v_[Top] -= offset;
        v_[Right] -= offset;
        v_[Bottom] -= offset;
        v_[Left] -= offset;
        return *this;
    }

    /// Returns a copy of `this` with `offset` subtracted from each margin.
    ///
    Margins operator-(float offset) const {
        Margins ret = *this;
        ret -= offset;
        return ret;
    }

    /// Returns a copy of `margins` with `offset` subtracted from each margin.
    ///
    friend Margins operator-(float offset, const Margins& margins) {
        return margins - offset;
    }

    /// Subtracts each margin of `other` from the respective margin of `this`.
    /// Returns a reference to `this`.
    ///
    Margins& operator-=(const Margins& other) {
        v_ -= other.v_;
        return *this;
    }

    /// Multiplies the value of each margin by `scale`.
    ///
    Margins& operator*=(float scale) {
        v_ *= scale;
        return *this;
    }

    /// Returns a copy of `this` with each margin multiplied by `scale`.
    ///
    Margins operator*(float scale) const {
        Margins ret = *this;
        ret *= scale;
        return ret;
    }

    /// Returns a copy of `margins` with each margin multiplied by `scale`.
    ///
    friend Margins operator*(float scale, const Margins& margins) {
        return margins * scale;
    }

    /// Divides the value of each margin by `divisor`.
    ///
    Margins& operator/=(float divisor) {
        v_ /= divisor;
        return *this;
    }

    /// Returns a copy of `this` with each margin divided by `divisor`.
    ///
    Margins operator/(float divisor) const {
        Margins ret = *this;
        ret /= divisor;
        return ret;
    }

    /// Returns a copy of `margins` with each margin divided by `divisor`.
    ///
    friend Margins operator*(float divisor, const Margins& margins) {
        return margins / divisor;
    }

private:
    geometry::Vec4f v_;
};

/// Returns a copy of `rect` offsetted outwards by `margins`.
///
geometry::Rect2f operator+(const geometry::Rect2f& rect, const Margins& margins) {
    return geometry::Rect2f(
        rect.xMin() + margins.left(),
        rect.yMin() + margins.top(),
        rect.xMax() - margins.right(),
        rect.yMax() - margins.bottom()
    );
}

/// Returns a copy of `rect` offsetted inwards by `margins`.
/// This is useful for padding.
///
geometry::Rect2f operator+(const geometry::Rect2f& rect, const Margins& margins) {
    return geometry::Rect2f(
        rect.xMin() + margins.left(),
        rect.yMin() + margins.top(),
        rect.xMax() - margins.right(),
        rect.yMax() - margins.bottom()
    );
}

} // namespace vgc::ui

#endif // VGC_UI_MARGINS_H
