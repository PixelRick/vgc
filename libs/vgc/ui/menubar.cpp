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

#include <vgc/ui/menubar.h>

#include <vgc/core/array.h>
#include <vgc/ui/strings.h>

#include <vgc/ui/detail/paintutil.h>

namespace vgc::ui {

MenuBar::MenuBar(const std::string& text)
    : Widget()
    , text_(text)
    , reload_(true)
    , isHovered_(false) {

    addStyleClass(strings::MenuBar);
}

MenuBarPtr MenuBar::create() {
    return MenuBarPtr(new MenuBar(""));
}

MenuBarPtr MenuBar::create(const std::string& text) {
    return MenuBarPtr(new MenuBar(text));
}

void MenuBar::setText(const std::string& text) {
    if (text_ != text) {
        text_ = text;
        reload_ = true;
        repaint();
    }
}

void MenuBar::onResize() {
    reload_ = true;
}

void MenuBar::onPaintCreate(graphics::Engine* engine) {
    triangles_ =
        engine->createDynamicTriangleListView(graphics::BuiltinGeometryLayout::XYRGB);
}

void MenuBar::onPaintDraw(graphics::Engine* engine, PaintOptions /*options*/) {
    if (reload_) {
        reload_ = false;
        core::FloatArray a = {};
        core::Color backgroundColor = detail::getColor(
            this,
            isHovered_ ? strings::background_color_on_hover : strings::background_color);
        core::Color textColor = detail::getColor(this, strings::text_color);
        float borderRadius = detail::getLength(this, strings::border_radius);
        graphics::TextProperties textProperties(
            graphics::TextHorizontalAlign::Center, graphics::TextVerticalAlign::Middle);
        graphics::TextCursor textCursor;
        bool hinting = style(strings::pixel_hinting) == strings::normal;
        detail::insertRect(a, backgroundColor, rect(), borderRadius);
        detail::insertText(
            a, textColor, rect(), 0, 0, 0, 0, text_, textProperties, textCursor, hinting);
        engine->updateVertexBufferData(triangles_, std::move(a));
    }
    engine->setProgram(graphics::BuiltinProgram::Simple);
    engine->draw(triangles_, -1, 0);
}

void MenuBar::onPaintDestroy(graphics::Engine*) {
    triangles_.reset();
}

bool MenuBar::onMouseMove(MouseEvent* /*event*/) {
    return true;
}

bool MenuBar::onMousePress(MouseEvent* /*event*/) {
    return true;
}

bool MenuBar::onMouseRelease(MouseEvent* /*event*/) {
    return true;
}

bool MenuBar::onMouseEnter() {
    isHovered_ = true;
    reload_ = true;
    repaint();
    return true;
}

bool MenuBar::onMouseLeave() {
    isHovered_ = false;
    reload_ = true;
    repaint();
    return true;
}

geometry::Vec2f MenuBar::computePreferredSize() const {
    PreferredSizeType auto_ = PreferredSizeType::Auto;
    PreferredSize w = preferredWidth();
    PreferredSize h = preferredHeight();
    geometry::Vec2f res(0, 0);
    if (w.type() == auto_) {
        res[0] = 100;
        // TODO: compute appropriate width based on text length
    }
    else {
        res[0] = w.value();
    }
    if (h.type() == auto_) {
        res[1] = 26;
        // TODO: compute appropriate height based on font size?
    }
    else {
        res[1] = h.value();
    }
    return res;
}

} // namespace vgc::ui
