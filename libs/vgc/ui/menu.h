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

#ifndef VGC_UI_Menu_H
#define VGC_UI_Menu_H

#include <vgc/core/color.h>
#include <vgc/graphics/richtext.h>
#include <vgc/ui/action.h>
#include <vgc/ui/widget.h>

namespace vgc::ui {

namespace detail {

struct MenuItem {
    ActionPtr action;
    graphics::RichText text;
};

} // namespace detail

VGC_DECLARE_OBJECT(Menu);

/// \class vgc::ui::Menu
/// \brief A menu widget.
///
class VGC_UI_API Menu : public Widget {
private:
    VGC_OBJECT(Menu, Widget)

protected:
    /// This is an implementation details. Please use
    /// Menu::create(text) instead.
    ///
    Menu(const std::string& text);

public:
    /// Creates a Menu.
    ///
    static MenuPtr create();

    /// Creates a Menu with the given text.
    ///
    static MenuPtr create(const std::string& text);

    /// Returns the Menu's action.
    ///
    Action* menuAction() const {
        return menuAction_;
    }

    // reimpl
    void onResize() override;
    void onPaintCreate(graphics::Engine* engine) override;
    void onPaintDraw(graphics::Engine* engine, PaintOptions options) override;
    void onPaintDestroy(graphics::Engine* engine) override;
    bool onMouseMove(MouseEvent* event) override;
    bool onMousePress(MouseEvent* event) override;
    bool onMouseRelease(MouseEvent* event) override;
    bool onMouseEnter() override;
    bool onMouseLeave() override;

protected:
    geometry::Vec2f computePreferredSize() const override;

private:
    // sub-menus can be children
    // hovering adds the sub-menu as overlay in window

    // XXX menu-item -> variant<menu, action, sep>

    Action* menuAction_;
    core::Array<detail::MenuItem> items_;
    graphics::GeometryViewPtr triangles_;
    bool isGeomDirty_;
    bool isHovered_;
};

} // namespace vgc::ui

#endif // VGC_UI_Menu_H
