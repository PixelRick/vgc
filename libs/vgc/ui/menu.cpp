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

#include <vgc/ui/menu.h>

#include <vgc/core/array.h>
#include <vgc/style/strings.h>
#include <vgc/ui/logcategories.h>
#include <vgc/ui/menubutton.h>
#include <vgc/ui/overlayarea.h>
#include <vgc/ui/popuplayer.h>
#include <vgc/ui/strings.h>

#include <vgc/ui/detail/paintutil.h>

namespace vgc::ui {

namespace {

/*
FlexDirection orthoDir(FlexDirection dir) {
    switch (dir) {
    case FlexDirection::Row:
    case FlexDirection::RowReverse:
        return FlexDirection::Column;
    case FlexDirection::Column:
    case FlexDirection::ColumnReverse:
    default:
        return FlexDirection::Row;
    }
}
*/

namespace commands {

VGC_UI_DEFINE_TRIGGER_COMMAND( //
    open,
    "ui.menu.open",
    "Open Menu")

VGC_UI_DEFINE_TRIGGER_COMMAND( //
    exit,
    "ui.menu.exit",
    "Exit Menu",
    Shortcut(Key::Escape));

} // namespace commands

} // namespace

MenuItem::MenuItem(Widget* widget) noexcept
    : widget_(widget) {
}

MenuItem::MenuItem(Action* action, MenuButton* button) noexcept
    : action_(action)
    , widget_(button) {
}

MenuItem::MenuItem(Action* action, MenuButton* button, Menu* menu) noexcept
    : action_(action)
    , widget_(button)
    , menu_(menu) {
}

Menu::Menu(CreateKey key, std::string_view title)
    : Flex(key, FlexDirection::Column, FlexWrap::NoWrap) {

    addStyleClass(strings::Menu);
    setFocusPolicy(FocusPolicy::Click);
    setFocusStrength(FocusStrength::Low);

    action_ = createTriggerAction(commands::open(), title);
    action_->isMenu_ = true;
    action_->triggered().connect(onSelfActionTriggeredSlot_());

    Action* exitAction = createTriggerAction(commands::exit());
    exitAction->triggered().connect(exitSlot_());
}

MenuPtr Menu::create() {
    return core::createObject<Menu>("");
}

MenuPtr Menu::create(std::string_view text) {
    return core::createObject<Menu>(text);
}

void Menu::setTitle(std::string_view title) {
    action_->setText(title);
    notifyChanged(false);
}

void Menu::addSeparator() {
    Widget* separator = createChild<Widget>();
    separator->addStyleClass(strings::separator);
    items_.emplaceLast(MenuItem(separator));
    notifyChanged(true);
}

void Menu::addItem(Action* action) {
    MenuButton* menuButton = createChild<MenuButton>(action);
    items_.emplaceLast(MenuItem(action, menuButton));
    onItemAdded_(items_.last());
    notifyChanged(true);
}

void Menu::addItem(Menu* menu) {
    Action* action = menu->menuAction();
    MenuButton* menuButton = createChild<MenuButton>(action);
    items_.emplaceLast(MenuItem(action, menuButton, menu));
    onItemAdded_(items_.last());
    notifyChanged(true);
}

Menu* Menu::createSubMenu(std::string_view title) {
    Menu* menu = createChild<Menu>(title);
    menu->hide();
    addItem(menu);
    return menu;
}

bool Menu::open(Widget* from) {
    close_();
    if (isPopupEnabled_) {
        Widget* host = parent();
        if (!openAsPopup_(from)) {
            return false;
        }
        host_ = host;
        isOpenAsPopup_ = true;
        show();
        popupOpened().emit();
    }
    else if (!isVisible()) {
        show();
    }
    return true;
}

bool Menu::close() {
    return close_();
}

bool Menu::closeSubMenu() {
    Menu* subMenu = subMenuPopup();
    if (subMenu) {
        return subMenu->close();
    }
    return false;
}

void Menu::setPopupEnabled(bool enabled) {
    if (isPopupEnabled_ == enabled) {
        return;
    }
    close_();
    isPopupEnabled_ = enabled;
}

void Menu::setShortcutTrackEnabled(bool enabled) {
    if (isShortcutTrackEnabled_ != enabled) {
        isShortcutTrackEnabled_ = enabled;
        for (const MenuItem& item : items_) {
            MenuButton* button = item.button();
            if (button) {
                button->setShortcutVisible(enabled);
            }
        }
        requestGeometryUpdate();
    }
}

namespace {

bool placeMenuFit(
    geometry::Rect2f& menuRect,
    const geometry::Vec2f& fixedCrossAlignShifts,
    const geometry::Rect2f& areaRect,
    const geometry::Rect2f& anchorRect,
    int dropDir) {

    const geometry::Vec2f menuSize = menuRect.size();
    const int crossDir = dropDir ? 0 : 1;

    const geometry::Vec2f areaMin = areaRect.pMin();
    const geometry::Vec2f areaMax = areaRect.pMax();
    const geometry::Vec2f anchorMin = anchorRect.pMin();
    const geometry::Vec2f anchorMax = anchorRect.pMax();
    geometry::Vec2f resultPos;

    if (anchorMax[dropDir] + menuSize[dropDir] <= areaMax[dropDir]) {
        // Enough space on the Right/Bottom
        resultPos[dropDir] = anchorMax[dropDir];
    }
    else if (anchorMin[dropDir] - menuSize[dropDir] >= areaMin[dropDir]) {
        // Enough space on the Left/Top
        resultPos[dropDir] = anchorMin[dropDir] - menuSize[dropDir];
    }
    else {
        return false;
    }

    float delta = menuSize[crossDir];
    if (anchorMin[crossDir] + delta + fixedCrossAlignShifts[1] <= areaMax[crossDir]) {
        // Enough space Bottom/Right
        resultPos[crossDir] = anchorMin[crossDir] + fixedCrossAlignShifts[1];
    }
    else if (
        anchorMax[crossDir] - delta - fixedCrossAlignShifts[0] >= areaMin[crossDir]) {
        // Enough space Top/Left
        resultPos[crossDir] = anchorMax[crossDir] - delta - fixedCrossAlignShifts[0];
    }
    else {
        return false;
    }

    menuRect.setPosition(resultPos);
    return true;
}

MenuDropDirection getMenuDropDirection(Menu* parentMenu, MenuButton* button) {
    if (parentMenu) {
        if (parentMenu->isRow()) {
            return MenuDropDirection::Vertical;
        }
        else {
            return MenuDropDirection::Horizontal;
        }
    }
    else if (button) {
        return button->menuDropDirection();
    }
    else {
        return MenuDropDirection::Horizontal;
    }
}

} // namespace

geometry::Vec2f Menu::computePopupPosition(Widget* opener, Widget* area) {

    MenuButton* button = dynamic_cast<MenuButton*>(opener);
    Menu* parentMenu = button ? button->parentMenu() : nullptr;

    MenuDropDirection dropDir = getMenuDropDirection(parentMenu, button);

    geometry::Rect2f anchorRect = opener->mapTo(area, opener->rect());
    geometry::Rect2f areaRect = area->rect();
    geometry::Rect2f menuRect(geometry::Vec2f(), preferredSize());

    const bool preferLeftRight = (dropDir == MenuDropDirection::Horizontal);
    const int sideDir = preferLeftRight ? 0 : 1;
    const int crossDir = preferLeftRight ? 1 : 0;

    Margins paddingAndBorder(rect(), contentRect());
    std::array<geometry::Vec2f, 2> fixedShifts = {
        geometry::Vec2f{-paddingAndBorder.bottom(), -paddingAndBorder.top()},
        geometry::Vec2f{-paddingAndBorder.right(), -paddingAndBorder.left()}};

    float crossShift = anchorRect.size()[sideDir] * 0.2f;
    fixedShifts[crossDir] = geometry::Vec2f(crossShift, crossShift);

    if (!placeMenuFit(menuRect, fixedShifts[sideDir], areaRect, anchorRect, sideDir)) {
        if (!placeMenuFit(
                menuRect, fixedShifts[crossDir], areaRect, anchorRect, crossDir)) {
            // doesn't fit at aligned locations, fallback
            menuRect.setPosition(anchorRect.pMax());
            menuRect.setWidth((std::min)(menuRect.width(), areaRect.width()));
            menuRect.setHeight((std::min)(menuRect.height(), areaRect.height()));
            geometry::Vec2f newPos = menuRect.position();
            if (menuRect.pMax().x() > areaRect.pMax().x()) {
                newPos[0] = areaRect.pMax().x() - menuRect.width();
            }
            else if (menuRect.pMin().x() < areaRect.pMin().x()) {
                newPos[0] = areaRect.pMin().x();
            }
            if (menuRect.pMax().y() > areaRect.pMax().y()) {
                newPos[1] = areaRect.pMax().y() - menuRect.width();
            }
            else if (menuRect.pMin().y() < areaRect.pMin().y()) {
                newPos[1] = areaRect.pMin().y();
            }
            menuRect.setPosition(newPos);
        }
    }
    return menuRect.position();
}

void Menu::notifyChanged(bool geometryChanged) {
    if (geometryChanged) {
        requestGeometryUpdate();
    }
    changed().emit();
}

void Menu::removeItem(Widget* widget) {
    bool changed_ = false;
    for (auto it = items_.begin(); it != items_.end(); ++it) {
        const MenuItem& item = *it;
        if (item.widget() == widget) {
            changed_ = true;
            preItemRemoved_(item);
            items_.erase(it);
            break;
        }
    }
    if (changed_) {
        changed().emit();
    }
}

void Menu::onItemAdded_(const MenuItem& item) {
    MenuButton* button = item.button();
    if (button) {
        button->parentMenu_ = this;
        //button->setDirection(orthoDir(direction()));
        button->setDirection(FlexDirection::Row);
        button->setShortcutVisible(isShortcutTrackEnabled_);
    }
    item.action()->triggered().connect(onItemActionTriggeredSlot_());
    item.action()->aboutToBeDestroyed().connect(onItemActionAboutToBeDestroyedSlot_());
}

void Menu::preItemRemoved_(const MenuItem& item) {
    MenuButton* button = item.button();
    if (button) {
        button->closePopupMenu();
        button->parentMenu_ = nullptr;
    }
    Action* action = item.action();
    if (action) {
        action->triggered().disconnect(onItemActionTriggeredSlot_());
    }
}

void Menu::setupWidthOverrides_() const {
    const core::Array<MenuItem>& items = items_;
    if (isShortcutTrackEnabled_) {
        float maxShortcutWidth = 0.f;
        for (const MenuItem& item : items) {
            MenuButton* button = item.button();
            if (button) {
                maxShortcutWidth =
                    (std::max)(maxShortcutWidth, button->preferredShortcutSize().x());
            }
        }
        for (const MenuItem& item : items) {
            MenuButton* button = item.button();
            if (button) {
                button->setShortcutSizeOverrides(maxShortcutWidth, -1.0f);
            }
        }
    }
}

void Menu::createPopupLayer_(OverlayArea* area, Widget* underlyingWidget) {
    if (popupLayer_) {
        destroyPopupLayer_();
    }
    PopupLayer* popupLayer = area->createOverlayWidget<PopupLayer>(
        OverlayResizePolicy::Stretch, underlyingWidget);
    popupLayer_ = popupLayer;
    popupLayer->resized().connect(exitSlot_());
    popupLayer->backgroundPressed().connect(exitSlot_());
}

void Menu::destroyPopupLayer_() {
    if (popupLayer_) {
        popupLayer_->destroy();
        popupLayer_ = nullptr;
    }
}

bool Menu::openAsPopup_(Widget* from) {

    MenuButton* button = dynamic_cast<MenuButton*>(from);
    Menu* parentMenu = button ? button->parentMenu() : nullptr;

    // Find the OverlayArea where to place the popup.
    //
    OverlayArea* area = nullptr;
    if (parentMenu && parentMenu->isOpenAsPopup()) {
        area = dynamic_cast<OverlayArea*>(parentMenu->parent());
    }
    if (!area) {
        area = from ? from->topmostOverlayArea() : topmostOverlayArea();
    }
    if (!area) {
        VGC_WARNING(
            LogVgcUi,
            "Menu couldn't be opened as a popup because the initiator widget has no "
            "top-most overlay area.")
        return false;
    }

    // Create a PopupLayer catching all clicks outside of the popup.
    //
    // Note that if this menu has a parent menu, then it is the responsibility
    // of the parent to create the PopupLayer, if there isn't one already. This
    // is done in `onSubMenuPopupOpened_()`.
    //
    if (!parentMenu) {
        createPopupLayer_(area);
    }

    geometry::Vec2f pos = computePopupPosition(from, area);
    area->addOverlayWidget(this);
    updateGeometry(pos, preferredSize());

    // Let the button know.
    if (button) {
        button->onMenuPopupOpened_(this);
    }

    return true;
}

bool Menu::close_(bool recursive) {
    if (isPopupEnabled_) {
        if (!isOpenAsPopup_) {
            return false;
        }
        if (host_) {
            reparent(host_);
        }
        host_ = nullptr;
        isOpenAsPopup_ = false;
        hide();
        popupClosed().emit(recursive);
    }
    else if (isVisible()) {
        hide();
    }
    clearFocus(FocusReason::Menu);
    return true;
}

void Menu::exit_() {
    closeSubMenu();
    if (isOpenAsPopup()) {
        bool recursive = true;
        close_(recursive);
    }
    destroyPopupLayer_();
    clearFocus(FocusReason::Menu);
}

void Menu::onSelfActionTriggered_(Widget* from) {
    if (isPopupEnabled_) {
        open(from);
    }
    else if (isVisible()) {
        close_();
    }
    else {
        open(nullptr);
    }
}

void Menu::onItemActionTriggered_(Widget* from) {

    // Detect whether the triggered action opened a new popup menu
    // or was any other type of action.
    //
    Menu* newPopup = nullptr;
    for (const MenuItem& item : items_) {
        MenuButton* button = item.button();
        if (item.widget() == from) {
            if (button) {
                newPopup = button->popupMenu();
            }
        }
        else if (button) {
            // close other menus
            button->closePopupMenu();
        }
    }

    if (newPopup) {
        // If a new popup menu was opened, then we register it as our
        // subMenuPopup(), and create a PopupLayer if necessary.
        onSubMenuPopupOpened_(newPopup);
    }
    else {
        // Otherwise, this means that an actual action has been perfomed,
        // so we we can now close this menu and all its parent menus.
        exit_();
    }
}

void Menu::onSubMenuPopupOpened_(Menu* subMenu) {

    // Register sub-menu.
    //
    subMenu->popupClosed().connect(onSubMenuPopupClosedSlot_());
    subMenu->aboutToBeDestroyed().connect(onSubMenuPopupDestroySlot_());
    subMenuPopup_ = subMenu;
    subMenuPopupHitRect_ = subMenu->mapTo(this, subMenu->rect());

    // Add margins to popup hit rect when applicable (no overlap with our buttons).
    //
    geometry::Rect2f itemsRect = contentRect();
    const float hitMargin = 5.f;
    Margins hitMargins = {};
    if (subMenuPopupHitRect_.xMin() >= itemsRect.xMax()
        || subMenuPopupHitRect_.xMax() <= itemsRect.xMin()) {
        hitMargins.setTop(hitMargin);
        hitMargins.setBottom(hitMargin);
    }
    if (subMenuPopupHitRect_.yMin() >= itemsRect.yMax()
        || subMenuPopupHitRect_.yMax() <= itemsRect.yMin()) {
        hitMargins.setRight(hitMargin);
        hitMargins.setLeft(hitMargin);
    }
    subMenuPopupHitRect_ = subMenuPopupHitRect_ + hitMargins;

    // Create a PopupLayer catching all clicks outside the popup or `this`
    // menu, and move the subMenu above the PopupLayer.
    //
    isDeferringOpen_ = false;
    if (!isOpenAsPopup_) {
        if (!popupLayer_) {
            OverlayArea* area = dynamic_cast<OverlayArea*>(subMenu->parent());
            if (!area) {
                area = topmostOverlayArea();
            }
            if (area) {
                createPopupLayer_(area, this);
                area->addOverlayWidget(subMenu);
            }
        }
    }
}

void Menu::onSubMenuPopupClosed_(bool recursive) {
    if (subMenuPopup_ == emitter()) {
        subMenuPopup_->popupClosed().disconnect(onSubMenuPopupClosedSlot_());
        subMenuPopup_->aboutToBeDestroyed().disconnect(onSubMenuPopupDestroySlot_());
        subMenuPopup_ = nullptr;
        if (recursive) {
            exit_();
        }
    }
}

void Menu::onSubMenuPopupDestroy_() {
    if (subMenuPopup_ == emitter()) {
        subMenuPopup_ = nullptr;
    }
}

void Menu::onItemActionAboutToBeDestroyed_() {
    Action* action = static_cast<Action*>(emitter());
    for (auto it = items_.begin(); it != items_.end(); ++it) {
        MenuItem& item = *it;
        if (item.action_ == action) {
            item.action_ = nullptr;
        }
    }
}

// Reimplementation of Widget virtual methods

void Menu::onParentWidgetChanged(Widget* newParent) {
    popupLayer_ = dynamic_cast<PopupLayer*>(newParent);
}

void Menu::onWidgetRemoved(Widget* widget) {
    removeItem(widget);
}

void Menu::preMouseMove(MouseMoveEvent* event) {
    Widget* hcc = hoverChainChild();
    if (hcc && hcc->parent() != this) {
        return;
    }

    geometry::Vec2f newHoverPos = event->position();
    if (isFirstMoveSinceEnter_) {
        // Hover discontinuity (leave/enter).
        lastHoverPos_ = newHoverPos;
    }

    geometry::Vec2f delta = newHoverPos - lastHoverPos_;
    float sqDist = delta.squaredLength();
    bool moved = sqDist > 15.f; // ~4 pixels

    const bool hasOpenSubmenuPopup = subMenuPopup_;

    // A menu is either docked (menu bar) or popup (dropdown).
    // A drop-down menu always opens sub-menus on hover.
    // A menu bar opens its sub-menus on hover only if one is already open.
    const bool shouldOpenSubmenuOnHover = isOpenAsPopup_ || hasOpenSubmenuPopup;
    const bool shouldProtectOpenSubMenu = isOpenAsPopup_;

    MenuButton* button = dynamic_cast<MenuButton*>(hcc);
    Action* action = button ? button->action() : nullptr;
    const bool isHccMenu = action && action->isMenu_ && action->isEnabled();

    bool doNothing = false;
    if (shouldProtectOpenSubMenu && subMenuPopup_ && !isFirstMoveSinceEnter_) {
        if (!moved) {
            // Move was too small, let's keep the sub-menu open.
            doNothing = true;
        }
        else {
            geometry::Vec2f origin = lastHoverPos_;
            geometry::Vec2f dir = newHoverPos - origin;
            dir.normalize();
            // Assumes pos is out of the popup rect.
            int hitPlaneDim = -1;
            float hitDist = 0.f;
            for (int i = 0; i < 2; ++i) {
                float c = dir[i];
                // Check parallelism.
                if (c == 0.f) {
                    continue;
                }
                float o = origin[i];
                // Find candidate plane.
                float plane = subMenuPopupHitRect_.pMin()[i];
                if (newHoverPos[i] > plane) {
                    plane = subMenuPopupHitRect_.pMax()[i];
                    if (newHoverPos[i] < plane) {
                        continue;
                    }
                }
                // Calculate dist.
                float d = (plane - o) / c;
                if (d > hitDist) {
                    hitPlaneDim = i;
                    hitDist = d;
                }
            }
            if (hitPlaneDim >= 0) {
                int hitCrossDim = hitPlaneDim ? 0 : 1;
                float vHit = origin[hitCrossDim] + hitDist * dir[hitCrossDim];
                geometry::Vec2f hitBounds = geometry::Vec2f(
                    subMenuPopupHitRect_.pMin()[hitCrossDim],
                    subMenuPopupHitRect_.pMax()[hitCrossDim]);
                // If hit, let's do nothing on hover.
                if (vHit >= hitBounds[0] && vHit <= hitBounds[1]) {
                    doNothing = true;
                }
            }
        }
    }

    if (hcc && !doNothing) {
        // We have no pointer to our current active button atm.
        // But we can check if the hovered button's open popup
        // menu is our open sub-menu.
        Menu* hccMenuPopup = button ? button->popupMenu() : nullptr;
        if (!subMenuPopup_ || subMenuPopup_ != hccMenuPopup) {
            if (isOpenAsPopup_ || isHccMenu) {
                closeSubMenu();
            }
            if (isHccMenu && shouldOpenSubmenuOnHover) {
                button->click(newHoverPos);
                // Update move origin now.
                lastHoverPos_ = newHoverPos;
            }
        }
    }

    if (moved) {
        lastHoverPos_ = newHoverPos;
    }
    isFirstMoveSinceEnter_ = false;
}

void Menu::preMousePress(MousePressEvent* event) {
    // Close everything if we click on an openable menu in a docked menu that
    // is already active.
    if (!isOpenAsPopup_ && subMenuPopup_) {
        Widget* hcc = hoverChainChild();
        if (!hcc || !hcc->isHoverLocked()) {
            exit_();
            event->stopPropagation();
        }
    }
    // Update move origin now.
    lastHoverPos_ = event->position();
}

void Menu::onMouseEnter() {
    isFirstMoveSinceEnter_ = true;
}

void Menu::onMouseLeave() {
}

void Menu::onVisible() {
}

void Menu::onHidden() {
    closeSubMenu();
    destroyPopupLayer_();
    isDeferringOpen_ = true;
}

geometry::Vec2f Menu::computePreferredSize() const {
    setupWidthOverrides_();
    geometry::Vec2f ret = Flex::computePreferredSize();
    return ret;
}

void Menu::updateChildrenGeometry() {
    setupWidthOverrides_();
    Flex::updateChildrenGeometry();
}

} // namespace vgc::ui
