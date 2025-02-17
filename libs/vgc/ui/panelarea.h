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

#ifndef VGC_UI_PANELAREA_H
#define VGC_UI_PANELAREA_H

#include <vgc/ui/cursor.h>
#include <vgc/ui/panel.h>
#include <vgc/ui/widget.h>

namespace vgc::ui {

/// \enum vgc::ui::PanelAreaType
/// \brief The type of a PanelArea
///
enum class PanelAreaType : UInt8 {
    HorizontalSplit = 0,
    VerticalSplit = 1,
    Tabs = 2
};

VGC_UI_API
VGC_DECLARE_ENUM(PanelAreaType)

VGC_DECLARE_OBJECT(PanelArea);
VGC_DECLARE_OBJECT(TabBar);
VGC_DECLARE_OBJECT(TabBody);

namespace detail {

struct PanelAreaSplitData {

    PanelAreaSplitData(PanelArea* childArea, float position, float size)
        : childArea(childArea)
        , position(position)
        , size(size) {
    }

    PanelArea* childArea;
    bool isInteractive = true;

    // Style values
    float stretch;
    style::LengthOrPercentage minSizeStyle;
    style::LengthOrPercentage maxSizeStyle;
    float minSize; // in px
    float maxSize; // in px
    float minSizeInDp;
    float maxSizeInDp;

    // Current size and preferred size. The preferred size is the size that the
    // child area had when the user last dragged a splitter.
    //
    // When the user drags a splitter, we perform all computation based on
    // current sizes, and update the preferred size accordingly.
    //
    // When the size of the window (or parent area) changes, we perform all
    // computation based on preferred sizes, and update the current sizes
    // accordingly.
    //
    // Note that we need to store the preferred size in dp to properly support
    // dragging the window between monitors with different dpi scaling.
    //
    float preferredSizeInDp;
    float position; // current position in px
    float size;     // current size in px

    // hinted values (in px)
    float hPosition;
    float hSize;

    // todo: isCollapsible, gap/padding, lastVisibleSize,
    //       animatedPosition, animatedSize...
};

using PanelAreaSplitDataArray = core::Array<PanelAreaSplitData>;

// Structure used to order child areas by "normalized slack", that is, how much
// total extra space is required before the child area's size reaches its max
// (or min) size. This order makes it possible to resolved all min/max
// constraints in one pass, since child areas reaching their min/max size
// faster are processed first.
//
struct PanelAreaResizeData {

    // Which child area this is referring to
    PanelAreaSplitData* splitData;

    // In stretch mode: stretch = data.stretch
    // In shrink mode:  stretch = (data.preferredSize - data.minSize) * data.stretch
    float stretch;

    // In stretch mode: normalizedSlack = (data.maxSize - data.preferredSize) / data.stretch
    // In shrink mode:  normalizedSlack = 1.0 / data.stretch
    // (as a special case, in both modes, if data.stretch == 0 then normalizedSlack = 0)
    float normalizedSlack;

    friend bool operator<(const PanelAreaResizeData& a, const PanelAreaResizeData& b) {
        return a.normalizedSlack < b.normalizedSlack;
    }
};

using PanelAreaResizeArray = core::Array<PanelAreaResizeData>;

} // namespace detail

/// \class vgc::ui::PanelArea
/// \brief Splits the workspace into different areas where to place Panels.
///
class VGC_UI_API PanelArea : public Widget {
private:
    VGC_OBJECT(PanelArea, Widget)

protected:
    PanelArea(CreateKey, PanelAreaType type);

public:
    /// Creates a `PanelArea`.
    ///
    static PanelAreaPtr create(PanelAreaType type);

    /// Creates a `PanelArea` of type `HorizontalSplit` as a child of the given `parent`.
    ///
    static PanelArea* createHorizontalSplit(Widget* parent) {
        return parent->createChild<PanelArea>(PanelAreaType::HorizontalSplit);
    }

    /// Creates a `PanelArea` of type `VerticalSplit` as a child of the given `parent`.
    ///
    static PanelArea* createVerticalSplit(Widget* parent) {
        return parent->createChild<PanelArea>(PanelAreaType::VerticalSplit);
    }

    /// Creates a `PanelArea` of type `Tabs` as a child of the given `parent`.
    ///
    static PanelArea* createTabs(Widget* parent) {
        return parent->createChild<PanelArea>(PanelAreaType::Tabs);
    }

    /// Returns the type of this `PanelArea`.
    ///
    PanelAreaType type() const {
        return type_;
    }

    /// Changes the type of this `PanelArea`.
    ///
    /// If the type is changed from `Tabs` to `VerticalSplit` or
    /// `HorizontalSplit` (or the other way around), and this `PanelArea`
    /// already have some children, then a warning is emitted and all children
    /// are destroyed before the type is changed.
    ///
    void setType(PanelAreaType type);

    /// Returns whether the `type()` of this `PanelArea` is `HorizontalSplit` or
    /// `VerticalSplit`.
    ///
    bool isSplit() const {
        return isSplit_(type_);
    }

    /// Adds a `Panel` to this `PanelArea`. A warning is emitted if this
    /// `PanelArea` is not of type `Tabs`.
    ///
    Panel* createPanel(std::string_view panelTitle = "Untitled");

    /// Returns the number of panels in this `PanelArea`.
    ///
    /// If the type of this `PanelArea` is `Tabs`, this is equal to the number of tabs.
    ///
    /// Otherwise, it is the recursive total of all panels in all child areas of
    /// this `PanelArea`.
    ///
    Int numPanels() const;

    /// Returns the `TabBar` of this `PanelArea`. Return `nullptr` if this
    /// area is not of type `Tabs`.
    ///
    TabBar* tabBar() const;

    /// Returns the `TabBody` of this `PanelArea`. Return `nullptr` if this
    /// area is not of type `Tabs`.
    ///
    TabBody* tabBody() const;

    // Implementation of StylableObject interface
    static void populateStyleSpecTable(style::SpecTable* table);
    void populateStyleSpecTableVirtual(style::SpecTable* table) override {
        populateStyleSpecTable(table);
    }

protected:
    // Reimplementation of StylableObject virtual methods.
    void onStyleChanged() override;

    // Reimplementation of Widget protected virtual methods
    // XXX Some of these are currently public in Widget, but we should
    //     probably make them protected for consistency
    void onWidgetAdded(Widget* child, bool wasOnlyReordered) override;
    void onWidgetRemoved(Widget* child) override;
    void onMouseEnter() override;
    void onMouseLeave() override;
    Widget* computeHoverChainChild(MouseHoverEvent* event) const override;
    void preMouseMove(MouseMoveEvent* event) override;
    bool onMouseMove(MouseMoveEvent* event) override;
    bool onMousePress(MousePressEvent* event) override;
    bool onMouseRelease(MouseReleaseEvent* event) override;
    void onResize() override;
    void updateChildrenGeometry() override;
    void onPaintCreate(graphics::Engine* engine) override;
    void onPaintDraw(graphics::Engine* engine, PaintOptions options) override;
    void onPaintDestroy(graphics::Engine* engine) override;

private:
    using SplitData = detail::PanelAreaSplitData;
    using SplitDataArray = detail::PanelAreaSplitDataArray;

    PanelAreaType type_;
    static bool isSplit_(PanelAreaType type) {
        return static_cast<UInt8>(type) < 2;
    }

    SplitDataArray splitData_; // size = num child widgets

    graphics::GeometryViewPtr triangles_;
    CursorChanger cursorChanger_;
    Int hoveredSplitHandle_ = -1; // invariant: -1 or [1..n-1]
    Int draggedSplitHandle_ = -1; // invariant: -1 or [1..n-1]
    float dragStartMousePosition_;
    float dragStartSplitSizeBefore_;
    float dragStartSplitSizeAfter_;

    // Style
    float halfHandleSize_ = 0.0f;
    float halfHandleHoveredSize_ = 0.0f;
    core::Color handleHoveredColor_;

    // Orders the child areas by "normalized slack". This could be a local
    // variable, but we make it a data member to avoid dynamic allocations.
    detail::PanelAreaResizeArray resizeArray_;

    bool isUpdatingSplitData_ = false;
    void onChildrenChanged_();
    Int computeHoveredSplitHandle_(const geometry::Vec2f& position) const;
    void updateHoveredSplitHandle_(const geometry::Vec2f& position);
    void setHoveredSplitHandle_(Int index);
    void startDragging_(const geometry::Vec2f& position);
    void continueDragging_(const geometry::Vec2f& position);
    void stopDragging_(const geometry::Vec2f& position);

    // Creates / Updates PanelTabs widget when necessary (i.e., when type = Tabs).
    void updateTabs_();
};

} // namespace vgc::ui

#endif // VGC_UI_PANELAREA_H
