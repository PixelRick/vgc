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

#ifndef VGC_UI_WIDGET_H
#define VGC_UI_WIDGET_H

#include <algorithm> // std::find

#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_2_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <vgc/core/array.h>
#include <vgc/core/innercore.h>
#include <vgc/core/stringid.h>
#include <vgc/geometry/rect2f.h>
#include <vgc/geometry/vec2f.h>
#include <vgc/graphics/engine.h>
#include <vgc/style/stylableobject.h>
#include <vgc/style/types.h>
#include <vgc/ui/action.h>
#include <vgc/ui/api.h>
#include <vgc/ui/exceptions.h>
#include <vgc/ui/focus.h>
#include <vgc/ui/keyevent.h>
#include <vgc/ui/logcategories.h>
#include <vgc/ui/margins.h>
#include <vgc/ui/mouseevent.h>
#include <vgc/ui/scrollevent.h>
#include <vgc/ui/shortcut.h>

namespace vgc::ui {

VGC_DECLARE_OBJECT(Action);
VGC_DECLARE_OBJECT(OverlayArea);
VGC_DECLARE_OBJECT(Widget);
VGC_DECLARE_OBJECT(Window);

// clang-format off

/// \enum vgc::ui::PaintOption
/// \brief Specifies widget paint options.
///
enum class PaintOption : UInt64 {
    None      = 0x00,
    Resizing  = 0x01,
    LayoutViz = 0x02,
};
VGC_DEFINE_FLAGS(PaintOptions, PaintOption)

/// \enum vgc::ui::Visibility
/// \brief Specifies widget visibility.
///
enum class Visibility : bool {

    /// Means "not invisible". A widget with such visibility could also be
    /// "not visible" if any widget in its parent chain is `Invisible`.
    /// A root widget with `Inherit` visibility can be considered visible.
    ///
    Inherit = true,

    /// An `Invisible` widget is explicitly "not visible" and its children
    /// are by inheritance "not visible" either.
    ///
    Invisible = false
};

/// \enum vgc::ui::HandledEventPolicy
/// \brief Specifies widget policy about already handled events.
///
/// Specifies whether the widget wants to receive events in the
/// bubbling phase even if one of their children already handled it.
/// 
enum class HandledEventPolicy : bool {
    Receive,
    Skip
};

// clang-format on

/// \class vgc::ui::Widget
/// \brief Base class of all elements in the user interface.
///
class VGC_UI_API Widget : public style::StylableObject {
private:
    VGC_OBJECT(Widget, style::StylableObject)
    VGC_PRIVATIZE_OBJECT_TREE_MUTATORS

protected:
    /// Constructs a Widget. This constructor is an implementation detail only
    /// available to derived classes. In order to create a Widget, please use
    /// the following:
    ///
    /// ```cpp
    /// WidgetPtr widget = Widget::create();
    /// ```
    ///
    Widget(CreateKey);

    /// Reimplements `Object::onDestroyed()`.
    ///
    /// If you override this method, do not forget to call `onDestroyed()` of your
    /// base class, preferrably at the end.
    ///
    void onDestroyed() override;

public:
    /// Creates a widget.
    ///
    static WidgetPtr create();

    /// Destroys this Widget.
    ///
    /// \sa vgc::core::Object::isAlive().
    ///
    void destroy() {
        destroyObject_();
    }

    /// Returns the parent Widget of this Widget. This can be nullptr
    /// for root widgets.
    ///
    /// \sa firstChild(), lastChild(), previousSibling(), and nextSibling().
    ///
    Widget* parent() const {
        // TODO; the parent widget should actually be a member variable, to be
        // able to correctly handle the case of a root widget owned by another
        // object (that is, the widget would have a non-null parent object, but
        // would still be the root widget).

        core::Object* list = parentObject();
        core::Object* widget = list ? list->parentObject() : nullptr;
        return static_cast<Widget*>(widget);
    }

    /// Returns the first child Widget of this Widget. Returns nullptr if this
    /// Widget has no children.
    ///
    /// \sa lastChild(), previousSibling(), nextSibling(), and parent().
    ///
    Widget* firstChild() const {
        return children_ ? children_->first() : nullptr;
    }

    /// Returns the last child Widget of this Widget. Returns nullptr if this
    /// Widget has no children.
    ///
    /// \sa firstChild(), previousSibling(), nextSibling(), and parent().
    ///
    Widget* lastChild() const {
        return children_ ? children_->last() : nullptr;
    }

    /// Returns the previous sibling of this Widget. Returns nullptr if this
    /// Widget is a root widget, or if it is the first child of its parent.
    ///
    /// \sa nextSibling(), parent(), firstChild(), and lastChild().
    ///
    Widget* previousSibling() const {
        return static_cast<Widget*>(previousSiblingObject());
    }

    /// Returns the next sibling of this Widget. Returns nullptr if this Widget
    /// is a root widget, or if it is the last child of its parent.
    ///
    /// \sa previousSibling(), parent(), firstChild(), and lastChild().
    ///
    Widget* nextSibling() const {
        return static_cast<Widget*>(nextSiblingObject());
    }

    /// Returns all children of this Widget as an iterable range.
    ///
    /// Example:
    ///
    /// \code
    /// for (Widget* child : widget->children()) {
    ///     // ...
    /// }
    /// \endcode
    ///
    WidgetListView children() const {
        return WidgetListView(children_);
    }

    /// Creates a new widget of type `WidgetClass` constructed with
    /// the given arguments `args`, and add it as a child of this widget.
    /// Returns a pointer to the created widget.
    ///
    template<typename WidgetClass, typename... Args>
    WidgetClass* createChild(Args&&... args) {
        core::ObjPtr<WidgetClass> child =
            WidgetClass::create(std::forward<Args>(args)...);
        addChild(child.get());
        return child.get();
    }

    /// Returns the number of child widgets of this widget.
    ///
    Int numChildren() const {
        return children().length();
    }

    /// Adds a child to this widget.
    ///
    void addChild(Widget* child);

    /// Adds the given `child` to this widget children before `nextSibling`.
    ///
    void insertChild(Widget* nextSibling, Widget* child);

    /// Adds the given `child` to this widget children at position `i`.
    ///
    void insertChild(Int i, Widget* child);

    /// Returns whether this Widget can be reparented with the given `newParent`.
    /// See `reparent()` for details.
    ///
    bool canReparent(Widget* newParent);

    /// Moves this widget from its current position in the widget tree to the
    /// last child of the given `newParent`. If `newParent` is already the
    /// current parent of this widget, then this widget is simply moved last
    /// without changing its parent. If `newParent` is `nullptr`, then the
    /// widget becomes a root widget.
    ///
    /// A `ChildCycleError` exception is raised if `newParent` is this widget
    /// itself or one of its descendant.
    ///
    void reparent(Widget* newParent);

    /// Returns whether `replacedWidget` can be replaced by this widget. See `replace()`
    /// for details.
    ///
    bool canReplace(Widget* replacedWidget);

    /// Replaces the given `replacedWidget` with this widget. This destroys
    /// `replacedWidget` and all its descendants, except this widget and all
    /// its descendants. Does nothing if `replacedWidget` is this widget
    /// itself.
    ///
    /// A `NullError` exception is raised if `replacedWidget` is `nullptr`.
    ///
    /// A `ChildCycleError` exception is raised if `replacedWidget` is a
    /// (strict) descendant this widget.
    ///
    void replace(Widget* replacedWidget);

    /// Returns whether this widget is a descendant of the given `other` widget.
    /// Returns true if this widget is equal to the `other` widget.
    ///
    bool isDescendantOf(const Widget* other) const {
        return isDescendantObjectOf(other);
    }

    /// Returns the root widget of this widget, that is, the ancestor of this
    /// widget which has no parent widget itself.
    ///
    /// \sa isRoot()
    ///
    Widget* root() const;

    /// Returns whether this widget is the root widget (i.e., it has no `parent()`).
    ///
    /// \sa root().
    ///
    bool isRoot() const {
        return !parent();
    }

    /// This signal is emitted from the root widget whenever a widget is
    /// added to this widget tree.
    ///
    VGC_SIGNAL(widgetAddedToTree, (Widget*, widget))

    /// This signal is emitted from the root widget whenever a widget is
    /// removed from this widget tree.
    ///
    VGC_SIGNAL(widgetRemovedFromTree, (Widget*, widget))

    /// Returns the furthest ancestor OverlayArea widget of this widget, if it exists.
    /// Otherwise returns nullptr.
    ///
    OverlayArea* topmostOverlayArea() const;

    /// Returns the position of the widget relative to its parent.
    ///
    geometry::Vec2f position() const {
        updateRootGeometry_();
        return position_;
    }

    /// Returns the X coordinate of the widget relative to its parent.
    ///
    float x() const {
        updateRootGeometry_();
        return position_[0];
    }

    /// Returns the Y coordinate of the widget relative to its parent.
    ///
    float y() const {
        updateRootGeometry_();
        return position_[1];
    }

    /// Translates the given `position` from the coordinate system of this widget to
    /// the system of `other`.
    ///
    /// It is fast if `this` is the parent of `other`. Otherwise the operation involves
    /// at most N intermediate operations where N is the tree path length from `this` to
    /// `other` passing through root.
    ///
    geometry::Vec2f mapTo(Widget* other, const geometry::Vec2f& position) const;

    /// Translates the given `rect` from the coordinate system of this widget to
    /// the system of `other`.
    ///
    /// It is fast if `this` is the parent of `other`. Otherwise the operation involves
    /// at most N intermediate operations where N is the tree path length from `this` to
    /// `other` passing through root.
    ///
    geometry::Rect2f mapTo(Widget* other, const geometry::Rect2f& rect) const;

    /// Returns the geometry of the widget relative to its parent.
    ///
    /// This is equivalent to `Rect2f::fromPositionSize(position(), size())`.
    ///
    geometry::Rect2f geometry() const {
        updateRootGeometry_();
        return geometry::Rect2f::fromPositionSize(position_, size_);
    }

    /// Returns the geometry of the widget relative to itself.
    ///
    /// This is equivalent to `Rect2f::fromPositionSize(0, 0, size())`.
    ///
    geometry::Rect2f rect() const {
        updateRootGeometry_();
        return geometry::Rect2f::fromPositionSize(0, 0, size_);
    }

    /// Returns the computed margin of this widget.
    ///
    Margins margin() const;

    /// Returns the computed padding of this widget.
    ///
    Margins padding() const;

    /// Returns the computed border of this widget.
    ///
    Margins border() const;

    /// Returns the content rect of this widget. This is equal to `rect()` with
    /// `borders()` and `padding()` removed.
    ///
    geometry::Rect2f contentRect() const;

    /// Sets the new position and size of this widget (relative to its parent),
    /// then calls updateChildrenGeometry().
    ///
    /// Unless this widget is the root widget, this method should only be
    /// called by the parent of this widget, inside updateChildrenGeometry().
    /// In other words, updateGeometry() calls updateChildrenGeometry(), which
    /// calls updateGeometry() on all its children, etc.
    ///
    /// In order to prevent infinite loops, this function does not
    /// automatically triggers a repaint nor informs the parent widget that the
    /// geometry changed. Indeed, the assumption is that the parent widget
    /// already knows, since it is the object that called this function in the
    /// first place.
    ///
    void updateGeometry(const geometry::Vec2f& position, const geometry::Vec2f& size);
    /// \overload
    void updateGeometry(float x, float y, float width, float height) {
        updateGeometry(geometry::Vec2f(x, y), geometry::Vec2f(width, height));
    }
    /// \overload
    void updateGeometry(const geometry::Rect2f& geometry) {
        updateGeometry(geometry.position(), geometry.size());
    }
    /// \overload (keeps current size)
    void updateGeometry(const geometry::Vec2f& position);
    /// \overload (keeps current size)
    void updateGeometry(float x, float y) {
        updateGeometry(geometry::Vec2f(x, y));
    }
    /// \overload (keeps current position and size)
    void updateGeometry();

    /// Returns the preferred size of this widget, that is, the size that
    /// layout classes will try to assign to this widget. However, note this
    /// widget may be assigned a different size if its size policy allows it to
    /// shrink or grow, or if it is impossible to meet all size constraints.
    ///
    /// If you need to modify the preferred size of a given widget, you can
    /// either change its `preferred-width` or `preferred-height` from auto
    /// (the default value) to a fixed length, or you can create a new Widget
    /// subclass and reimplement computePreferredSize() for finer control.
    ///
    /// Note that this function returns the "computed" preferred size of the
    /// widget. In particular, while preferredWidth() and preferredHeight() may
    /// return "auto", this function always returns actual values based on the
    /// widget content.
    ///
    geometry::Vec2f preferredSize() const {
        updatePreferredSize_();
        return preferredSize_;
    }

    /// Returns the size of the widget.
    ///
    geometry::Vec2f size() const {
        updateRootGeometry_();
        return size_;
    }

    /// Returns the width of the widget.
    ///
    float width() const {
        updateRootGeometry_();
        return size_[0];
    }

    /// Returns the height of the widget.
    ///
    float height() const {
        updateRootGeometry_();
        return size_[1];
    }

    /// Returns the window that contains the widget.
    ///
    Window* window() const {
        return root()->window_;
    }

    /// Returns whether the content of this widget is automatically clipped
    /// to the rect() of the widget, or if instead the content is allowed
    /// to overflow outside the rect().
    ///
    /// By default, isClippingEnabled() is false, that is, it is the
    /// responsability of widgets not to draw outside of their rect().
    ///
    /// \sa `setClippingEnabled()`, `rect()`, `contentRect()`.
    ///
    // XXX use contentRect() instead?
    //
    bool isClippingEnabled() const {
        return isClippingEnabled_;
    }

    /// Sets whether the content of this widget is automatically clipped
    /// to the rect() of the widget.
    ///
    /// by default, isClippingEnabled() is false, so it can be useful to call
    /// `setClippingEnabled(true)` in the constructor of widget subclasses that
    /// want their content automatically clipped, for example, `Canvas`.
    ///
    /// \sa `isClippingEnabled()`
    ///
    void setClippingEnabled(bool isClippingEnabled);

    /// Returns the preferred width of this widget.
    ///
    style::LengthOrPercentageOrAuto preferredWidth() const;

    /// Returns the width stretch factor of this widget. This value is never negative.
    ///
    float horizontalStretch() const;

    /// Returns the width shrink factor of this widget. This value is never negative.
    ///
    float horizontalShrink() const;

    /// Returns the preferred height of this widget.
    ///
    style::LengthOrPercentageOrAuto preferredHeight() const;

    /// Returns the preferred width of the widget for a given height.
    ///
    /// By default, this returns `preferredSize()[0]`, regardless of the given
    /// height. Re-implement this function in subclasses if the preferred width
    /// of your widget depends on its height.
    ///
    virtual float preferredWidthForHeight(float height) const;

    /// Returns the preferred height of the widget for a given width.
    ///
    /// By default, this returns `preferredSize()[1]`, regardless of the given
    /// width. Re-implement this function in subclasses if the preferred height
    /// of your widget depends on its width.
    ///
    virtual float preferredHeightForWidth(float width) const;

    /// Returns the height stretch factor of this widget.
    ///
    float verticalStretch() const;

    /// Returns the height shrink factor of this widget.
    ///
    float verticalShrink() const;

    /// Returns whether a geometry update request is pending for this widget.
    ///
    bool isGeometryUpdateRequested() const {
        return isGeometryUpdateRequested_;
    }

    /// This method should be called when the size policy or preferred size of
    /// this widget changed, to inform its parent that its geometry should be
    /// recomputed.
    ///
    void requestGeometryUpdate();

    /// This signal is emitted if:
    ///
    /// 1. this widget is a root widget, and
    ///
    /// 2. this widget or any of its descendants requested its geometry to be
    /// updated via `requestGeometryUpdate()`.
    ///
    /// This signal will never be re-emitted as long as updateGeometry() is not
    /// called.
    ///
    /// If this signal is emitted, this means that some layout recomputation is
    /// required somewhere in the tree, and in particular that
    /// `preferredSize()` may have changed.
    ///
    /// This signal should typically be listened to by the owner of the widget
    /// tree, for example a `Window`, or a third-party widget tree (e.g., Qt)
    /// embedding a `vgc::ui::Widget` subtree. The typical response to the
    /// signal is to determine an appropriate new position and size of the
    /// widget based on its new `preferredSize()`, then call
    /// `updateGeometry(newPosition, newSize)`.
    ///
    /// Note that it is best practice to defer the call to
    /// `updateGeometry(newPosition, newSize)` until you actually need to
    /// repaint the widget, in order to avoid multiple layout recomputations
    /// between two consecutive repaints. If the position and size of the
    /// widget shouldn't change, you can even skip calling `updateGeometry()`
    /// entirely, since the `paint()` method does it automatically if a
    /// geometry update was requested, but `updateGeometry()` hasn't been
    /// called yet.
    ///
    VGC_SIGNAL(geometryUpdateRequested)

    /// This virtual function is called each time the widget is resized. When
    /// this function is called, the widget already has its new size.
    ///
    virtual void onResize();

    /// Requests this widget to be repainted, for example because the data
    /// displayed by this widget has changed. The widget is not immediately
    /// repainted: it is only scheduled for repaint. The actual moment when the
    /// widget is repainted depends on the graphics::Engine and other
    /// platform-dependent factors.
    ///
    /// You should typically call this function in your event handlers (e.g.,
    /// mousePressEvent()), to notify that the appearance of this widget has
    /// changed as a result of the event. Such call can be indirect, below is a
    /// example scenario:
    ///
    /// 1. The user clicks on an "Add Circle" button.
    /// 2. The event handler of the button emits the "clicked" signal.
    /// 3. A listener of this signal calls scene->addCircle().
    /// 4. This modifies the scene, which emits a "changed" signal.
    /// 5. A view of the scene detects the change, and calls this->requestRepaint().
    ///
    /// Note how in this scenario, the repainted view is unrelated to the
    /// button which initially handled the event.
    ///
    void requestRepaint();

    /// This signal is emitted if:
    ///
    /// 1. this widget is a root widget, and
    ///
    /// 2. this widget or any of its descendants requested to be repainted,
    /// either directly via `requestRepaint()`, or indirectly via
    /// `requestGeometryUpdate()`.
    ///
    /// This signal will never be re-emitted as long as paint() is not
    /// called.
    ///
    /// This signal should typically be listened to by the owner of the widget
    /// tree, for example a `Window`, or a third-party widget tree (e.g., Qt)
    /// embedding a `vgc::ui::Widget` subtree. The typical response to the
    /// signal is to perform some graphics engine initialization then call
    /// `Widget::paint()` whenever possible. In a multithreaded rendering
    /// architecture, this could mean as soon as the current render (if any) is
    /// finished, or perhaps until the next V-Sync.
    ///
    VGC_SIGNAL(repaintRequested)

    /// Initiates time-consuming precomputation of graphics resources that are
    /// required for painting this widget.
    ///
    /// This function does the following, in this order:
    ///
    /// 1. Ensures that this widget's geometry is up to date.
    ///
    /// 2. Checks whether the engine changed since the last paint, in which case
    ///    `onPaintDestroy(oldEngine)` and `onPaintCreate(newEngine)` are called.
    ///
    /// 3. Calls `onPaintPrepare()`.
    ///
    /// 4. Calls `child->preparePaint()` for all children of this widget.
    ///
    void preparePaint(graphics::Engine* engine, PaintOptions options = PaintOption::None);

    /// Paints this widget.
    ///
    /// This function does the following, in this order:
    ///
    /// 1. Ensures that this widget's geometry is up to date.
    ///
    /// 2. Checks whether the engine changed since the last paint, in which case
    ///    `onPaintDestroy(oldEngine)` and `onPaintCreate(newEngine)` are called.
    ///
    /// 3. Setup the engine's scissor rect, if `isClippingEnabled()` is `true`.
    ///
    /// 4. Calls `onPaintDraw()`.
    ///
    /// This function does nothing if `isVisible()` if `false`.
    ///
    /// If this widget is a root widget, then this function should typically be
    /// called by its owner when the `repaintRequested` signal is emitted.
    ///
    /// If this widget has a parent widget, then this function should typically
    /// be called in the `onPaintDraw()` implementation of its parent. This is
    /// in fact done in the default implementation of `onPaintDraw()`, which
    /// calls `paintChildren()`, which in turn calls `child->paint()` on all
    /// visible children.
    ///
    void paint(graphics::Engine* engine, PaintOptions options = PaintOption::None);

    /// Returns the `background-color` style attribute of this widget.
    ///
    /// This is equivalent to:
    ///
    /// ```cpp
    /// style(graphics::strings::background_color).to<core::Color>()
    /// ```
    ///
    const core::Color& backgroundColor() {
        return backgroundColor_;
    }

    /// Returns the widget policy regarding handled events.
    ///
    /// The default value is HandledEventPolicy::Skip.
    ///
    /// \sa HandledEventPolicy.
    ///
    HandledEventPolicy handledEventPolicy() const {
        return handledEventPolicy_;
    }

    /// Sets the widget policy regarding handled events.
    ///
    /// The default value is HandledEventPolicy::Skip.
    ///
    /// \sa HandledEventPolicy.
    ///
    void setHandledEventPolicy(HandledEventPolicy handledEventPolicy) {
        handledEventPolicy_ = handledEventPolicy;
    }

    /// Starts capturing the mouse.
    ///
    /// After calling this method, and until `stopMouseCapture()` is called,
    /// all system mouse events are redirected to this widget instead of any
    /// other widgets or applications.
    ///
    /// Calling this function is a system-wide behavior that prevents the user
    /// from interacting with other applications, so it should be used with
    /// extreme care.
    ///
    /// \sa stopMouseCapture(), mouseCaptor(), mouseCaptureStarted(),
    ///     startKeyboardCapture().
    ///
    void startMouseCapture();

    /// Stops the mouse capture.
    ///
    /// Note that this method has no effect if this widget is not the
    /// `mouseCaptor()`. If you want to stop the mouse capture from another
    /// widget, you must call `mouseCaptor()->stopMouseCapture()`.
    ///
    /// \sa startMouseCapture(), mouseCaptor(), mouseCaptureStopped(),
    ///     stopKeyboardCapture().
    ///
    void stopMouseCapture();

    /// This signal is emitted if:
    ///
    /// 1. this widget is a root widget, and
    ///
    /// 2. this widget or any of its descendants called `startMouseCapture()`
    ///
    /// This signal should typically be listened to by the owner of the widget
    /// tree, for example a `Window`, or a third-party widget tree (e.g., Qt)
    /// embedding a `vgc::ui::Widget` subtree. The typical response is to
    /// redirect all mouse events to the `mouseCaptor()` instead of the root
    /// widget.
    ///
    /// \sa mouseCaptureStopped(), startMouseCapture(), keyboardCaptureStarted().
    ///
    VGC_SIGNAL(mouseCaptureStarted)

    /// This signal is emitted if:
    ///
    /// 1. this widget is a root widget, and
    ///
    /// 2. this widget or any of its descendants called `stopMouseCapture()`
    ///
    /// This signal should typically be listened to by the owner of the widget
    /// tree, for example a `Window`, or a third-party widget tree (e.g., Qt)
    /// embedding a `vgc::ui::Widget` subtree. The typical response is to stop
    /// redirecting all mouse events to the `mouseCaptor()`.
    ///
    /// \sa mouseCaptureStarted(), stopMouseCapture(), keyboardCaptureStopped().
    ///
    VGC_SIGNAL(mouseCaptureStopped)

    /// Returns the widget that captures the mouse, if any.
    ///
    /// \sa startMouseCapture(), stopMouseCapture(), keyboardCaptor().
    ///
    Widget* mouseCaptor() const {
        return root()->mouseCaptor_;
    }

    /// Starts capturing the keyboard.
    ///
    /// After calling this method, and until `stopKeyboardCapture()` is called,
    /// all system keyboard events are redirected to this widget instead of any
    /// other widgets or applications.
    ///
    /// Calling this function is a system-wide behavior that prevents the user
    /// from interacting with other applications, so it should be used with
    /// extreme care.
    ///
    /// \sa stopKeyboardCapture(), keyboardCaptor(), keyboardCaptureStarted(),
    ///     startMouseCapture().
    ///
    void startKeyboardCapture();

    /// Stops the keyboard capture.
    ///
    /// Note that this method has no effect if this widget is not the
    /// `keyboardCaptor()`. If you want to stop the keyboard capture from
    /// another widget, you must call `keyboardCaptor()->stopKeyboardCapture()`.
    ///
    /// \sa startKeyboardCapture(), keyboardCaptor(), keyboardCaptureStopped(),
    ///     stopMouseCapture().
    ///
    void stopKeyboardCapture();

    /// This signal is emitted if:
    ///
    /// 1. this widget is a root widget, and
    ///
    /// 2. this widget or any of its descendants called `startKeyboardCapture()`
    ///
    /// This signal should typically be listened to by the owner of the widget
    /// tree, for example a `Window`, or a third-party widget tree (e.g., Qt)
    /// embedding a `vgc::ui::Widget` subtree. The typical response is to
    /// redirect all keyboard events to the `keyboardCaptor()` instead of the root
    /// widget.
    ///
    /// \sa keyboardCaptureStopped(), startKeyboardCapture(), mouseCaptureStarted().
    ///
    VGC_SIGNAL(keyboardCaptureStarted)

    /// This signal is emitted if:
    ///
    /// 1. this widget is a root widget, and
    ///
    /// 2. this widget or any of its descendants called `stopKeyboardCapture()`
    ///
    /// This signal should typically be listened to by the owner of the widget
    /// tree, for example a `Window`, or a third-party widget tree (e.g., Qt)
    /// embedding a `vgc::ui::Widget` subtree. The typical response is to stop
    /// redirecting all keyboard events to the `keyboardCaptor()`.
    ///
    /// \sa keyboardCaptureStarted(), stopKeyboardCapture(), mouseCaptureStopped().
    ///
    VGC_SIGNAL(keyboardCaptureStopped)

    /// Returns the widget the captures the keyboard, if any.
    ///
    /// \sa startKeyboardCapture(), stopKeyboardCapture(), mouseCaptor().
    ///
    Widget* keyboardCaptor() const {
        return root()->keyboardCaptor_;
    }

    /// Propagates a mouse move event through the widget hierarchy.
    ///
    /// It can only be called on the root widget.
    ///
    bool mouseMove(MouseMoveEvent* event);

    /// Propagates a mouse press event through the widget hierarchy.
    ///
    /// It can only be called on the root widget.
    ///
    bool mousePress(MousePressEvent* event);

    /// Propagates a mouse release event through the widget hierarchy.
    ///
    /// It can only be called on the root widget.
    ///
    bool mouseRelease(MouseReleaseEvent* event);

    /// Propagates a mouse scroll event through the widget hierarchy.
    ///
    /// It can only be called on the root widget.
    ///
    bool mouseScroll(ScrollEvent* event);

    /// Override this function if you wish to handle MouseMove events during the
    /// capture phase (from root to leaf).
    ///
    /// The hover-chain child is automatically updated (if it is not hover-locked)
    /// prior to calling this function using `computeHoverChainChild()`.
    /// However you can change it again in this function using `setHoverChainChild()`.
    /// This let's you override a hover-locked hover-chain child.
    ///
    virtual void preMouseMove(MouseMoveEvent* event);

    /// Override this function if you wish to handle MousePress events during the
    /// capture phase (from root to leaf).
    ///
    virtual void preMousePress(MousePressEvent* event);

    /// Override this function if you wish to handle MouseRelease events during the
    /// capture phase (from root to leaf).
    ///
    virtual void preMouseRelease(MouseReleaseEvent* event);

    /// Override this function if you wish to handle MouseMove events. You must
    /// return true if the event was handled, false otherwise.
    ///
    virtual bool onMouseMove(MouseMoveEvent* event);

    /// Override this function if you wish to handle MousePress events. You
    /// must return true if the event was handled, false otherwise.
    ///
    virtual bool onMousePress(MousePressEvent* event);

    /// Override this function if you wish to handle MouseRelease events. You
    /// must return true if the event was handled, false otherwise.
    ///
    virtual bool onMouseRelease(MouseReleaseEvent* event);

    /// Override this function if you wish to handle MouseScroll events. You
    /// must return true if the event was handled, false otherwise.
    ///
    virtual bool onMouseScroll(ScrollEvent* event);

    /// Returns whether this widget children are allowed to be hovered or not.
    ///
    bool isChildHoverEnabled() const {
        return isChildHoverEnabled_;
    }

    /// Sets whether this widget children are allowed to be hovered or not.
    ///
    /// It is reset to true whenever the widget is hover-unlocked.
    ///
    void setChildHoverEnabled(bool enabled) {
        isChildHoverEnabled_ = enabled;
    }

    /// Returns whether this widget is hovered.
    ///
    /// During a mouse event sequence handled by a target widget, the chain
    /// of hovered widgets (the hover-chain) can be locked.
    ///
    bool isHovered() const {
        return isHovered_;
    }

    /// Sets the hovered state of this widget and calls `onMouseEnter` or
    /// `onMouseLeave` accordingly. Its children and the hovered chain are
    /// also updated.
    ///
    /// Returns true on success.
    ///
    bool setHovered(bool hovered);

    /// Returns which "child" widget (if any), should be considered hovered as
    /// a result of the given mouse `event`.
    ///
    /// The returned widget is typically an actual child widget of this widget,
    /// but it can also be another related widget that can be semantically
    /// considered a child for the purpose of mouse hover.
    ///
    /// The default implementation returns `nullptr` if `isChildHoverEnabled()`
    /// is false, otherwise returns the last child widget (that is, painted
    /// last), such that `computeIsHovered()` is true, after mapping to event
    /// to the local coordinates of the child.
    ///
    /// You can override this method if you wish this widget to have a
    /// non-standard method of deciding which of its children is hovered. For
    /// example, `PopupLayer` overrides this method to let mouse events "pass
    /// through" its underlying widget, and `PanelArea` overrides this method
    /// to ensure that no subpanels are hovered if the mouse is within
    /// `handle-size` of a boundary between two subpanels.
    ///
    /// This method is called by the default implementation of
    /// `updateHoverChainChild()`. Therefore, if you wish even more control,
    /// you can override `updateHoverChainChild()` instead..
    ///
    virtual Widget* computeHoverChainChild(MouseHoverEvent* event) const;

    /// Returns whether this widget should be considered hovered as a result
    /// of the given mouse `event`.
    ///
    /// The default implementation returns true if and only if both
    /// `isVisible()` and `rect().contains(event->position())` are true.
    ///
    /// You can override this function if you wish this widget to have a
    /// non-standard method of deciding whether it is hovered. For example,
    /// `Tooltip` overrides this method to always return false since it
    /// desires all mouse events to always pass through itself.
    ///
    virtual bool computeIsHovered(MouseHoverEvent* event) const;

    /// If `this` is hovered, it makes the given `newHoverChainChild` the
    /// hover-chain child of `this`.
    ///
    /// Returns true on success. That is, if `this` is now the hover-chain parent
    /// of `newHoverChainChild` and both widgets in question are still alive.
    ///
    bool setHoverChainChild(Widget* newHoverChainChild);

    /// Returns the widget that is hovered inside this widget.
    /// If this widget is not hovered, it does not have an hover-chain child.
    ///
    /// The returned widget is not always a child widget of this widget.
    ///
    Widget* hoverChainChild() const {
        return hoverChainChild_;
    }

    /// Returns the widget that this widget is hovered inside of.
    /// If this widget is not hovered, it does not have an hover-chain parent.
    ///
    /// The returned widget is not always the parent widget of this widget.
    ///
    Widget* hoverChainParent() const {
        return hoverChainParent_;
    }

    /// Returns whether this widget is hover-locked.
    ///
    /// A widget that is hover-locked will keep its place in the hover-chain
    /// at least until the next mouse event.
    ///
    /// It allows a widget to keep receiving mouse events as long as it wants
    /// as well as keep the widgets in the hover-chain in a mouse entered state.
    /// However a hover-locked widget has to delegate handling to its hover-chain
    /// child first if it is itself hover-locked.
    ///
    bool isHoverLocked() const {
        return isHoverLocked_;
    }

    bool isInHoveredRow() const {
        return isInHoveredRow_;
    }

    void setInHoveredRow(bool isInHoveredRow) {
        isInHoveredRow_ = isInHoveredRow;
    }

    bool isInHoveredColumn() const {
        return isInHoveredColumn_;
    }

    void setInHoveredColumn(bool isInHoveredColumn) {
        isInHoveredColumn_ = isInHoveredColumn;
    }

    const MouseButtons& pressedButtons() const {
        return pressedButtons_;
    }

    /// Returns the visibility of this widget.
    ///
    /// See `Visibility` enumerators for more details.
    ///
    Visibility visibility() const {
        return visibility_;
    }

    /// Sets the visibility of this widget.
    ///
    /// See `Visibility` enumerators for more details.
    ///
    void setVisibility(Visibility visibility);

    /// Returns the computed visibility. It is true only if all widgets
    /// up to the root have `visibility()` set to `Visibility::Inherit`;
    ///
    bool isVisible() const {
        return computedVisibility_;
    }

    /// Sets the visibility to `Inherit`.
    ///
    /// Note that this does not imply `isVisible() == true` after being called.
    ///
    /// If this causes `isVisible()` to change from `false` to `true`, then
    /// `onVisible()` is called.
    ///
    /// \sa onVisible(), isVisible()
    ///
    void show() {
        setVisibility(Visibility::Inherit);
    }

    /// Sets the visibility to `Invisible`.
    ///
    /// This implies `isVisible() == false` after being called.
    ///
    /// If this causes `isVisible()` to change from `true` to `false`, then
    /// `onHidden()` is called.
    ///
    /// \sa onHidden(), isVisible()
    ///
    void hide() {
        setVisibility(Visibility::Invisible);
    }

    /// Returns whether this widget tree is active, that is, whether it
    /// receives key press and focus events.
    ///
    /// More precisely, when the widget tree is active, then one FocusOut event
    /// and one FocusIn event are sent whenever the focused widget changes.
    ///
    /// On the contrary, when the widget tree is not active, then no FocusIn
    /// and FocusOut events are sent.
    ///
    /// A FocusIn event is sent to the focused widget (if any) when the tree
    /// becomes active.
    ///
    /// A FocusOut event is sent to the focused widget (if any) when the tree
    /// becomes inactive.
    ///
    /// \sa setTreeActive()
    ///
    bool isTreeActive() const {
        return root()->isTreeActive_;
    }

    /// Sets whether this widget tree is active.
    ///
    /// \sa isTreeActive()
    ///
    void setTreeActive(bool active, FocusReason reason);

    /// Returns the focus policy of this widget.
    ///
    FocusPolicyFlags focusPolicy() const {
        return focusPolicy_;
    }

    /// Sets the focus policy of this widget.
    ///
    void setFocusPolicy(FocusPolicyFlags policy) {
        focusPolicy_ = policy;
    }

    /// Returns the focus strength of this widget.
    ///
    FocusStrength focusStrength() const {
        return focusStrength_;
    }

    /// Sets the focus strength of this widget.
    ///
    void setFocusStrength(FocusStrength strength) {
        focusStrength_ = strength;
    }

    /// Makes this widget the focused widget of this widget tree, and emits the
    /// `focusSet()` signal.
    ///
    /// If the tree is active, then this widget will now receive keyboard
    /// events.
    ///
    /// If this widget was not already in the `focusStack()`, then it is added
    /// to the stack and `onFocusStackIn()` is called.
    ///
    /// If the tree is active and if this widget was not already the focused
    /// widget, then this widget will receive a FocusIn event.
    ///
    /// Note that if this widget was already the focused widget, then typically
    /// it will not receive a FocusIn event. However, the `focusSet()` signal
    /// is still emitted, and as a result the tree may switch from inactive to
    /// active, in which case this widget will in fact indirectly receive a
    /// FocusIn event.
    ///
    /// After calling this function, all ancestors of this widget become part
    /// of what is called the "focus branch": it is the branch that goes from
    /// the root of the widget tree to the focused widget. For all widgets in
    /// this branch (including the focused widget), `hasFocusedWidget()`
    /// returns true.
    ///
    /// \sa `isTreeActive()`, `clearFocus()`.
    ///
    void setFocus(FocusReason reason);

    /// This signal is emitted whenever this widget, or any of its descendants,
    /// became the focused widget.
    ///
    VGC_SIGNAL(focusSet, (FocusReason, reason))

    /// Removes this widget or any of its descendants from the `focusStack()`.
    ///
    /// If this causes any widget to be removed from the stack, then
    /// `onFocusStackOut()` will be called on these widgets.
    ///
    /// If this causes the current `focusedWidget()` to not be the focused
    /// widget anymore, and if the tree is active, then the focused widget will
    /// receive a `FocusOut` event, and from now on will not receive keyboard
    /// events. The new focused widget, if any, will receive a `FocusIn` event.
    ///
    /// \sa `setFocus()`.
    ///
    void clearFocus(FocusReason reason);

    /// This signal is emitted whenever this widget, or any of its descendants,
    /// was the focused widget but isn't anymore.
    ///
    VGC_SIGNAL(focusCleared, (FocusReason, reason))

    /// Returns the focus stack of this widget tree.
    ///
    /// '
    core::Array<WidgetPtr> focusStack() const;

    /// Returns the focused widget of this widget tree, if any.
    ///
    /// This is the last widget of the `focusStack()`.
    ///
    Widget* focusedWidget() const;

    /// Returns whether this widget is the focused widget.
    ///
    bool isFocusedWidget() const {
        return focusedWidget() == this;
    }

    /// Returns whether the focused widget is this widget or any of its descendants.
    ///
    /// If you wish to know whether the whole widget tree has a focused widget,
    /// you can call root()->hasFocusedWidget();
    ///
    bool hasFocusedWidget() const {
        Widget* focusedWidget_ = focusedWidget();
        return focusedWidget_ && focusedWidget_->isDescendantOf(this);
    }

    /// Returns whether this widget is the focused widget, and this widget tree
    /// is active.
    ///
    bool hasFocus() const {
        return isFocusedWidget() && isTreeActive();
    }

    /// Returns whether this widget or any of its descendants is the focused
    /// widget, and this widget tree is active.
    ///
    bool hasFocusWithin() const {
        return hasFocusedWidget() && isTreeActive();
    }

    /// This function is called when:
    /// 1. isTreeActive() is true and the focused widget changed, or
    /// 2. isTreeActive() changed from false to true
    ///
    /// Note that this function is only called for the focused widget itself,
    /// not for all its ancestors.
    ///
    /// \sa `onFocusOut()`, `setFocus()`, `clearFocus()`, `isTreeActive()`.
    ///
    virtual void onFocusIn(FocusReason reason);

    /// This function is called when:
    /// 1. isTreeActive() is true and the focused widget changed, or
    /// 2. isTreeActive() changed from true to false
    ///
    /// This function is called just after the widget loses focus, so you can rely
    /// on the return value of `isFocusedWidget()` to determine whether this widget is
    /// still the focused widget despite losing focus (i.e., situation 2. above).
    ///
    /// Note that this function is only called for the focused widget itself,
    /// not for all its ancestors.
    ///
    /// \sa `onFocusIn()`, `setFocus()`, `clearFocus()`, `isTreeActive()`.
    ///
    virtual void onFocusOut(FocusReason reason);

    /// This function is called when a widget is added to the focus stack.
    ///
    /// Note that this function is only called for the widget itself, not for
    /// all its ancestors.
    ///
    /// \sa `onFocusStackOut()`, `focusStack()`.
    ///
    virtual void onFocusStackIn(FocusReason reason);

    /// This function is called when a widget is removed from the focus stack.
    ///
    /// Note that this function is only called for the widget itself, not for
    /// all its ancestors.
    ///
    /// \sa `onFocusStackInt()`, `focusStack()`.
    ///
    virtual void onFocusStackOut(FocusReason reason);

    /// Returns whether this widget accepts text input.
    ///
    /// \sa `setTextInputReceiver()`.
    ///
    bool isTextInputReceiver() const {
        return isTextInputReceiver_;
    }

    /// Sets whether this widget accepts text input.
    ///
    /// This should be set to `true` for widgets that behave like text editors
    /// (e.g., `LineEdit`), so that dead keys can be combined with the
    /// follow-up characters before being transmitted to the widget, and so
    /// that virtual keyboards can appear in contexts where it makes sense
    /// (e.g., mobile operating systems, Chinese input, accessibility
    /// settings).
    ///
    /// \sa `isTextInputReceiver()`.
    ///
    void setTextInputReceiver(bool isTextInputReceiver);

    /// This signal is emitted whenever the value of `isTextInputReceiver()`
    /// changes.
    ///
    VGC_SIGNAL(textInputReceiverChanged, (bool, newValue))

    /// Propagates a key press event through the widget hierarchy.
    ///
    /// It can only be called on the root widget.
    ///
    bool keyPress(KeyPressEvent* event);

    /// Propagates a key release event through the widget hierarchy.
    ///
    /// It can only be called on the root widget.
    ///
    bool keyRelease(KeyReleaseEvent* event);

    /// Override this function if you wish to handle KeyPress events during the
    /// capture phase (from root to leaf).
    ///
    virtual void preKeyPress(KeyPressEvent* event);

    /// Override this function if you wish to handle KeyRelease events during the
    /// capture phase (from root to leaf).
    ///
    virtual void preKeyRelease(KeyReleaseEvent* event);

    /// Override this function if you wish to handle key press events. You must
    /// return true if the event was handled, false otherwise.
    ///
    virtual bool onKeyPress(KeyPressEvent* event);

    /// Override this function if you wish to handle key release events. You must
    /// return true if the event was handled, false otherwise.
    ///
    virtual bool onKeyRelease(KeyReleaseEvent* event);

    /// Returns the list of actions of this widget.
    ///
    ActionListView actions() const {
        return ActionListView(actions_);
    }

    /// Creates an action of type `TAction`, adds it to this widget, and
    /// returns the action.
    ///
    template<typename TAction, typename... Args>
    TAction* createAction(Args&&... args) {
        core::ObjPtr<TAction> action = TAction::create(std::forward<Args>(args)...);
        TAction* action_ = action.get();
        addAction(action_);
        return action_;
    }

    /// Adds the given `action` to the list of actions of this widget.
    ///
    /// The widget takes ownership of the action.
    ///
    /// If the action previously belonged to another widget, it is first removed
    /// from the other widget.
    ///
    void addAction(Action* action);

    /// Removes the given `action` from the list of actions of this widget.
    ///
    void removeAction(Action* action);

    /// Clears the list of actions of this widget.
    ///
    void clearActions();

    /// Creates an action of type `ActionType::Trigger`, adds it to this
    /// widget, and returns the action.
    ///
    template<typename... Args>
    Action* createTriggerAction(Args&&... args) {
        return createAction<Action>(std::forward<Args>(args)...);
    }

    /// This signal is emitted whenever an action is added to this widget.
    ///
    VGC_SIGNAL(actionAdded, (Action*, addedAction));

    /// This signal is emitted whenever an action is removed from this widget.
    ///
    VGC_SIGNAL(actionRemoved, (Action*, removedAction));

    // Implementation of StylableObject interface
    static void populateStyleSpecTable(style::SpecTable* table);
    void populateStyleSpecTableVirtual(style::SpecTable* table) override {
        populateStyleSpecTable(table);
    }

    /// Returns the current mouse drag action, if any.
    ///
    Action* currentMouseDragAction() const {
        return root()->currentMouseDragAction_.getIfAlive();
    }

protected:
    // Reimplementation of Object virtual methods.
    void onChildRemoved(Object* child) override;

    /// Useful for onMousePress
    void setPressedButtons(const MouseButtons& buttons) {
        pressedButtons_ = buttons;
    }

    // Reimplementation of StylableObject virtual methods.
    void onStyleChanged() override;

    /// Override this function if you wish to handle the reparenting of
    /// this widget under a new parent widget.
    ///
    virtual void onParentWidgetChanged(Widget* newParent);

    /// Override this function if you wish to handle the addition of
    /// child widgets to this widget.
    ///
    /// `wasOnlyReordered` is true if it was already a child but got reordered.
    ///
    virtual void onWidgetAdded(Widget*, bool wasOnlyReordered);

    /// Override this function if you wish to handle the removal of
    /// child widgets from this widget.
    ///
    virtual void onWidgetRemoved(Widget*);

    /// This event handler is called whenever the widget is hovered by the
    /// mouse at the position given by `event->position()`. This can happen in
    /// the following situations:
    ///
    /// - The mouse moved.
    /// - A mouse button was released, completing a mouse drag action,
    ///   therefore updating which widget is now hovered.
    /// - The window gained focus, for example after Alt-Tab.
    ///
    virtual void onMouseHover(MouseHoverEvent* event);

    /// Override this function if you wish to handle MouseEnter events. You
    /// must return true if the event was handled, false otherwise.
    ///
    virtual void onMouseEnter();

    /// Override this function if you wish to handle MouseLeave events. You
    /// must return true if the event was handled, false otherwise.
    ///
    virtual void onMouseLeave();

    /// Override this function if you wish to handle the update of your
    /// hover-chain child yourself.
    ///
    /// Returns true on success.
    ///
    /// This method is used by the mouse event system when it needs to update
    /// the hover chain child (on all mouse events if there is no
    /// hover-locked hover-chain child).
    ///
    virtual bool updateHoverChainChild(MouseHoverEvent* event);

    /// Update the hover-chain starting from this widget.
    /// Returns whether the hover-chain changed or not.
    ///
    bool updateHoverChain();

    /// Override this function if you wish to do something when the widget
    /// becomes visible. "Visible" here means not invisible nor invisible by
    /// inheritance, and it can still be occluded.
    ///
    virtual void onVisible();

    /// Override this function if you wish to do something when the widget
    /// becomes hidden. "Hidden" here means invisible or invisible by
    /// inheritance. An occluded widget is not hidden.
    ///
    virtual void onHidden();

    /// Computes the preferred size of this widget based on its size policy, as
    /// well as its content and the preferred size and size policy of its
    /// children.
    ///
    /// For example, the preferred size of Row class is computed based on
    /// the preferred size of its children, and the preferred size of Label is
    /// computed based on the length of the text.
    ///
    /// If you reimplement this method, make sure to check whether the
    /// widthPolicy() or heightPolicy() of this widget is different from
    /// PreferredSizeType::Auto, in which case this function should return the
    /// specified fixed value.
    ///
    /// Note that if, for example, widthPolicy() is a fixed value, but
    /// heightPolicy() is Auto, then the preferred height may depend on the
    /// value of the fixed width.
    ///
    virtual geometry::Vec2f computePreferredSize() const;

    /// Updates the position and size of children of this widget (by calling
    /// the updateGeometry() methods of the children), based on the current
    /// width and height of this widget.
    ///
    virtual void updateChildrenGeometry();

    /// This function is called once before the first call to
    /// `onPaintPrepare()` or `onPaintDraw()` for any given `engine`.
    ///
    /// You can override this function to initialize any GPU resources needed
    /// for painting.
    ///
    /// If you override this function, you must call the base implementation at
    /// the start of your implementation, via `SuperClass::onPaintCreate(engine)`.
    ///
    /// The default implementation creates the GPU resources needed for all
    /// widgets, for example those needed to paint the widget's background.
    ///
    virtual void onPaintCreate(graphics::Engine* engine);

    /// This function is called before `onPaintDraw()`.
    ///
    /// You can override this function to initiate time-consuming
    /// precomputation of graphics resources that are required for painting
    /// this widget. For example, you can start a computation in a separate
    /// thread in `onPaintPrepare()`, then in `onPaintDraw()`, if the
    /// computation is not finished, you could choose either to wait for the
    /// computation to be finished, or decide to draw an approximation.
    ///
    /// If you override this function, you must call the base implementation at
    /// the start of your implementation, via
    /// `SuperClass::onPaintPrepare(engine, options)`.
    ///
    /// The default implementation does nothing.
    ///
    virtual void onPaintPrepare(graphics::Engine* engine, PaintOptions options);

    /// This function is called when the widget needs to be repainted.
    ///
    /// You can override this function to perform your paint operations.
    ///
    /// The default implementation is:
    ///
    /// ```
    /// void Widget::onPaintDraw(graphics::Engine* engine, PaintOptions options) {
    ///     paintBackground();
    ///     paintChildren();
    /// }
    /// ```
    ///
    /// If you override this function, then depending on your needs you can:
    ///
    /// 1. Call the base implementation `SuperClass::onPaintDraw(engine, options)`
    ///    at the start of your implementation, which is useful to draw overlays
    ///    over children, or
    ///
    /// 2. Call the base implementation `SuperClass::onPaintDraw(engine, options)`
    ///    at the end of your implementation, which is useful if you know that
    ///    there is no background and want to draw something before drawing
    ///    children, or
    ///
    /// 3. Decide not to call the base implementation at all and perform everything
    ///    yourself, e.g., to draw something between the background and the children,
    ///    or draw the children in a specific order, or some other custom needs.
    ///
    virtual void onPaintDraw(graphics::Engine* engine, PaintOptions options);

    /// This function is called once after the last call to `onPaintDraw()` for
    /// any given `engine`, for example before the widget is destructed, or if
    /// switching graphics engine.
    ///
    /// You can override this function to destroy any GPU resources that you have
    /// previously initialized in `onPaintCreate()`, `onPaintPrepare()`, or
    /// `onPaintDraw()`.
    ///
    /// If you override this function, you must call the base implementation at
    /// the start of your implementation, via
    /// `SuperClass::onPaintDestroy(engine, options)`.
    ///
    virtual void onPaintDestroy(graphics::Engine* engine);

    /// Paints the background of this widget.
    ///
    /// This function is called in the default implementation of
    /// `onPaintDraw()`, and you may also want to call it in your own
    /// reimplementations if you decide not to call the base implementation.
    ///
    void paintBackground(graphics::Engine* engine, PaintOptions options);

    /// Paints all visible children of this widget, by setting up their
    /// appropriate view matrix and calling `child->paint(engine, options)`.
    ///
    void paintChildren(graphics::Engine* engine, PaintOptions options);

private:
    friend Window;

    WidgetList* children_ = nullptr;
    ActionList* actions_ = nullptr;
    Window* window_ = nullptr;

    bool isReparentingWithinSameTree_ = false;

    void onWidgetAdded_(Widget* widget, bool wasOnlyReordered);
    void onWidgetRemoved_(Widget* widget);

    void onActionAdded_(Action* action, bool wasOnlyReordered);
    void onActionRemoved_(Action* action);

    // Layout
    mutable geometry::Vec2f preferredSize_ = {};
    mutable bool isPreferredSizeComputed_ = false;
    mutable bool isGeometryUpdateRequested_ = false;
    bool isGeometryUpdateOngoing_ = false;
    bool isRepaintRequested_ = false;
    bool isClippingEnabled_ = false;
    geometry::Vec2f position_ = {};
    geometry::Vec2f size_ = {};
    geometry::Vec2f lastResizeEventSize_ = {};
    // XXX only valid for root. to be moved to WidgetTree data.
    geometry::Vec2f lastMousePosition_ = {};
    ModifierKeys lastModifierKeys_ = {};
    double lastTimestamp_ = 0;

    void resendPendingRequests_();

    void updatePreferredSize_() const {
        if (!isPreferredSizeComputed_) {
            preferredSize_ = computePreferredSize();
            float w = preferredSize_[0];
            float h = preferredSize_[1];
            if (w < 0) {
                VGC_WARNING(LogVgcUi, "Computed preferred width ({}) is negative", w);
                preferredSize_[0] = 0;
            }
            if (h < 0) {
                VGC_WARNING(LogVgcUi, "Computed preferred height ({}) is negative", h);
                preferredSize_[1] = 0;
            }
            isPreferredSizeComputed_ = true;
        }
    }

    // Assumes a geometry update is necessary.
    void updateGeometry_();

    void updateRootGeometry_() const {
        Widget* root_ = root();
        root_->updateGeometry();
    }

    void prePaintUpdateGeometry_();

    // Background
    graphics::GeometryViewPtr triangles_;
    core::Color backgroundColor_;
    style::BorderRadii borderRadii_;
    bool backgroundChanged_ = true;
    // TODO: border width/style

    // Events
    HandledEventPolicy handledEventPolicy_ = HandledEventPolicy::Skip;

    // Handling of actions.
    // All of those are stored at the root.
    //
    MouseButton mouseActionButton_ = MouseButton::None;
    core::Array<MouseActionEventPtr> pendingMouseActionEvents_;
    WidgetPtr pendingMouseClickWidget_;
    WidgetPtr pendingMouseDragWidget_;
    WidgetPtr currentMouseDragWidget_;
    ActionPtr pendingMouseClickAction_;
    ActionPtr pendingMouseDragAction_;
    ActionPtr currentMouseDragAction_;

    // Mouse
    // TODO: Move the whole hover chain and mouse captor to WidgetTree?
    Widget* mouseCaptor_ = nullptr;
    Widget* hoverChainParent_ = nullptr;
    Widget* hoverChainChild_ = nullptr;
    geometry::Vec2f lastMouseHoverPosition_;
    ModifierKeys lastMouseHoverModifierKeys_;
    bool forceNextMouseHover_ = true;
    bool isHovered_ = false;
    bool isInHoveredRow_ = false;
    bool isInHoveredColumn_ = false;
    bool isHoverLocked_ = false;
    bool isChildHoverEnabled_ = true;
    MouseButtons pressedButtons_ = {};

    void updateFocusOnClick_(Widget* clickedWidget);

    void appendToPendingMouseActionEvents_(MouseEvent* event);
    void mapMouseActionPosition_(MouseEvent* event, Widget* widget);
    void maybeStartPendingMouseAction_();
    void cancelActionsFromDeadWidgets_();
    bool handleMousePressActions_(MouseEvent* event);
    bool handleMouseMoveActions_(MouseEvent* event);
    bool handleMouseReleaseActions_(MouseEvent* event);

    bool checkAlreadyHovered_();
    void mouseMove_(MouseMoveEvent* event);
    void mousePress_(MousePressEvent* event);
    void mouseRelease_(MouseReleaseEvent* event);
    void mouseScroll_(ScrollEvent* event);

    void mouseHover_();
    void mouseEnter_();
    void mouseLeave_();

    void onUnhover_();

    void lockHover_();
    void unlockHover_();
    void onHoverUnlocked_();

    // Status
    Visibility visibility_ = Visibility::Inherit;
    bool computedVisibility_ = true;

    void updateComputedVisibility_();
    void setComputedVisibility_(bool isVisible);

    // Keyboard
    //
    // focus_ can have the following values:
    // - nullptr: this means that there is no focused widget in this branch
    // - this: the focused widget is this widget
    // - child ptr: the focused widget is a descendant of this widget
    //
    // Note that the focus_ data member is only about the currently focused
    // widget. In addition, the widget tree stores a stack of previously
    // focused widgets. This enables the ability to have widget with "temporary
    // focus policy" (non-sticky), which are given the focus and pushed to the
    // stack, then once they lose focus, they are popped from the stack and the
    // focus is given back to the widget that previously had the focus.
    //
    // Possible space optimizations:
    // - store both policy and strength as a unique UInt8
    // - Make them virtual method
    // - Make isTextInputReceiver() a virtual method
    // - Don't cache the focus_ chain (redundant with focusStack_.last().ancestors())
    //
    // Per-widget attributes.
    Widget* focus_ = nullptr; // Cached chain from root to current focused widget.
    FocusPolicyFlags focusPolicy_ = FocusPolicy::Never;
    FocusStrength focusStrength_ = FocusStrength::Medium;
    bool isTextInputReceiver_ = false;
    // Per-tree attributes
    bool isTreeActive_ = false;
    bool isFocusSetOrCleared_ = false;
    core::Array<WidgetPtr> focusStack_;
    Widget* keyboardCaptor_ = nullptr;

    void emitFocusInOutEvents_(
        const core::Array<WidgetPtr>& oldStack,
        const core::Array<WidgetPtr>& newStack,
        FocusReason reason);

    void keyEvent_(
        PropagatedKeyEvent* event,
        bool isPress,
        const core::Array<WidgetPtr>& chain,
        Int index);

    // Engine
    graphics::Engine* lastPaintEngine_ = nullptr;
    void releaseEngine_();
    void setEngine_(graphics::Engine* engine);
    void prePaintUpdateEngine_(graphics::Engine* engine);

    VGC_SLOT(onEngineAboutToBeDestroyed_, releaseEngine_)
    VGC_SLOT(onWidgetAddedSlot_, onWidgetAdded_)
    VGC_SLOT(onWidgetRemovedSlot_, onWidgetRemoved_)
    VGC_SLOT(onActionAddedSlot_, onActionAdded_)
    VGC_SLOT(onActionRemovedSlot_, onActionRemoved_)
};

} // namespace vgc::ui

#endif // VGC_UI_WIDGET_H
