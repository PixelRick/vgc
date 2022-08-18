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

#ifndef VGC_UI_GRID_H
#define VGC_UI_GRID_H

#include <vgc/core/array.h>
#include <vgc/ui/margins.h>
#include <vgc/ui/widget.h>

namespace vgc::ui {

namespace detail {

struct GridCell {
    Widget* widget = nullptr;
    // Cached metrics
    Margins margins;
    geometry::Vec2f stretch;
    geometry::Vec2f shrink;
    geometry::Vec2f preferredSize;
};

struct GridTrack {
    float offset;
    // Cached metrics
    float maxChildPreferredSize = 0.f;
    float minChildPreferredSize = 0.f;
    float maxStretchWidth = 0.f;
    float stretchHeight = 0.f;
    float shrinkWidth = 0.f;
    float shrinkHeight = 0.f;
    PreferredSize preferredSize;
};

} // namespace detail

VGC_DECLARE_OBJECT(Grid);

/// \class vgc::ui::Grid
/// \brief Arrange a sequence of widgets in rows and/or columns.
///
class VGC_UI_API Grid : public Widget {
private:
    VGC_OBJECT(Grid, Widget)

    using GridCell = detail::GridCell;

protected:
    /// This is an implementation details. Please use
    /// Grid::create() instead.
    ///
    Grid();

public:
    /// Creates a Row.
    ///
    static GridPtr create();

    /// Returns the current number of columns.
    ///
    Int numRows() const {
        return numRows_;
    }

    /// Returns the current number of rows.
    ///
    Int numColumns() const {
        return numColumns_;
    }

    /// Adds a widget to this grid and in the cell at the `i`-th row and `j`-th column.
    ///
    void addWidget(Widget* widget, Int i, Int j);

    /// Returns the widget in the cell at the `i`-th row and `j`-th column.
    ///
    Widget* getWidgetAt(Int i, Int j) const;

    /// Clears the cell at the `i`-th row and `j`-th column from any widget.
    ///
    WidgetPtr clearCell(Int i, Int j);

protected:
    void onWidgetAdded(Widget* child) override;
    void onWidgetRemoved(Widget* child) override;
    geometry::Vec2f computePreferredSize() const override;
    void updateChildrenGeometry() override;

private:
    core::Array<GridCell> cells_;

    core::Array<float> offsets_;
    float preferredRowHeight_;

    Int numRows_ = 0;
    Int numColumns_ = 0;

    float getPreferredRowHeight_(Int i) const;
    float getPreferredColumnWidth_(Int j) const;
    void erase_(Widget* widget);
    void resize_(Int numRows, Int numColumns);

    GridCell& cellAt_(Int i, Int j) {
        return cells_[i * numRows_ + j];
    }

    void resizeUpTo_(Int i, Int j) {
        resize_((std::max)(numRows_, i + 1), std::max(numColumns_, j + 1));
    }

    float getRowStartY_(Int index) {
        return offsets_[index];
    }

    float getRowEndY_(Int index) {
        return offsets_[index + 1];
    }

    float getColumnStartX_(Int index) {
        return offsets_[numRows_ + 1 + index];
    }

    float getColumnEndX_(Int index) {
        return offsets_[numRows_ + 1 + index + 1];
    }

    float getRowHeight_(Int index) {
        return getRowEndY_(index) - getRowStartY_(index);
    }

    float getColumnWidth_(Int index) {
        return getColumnEndX_(index) - getColumnStartX_(index);
    }
};

} // namespace vgc::ui

#endif // VGC_UI_GRID_H
