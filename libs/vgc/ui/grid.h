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
#include <vgc/geometry/Range1f.h>
#include <vgc/ui/margins.h>
#include <vgc/ui/widget.h>

namespace vgc::ui {

namespace detail {

namespace DirIndex {
enum Value {
    Width = 0,  // in x
    Height = 1, // in y
};
} // namespace DirIndex

struct GridCell {
    Widget* widget = nullptr;

    struct Metrics {
        Margins margins;
        geometry::Vec2f stretch;
        geometry::Vec2f shrink;
        geometry::Vec2f preferredSize;
    };
    mutable Metrics metrics;

    constexpr bool isShrinkable(DirIndex::Value v) const {
        return metrics.shrink[v] > 0;
    }

    constexpr bool isStretchable(DirIndex::Value v) const {
        return metrics.stretch[v] > 0;
    }

    constexpr void zeroMetrics() const {
        metrics.margins = {};
        metrics.stretch = {};
        metrics.shrink = {};
        metrics.preferredSize = {};
    }
};

struct GridTrackInfo {
    float offset = 0.f;
    float size = 0.f;

    struct Metrics {
        // size defined by grid-template-rows, grid-template-columns
        //              or grid-auto-rows, grid-auto-columns.
        PreferredSize templateSize;
        geometry::Range1f cellPreferredSizeRange;
        // this average is only among the stretchables
        float avgCellStretch = 0.f;
        // this average is only among the shrinkables
        float avgCellShrink = 0.f;
        float maxUnshrinkableCellSize = 0.f;
        // empty cells do not count
        Int numShrinkableCells = 0;
        // empty cells do not count
        Int numStretchableCells = 0;
        bool hasNonShrinkableCells = false;
        bool hasNonStretchableCells = false;
    };
    mutable Metrics metrics;

    constexpr float endOffset() const {
        return offset + size;
    }

    constexpr bool hasShrinkables() const {
        return metrics.numShrinkableCells > 0;
    }

    constexpr bool hasStretchables() const {
        return metrics.numStretchableCells > 0;
    }

    constexpr void zeroMetrics() const {
        metrics.cellPreferredSizeRange = {};
        metrics.avgCellStretch = 0.f;
        metrics.avgCellShrink = 0.f;
        metrics.maxUnshrinkableCellSize = 0.f;
        metrics.numShrinkableCells = 0;
        metrics.numStretchableCells = 0;
        metrics.hasNonShrinkableCells = false;
        metrics.hasNonStretchableCells = false;
    }
};

} // namespace detail

VGC_DECLARE_OBJECT(Grid);

/// \class vgc::ui::Grid
/// \brief Arrange a sequence of widgets in rows and/or columns.
///
class VGC_UI_API Grid : public Widget {
private:
    VGC_OBJECT(Grid, Widget)

    using Cell = detail::GridCell;
    using TrackInfo = detail::GridTrackInfo;

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
    core::Array<Cell> cells_;
    core::Array<TrackInfo> tracks_;
    Int numRows_ = 0;
    Int numColumns_ = 0;
    mutable PreferredSize autoRowsSize_;
    mutable PreferredSize autoColumnsSize_;

    Cell& cellAt_(Int i, Int j) {
        return cells_[i * numRows_ + j];
    }

    const Cell& cellAt_(Int i, Int j) const {
        return cells_[i * numRows_ + j];
    }

    const TrackInfo& rowInfo_(Int i) const {
        return tracks_[i];
    }

    const TrackInfo& columnInfo_(Int j) const {
        return tracks_[numRows_ + j];
    }

    float getPreferredRowHeight_(Int i) const;
    float getPreferredColumnWidth_(Int j) const;

    void erase_(Widget* widget);
    void resize_(Int numRows, Int numColumns);

    void resizeUpTo_(Int i, Int j) {
        resize_((std::max)(numRows_, i + 1), (std::max)(numColumns_, j + 1));
    }
};

} // namespace vgc::ui

#endif // VGC_UI_GRID_H
