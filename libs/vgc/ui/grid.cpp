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

#include <vgc/ui/Grid.h>

#include <vgc/ui/strings.h>

#include <vgc/ui/detail/paintutil.h>

namespace vgc::ui {

Grid::Grid()
    : Widget() {

    addStyleClass(strings::Grid);
}

GridPtr Grid::create() {
    return GridPtr(new Grid());
}

void Grid::addWidget(Widget* widget, Int i, Int j) {
    // clear from current position if already in a cell
    if (widget) {
        erase_(widget);
        resizeUpTo_(i, j);
        cellAt_(i, j).widget = widget;
        addChild(widget);
        updateGeometry();
    }
}

Widget* Grid::getWidgetAt(Int i, Int j) const {
    // XXX add bound checks
    return cells_[i * numRows_ + j].widget;
}

WidgetPtr Grid::clearCell(Int i, Int j) {
    WidgetPtr w = getWidgetAt(i, j);
    if (w) {
        // this indirectly calls erase_(w) and updateGeometry()
        // see onWidgetRemoved()
        //
        w->reparent(nullptr);
    }
    return w;
}

void Grid::onWidgetAdded(Widget*) {
}

void Grid::onWidgetRemoved(Widget* widget) {
    erase_(widget);
    updateGeometry();
}

namespace {

float getLeftRightMargins(const Widget* widget) {
    return detail::getLength(widget, strings::margin_left)
           + detail::getLength(widget, strings::margin_right);
}

float getTopBottomMargins(const Widget* widget) {
    return detail::getLength(widget, strings::margin_top)
           + detail::getLength(widget, strings::margin_bottom);
}

float getLeftRightPadding(const Widget* widget) {
    return detail::getLength(widget, strings::padding_left)
           + detail::getLength(widget, strings::padding_right);
}

float getTopBottomPadding(const Widget* widget) {
    return detail::getLength(widget, strings::padding_top)
           + detail::getLength(widget, strings::padding_bottom);
}

} // namespace

geometry::Vec2f Grid::computePreferredSize() const {
    PreferredSizeType auto_ = PreferredSizeType::Auto;
    PreferredSize w = preferredWidth();
    PreferredSize h = preferredHeight();

    using namespace strings;

    autoRowsSize_ = style(grid_auto_rows).to<PreferredSize>();;
    autoColumnsSize_ = style(grid_auto_columns).to<PreferredSize>();;

    // compute and cache all metrics
    // -----

    // zero track metrics
    for (const TrackInfo& track : tracks_) {
        track.zeroMetrics();
    }

    for (Int i = 0; i < numRows_; ++i) {
        const TrackInfo& rowInfo = rowInfo_(i);
        TrackInfo::Metrics& rowMetrics = rowInfo.metrics;
        // XXX support "grid-template-rows" when lists are supported by style parser
        // fallback when undefined: grid-auto-rows
        rowMetrics.templateSize = autoRowsSize_;

        for (Int j = 0; j < numColumns_; ++i) {
            const TrackInfo& columnInfo = columnInfo_(i);
            TrackInfo::Metrics& columnMetrics = columnInfo.metrics;
            // XXX support "grid-template-columns" when lists are supported by style parser
            // fallback when undefined: grid-auto-columns
            columnMetrics.templateSize = autoColumnsSize_;

            const Cell& cell = cellAt_(i, j);
            Cell::Metrics& cellMetrics = cell.metrics;

            //cell.widget
        }
    }

    geometry::Vec2f res(0, 0);
    if (w.type() == auto_) {
        float paddingLeft = detail::getLength(this, strings::padding_left);
        float paddingRight = detail::getLength(this, strings::padding_right);
        res[0] = paddingLeft + paddingRight;
        for (Int i = 0; i < numColumns_; ++i) {
            res[0] += getPreferredColumnWidth_(i);
            // += gap
        }
    }
    else {
        res[0] = w.value();
    }
    if (h.type() == auto_) {
        float paddingTop = detail::getLength(this, strings::padding_top);
        float paddingBottom = detail::getLength(this, strings::padding_bottom);
        res[1] = paddingTop + paddingBottom;
        for (Int i = 0; i < numRows_; ++i) {
            res[0] += getPreferredRowHeight_(i);
            // += gap
        }
    }
    else {
        res[1] = h.value();
    }
    return res;
}

void Grid::updateChildrenGeometry() {

    bool hinting = (style(strings::pixel_hinting) == strings::normal);

    core::Array<float> preferredColumnWidths(numColumns_, core::NoInit{});
    float totalPreferredW = 0;
    for (Int i = 0; i < numColumns_; ++i) {
        float w = getPreferredColumnWidth_(i);
        totalPreferredW += w;
        preferredColumnWidths.getUnchecked(i) = w;
    }
    float scaleW = width() / totalPreferredW;

    core::Array<float> preferredRowHeights(numRows_, core::NoInit{});
    float totalPreferredH = 0;
    for (Int j = 0; j < numRows_; ++j) {
        Widget* w = getWidgetAt(j, 0);
        totalPreferredW += w ? w->preferredSize().x() : 40;
    }
    float scaleH = height() / totalPreferredH;

    offsets_.resize(numRows_ + 1 + numColumns_ + 1);
    float* const rowYs = offsets_.data();
    float* const rowEndYs = rowYs + 1;
    float* const colXs = rowYs + numRows_ + 1;
    float* const colEndXs = colXs + 1;

    // compute row sizes
    if (hinting) {
        float available = height();
        // temporarily use offsets_ to store heights
        for (Int i = 0; i < numRows_; ++i) {
            float alloc = std::floor(scaleH * rowHeight);
            rowEndYs[i] = available;
            available -= alloc;
        }
        float k = static_cast<Int>(available);
        float y = 0.f;
        for (Int i = 0; i < numRows_; ++i) {
            y += rowEndYs[i];
            if (i < k) {
                // redistribute freeH
                y += 1.f;
            }
            rowEndYs[i + 1] = y;
        }
    }
    else {
        float y = 0.f;
        for (Int i = 0; i < numRows_; ++i) {
            y += scaleH * rowHeight;
            rowEndYs[i] = y;
        }
    }
    // compute column sizes
    if (hinting) {
        float available = width();
        // temporarily use offsets_ to store widths
        for (Int i = 0; i < numColumns_; ++i) {
            float alloc = std::floor(scaleW * getPreferredColumnWidth_(i));
            colEndXs[i] = alloc;
            available -= alloc;
        }
        float k = static_cast<Int>(available);
        float x = 0.f;
        for (Int i = 0; i < numColumns_; ++i) {
            x += colEndXs[i];
            if (i < k) {
                // redistribute freeH
                x += 1.f;
            }
            colEndXs[i] = x;
        }
    }
    else {
        float x = 0.f;
        for (Int i = 0; i < numColumns_; ++i) {
            x += scaleW * getPreferredColumnWidth_(i);
            colEndXs[i] = x;
        }
    }

    bool isRow =
        (direction_ == GridDirection::Row) || (direction_ == GridDirection::RowReverse);
    bool isReverse = (direction_ == GridDirection::RowReverse)
                     || (direction_ == GridDirection::ColumnReverse);
    bool hinting = (style(strings::pixel_hinting) == strings::normal);
    float paddingLeft = detail::getLength(this, strings::padding_left);
    float paddingRight = detail::getLength(this, strings::padding_right);
    float paddingTop = detail::getLength(this, strings::padding_top);
    float paddingBottom = detail::getLength(this, strings::padding_bottom);
    float preferredMainSize = isRow ? preferredSize().x() : preferredSize().y();
    float mainPaddingBefore = isRow ? paddingLeft : paddingTop;
    float crossPaddingBefore = isRow ? paddingTop : paddingLeft;
    float crossPaddingAfter = isRow ? paddingBottom : paddingRight;
    float mainSize = isRow ? width() : height();
    float crossSize = isRow ? height() : width();
    float freeSpace = mainSize - preferredMainSize;
    float eps = 1e-6f;
    // TODO: have a loop to resolve constraint violations, as per 9.7.4:
    // https://www.w3.org/TR/css-Gridbox-1/#resolve-Gridible-length
    // Indeed, although we currently don't have explicit min/max constraints,
    // we still have an implicit min-size = 0 constraint. The algorithm
    // below don't properly handle this constraint: if the size of one of
    // the items is shrinked to a negative size, then it is clamped to zero,
    // but the lost space due to clamping isn't redistributed to other items,
    // causing an overflow.
    float childStretchBonus = 0;
    float totalStretch = computeTotalStretch(isRow, freeSpace, this, childStretchBonus);
    if (totalStretch < eps) {
        // For now, we stretch evenly as if all childStretch were equal to
        // one. Later, we should instead insert empty space between the items,
        // based on alignment properties, see:
        // https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Box_Alignment
        childStretchBonus = 1;
        totalStretch = computeTotalStretch(isRow, freeSpace, this, childStretchBonus);
    }
    float extraSpacePerStretch = freeSpace / totalStretch;
    float childMainPosition = mainPaddingBefore;
    Widget* child = isReverse ? lastChild() : firstChild();
    while (child) {
        stretchChild(
            isRow,
            freeSpace,
            crossSize,
            extraSpacePerStretch,
            child,
            childStretchBonus,
            childMainPosition,
            crossPaddingBefore,
            crossPaddingAfter,
            hinting);
        child = isReverse ? child->previousSibling() : child->nextSibling();
    }
}

float Grid::getPreferredRowHeight_(Int i) const {
    Widget* w = getWidgetAt(i, 0);
    return w ? w->preferredSize().x() + getLeftRightMargins(w) : 26.f;
}

float Grid::getPreferredColumnWidth_(Int j) const {
    Widget* w = getWidgetAt(j, 0);
    return w ? w->preferredSize().x() + getLeftRightMargins(w) : 50.f;
}

void Grid::erase_(Widget* widget) {
    for (Cell& cell : cells_) {
        if (cell.widget == widget) {
            cell = Cell();
            // XXX shrink array ?
        }
    }
}

void Grid::resize_(Int numRows, Int numColumns) {
}

} // namespace vgc::ui
