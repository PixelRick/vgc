// Copyright 2023 The VGC Developers
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

#ifndef VGC_WORKSPACE_FREEHANDEDGEGEOMETRY_H
#define VGC_WORKSPACE_FREEHANDEDGEGEOMETRY_H

#include <vgc/core/id.h>
#include <vgc/geometry/catmullrom.h>
#include <vgc/geometry/yuksel.h>
#include <vgc/vacomplex/keyedge.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/edgegeometry.h>

namespace vgc::workspace {

class CellProperty {
protected:
    explicit CellProperty(core::StringId name) : name_(name) {}

private:
    virtual bool updateFromDomCell_(dom::Element* element) = 0;
    virtual void writeToDomCell_(dom::Element* element) const = 0;
    virtual void removeFromDomCell_(dom::Element* element) const = 0;

    const core::StringId name_;
};



class EdgeStyle final : public CellProperty {
public:
    EdgeStyle() noexcept : CellProperty(core::StringId("style")) {};

private:
    core::Color color_ = {};

    bool updateFromDomCell_(dom::Element* element) override;
    void writeToDomCell_(dom::Element* element) const override;
    void removeFromDomCell_(dom::Element* element) const override;
};

class CellProperties {
private:
    bool updateFromDomCell_(dom::Element* element);
    void writeToDomCell_(dom::Element* element) const;
    void removeFromDomCell_(dom::Element* element) const;

    std::unique_ptr<EdgeStyle> styleProperty_;
    //core::Array<std::unique_ptr<CellProperty>> customProperties_;
};

class KeyEdgeProperties : public CellProperties {
private:

    // proposals for concatenate/blend operation interface:

    // do copy +
    void concatenate(bool after, const KeyEdgeProperties& other, bool reversed);

    // or assign from
    void assignConcat(const KeyEdgeProperties& a, bool reversedA, const KeyEdgeProperties& b, bool reversedB);

    // OTHER IDEA: convert to type before...
};

class KeyFaceProperties : public CellProperties {
private:

    // proposals for concatenate/blend operation interface:

    // do copy +
    void concatenate(const KeyEdgeProperties& other);

    // or assign from
    void assignConcat(const KeyEdgeProperties& a, const KeyEdgeProperties& b);
};


class FreehandEdgePoint {
public:
    VGC_WARNING_PUSH
    VGC_WARNING_MSVC_DISABLE(26495) // member variable uninitialized
    FreehandEdgePoint(core::NoInit)
        : pos_(core::noInit) {
    }
    VGC_WARNING_POP

    FreehandEdgePoint(const geometry::Vec2d& position, double width)
        : pos_(position)
        , width_(width) {
    }

    FreehandEdgePoint(const geometry::StrokeSample2d& sample)
        : pos_(sample.position())
        , width_(sample.halfwidth(0) * 2) {
    }

    FreehandEdgePoint lerp(const FreehandEdgePoint& b, double u) {
        FreehandEdgePoint result = *this;
        result.pos_ += u * (b.pos_ - pos_);
        result.width_ += u * (b.width_ - width_);
        return result;
    }

    FreehandEdgePoint average(const FreehandEdgePoint& b) {
        return FreehandEdgePoint(0.5 * (pos_ + b.pos_), 0.5 * (width_ + b.width_));
    }

    geometry::Vec2d position() const {
        return pos_;
    }

    double width() const {
        return width_;
    }

private:
    geometry::Vec2d pos_;
    double width_ = 0;
};

class VGC_WORKSPACE_API FreehandEdgeGeometry : public EdgeGeometry {
public:
    using SharedConstPositions = geometry::SharedConstVec2dArray;
    using SharedConstWidths = core::SharedConstDoubleArray;

    using StrokeType = geometry::CatmullRomSplineStroke2d;
    //using StrokeType = geometry::YukselSplineStroke2d;

    static std::shared_ptr<FreehandEdgeGeometry> createFromPoints(
        core::Span<FreehandEdgePoint> points,
        bool isClosed,
        double tolerance);

    FreehandEdgeGeometry(bool isClosed)
        : EdgeGeometry(isClosed) {

        stroke_ = createStroke_();
    }

    FreehandEdgeGeometry(bool isClosed, double constantWidth)
        : EdgeGeometry(isClosed) {

        stroke_ = createStroke_();
        stroke_->setConstantWidth(constantWidth);
    }

    FreehandEdgeGeometry(
        const SharedConstPositions& positions,
        const SharedConstWidths& widths,
        bool isClosed,
        bool isWidthConstant)

        : EdgeGeometry(isClosed)
        , sharedConstPositions_(positions)
        , sharedConstWidths_(widths) {

        stroke_ = createStroke_();
        if (isWidthConstant) {
            stroke_->setConstantWidth(widths.get()[0]);
        }
        else {
            stroke_->setWidths(widths);
        }
        stroke_->setPositions(positions);
    }

    const geometry::Vec2dArray& positions() const {
        return isBeingEdited_ ? editPositions_ : stroke_->positions();
    }

    const core::DoubleArray& widths() const {
        return isBeingEdited_ ? editWidths_ : stroke_->widths();
    }

    void setPositions(const SharedConstPositions& positions);

    void setPositions(geometry::Vec2dArray positions);

    void setWidths(const SharedConstWidths& widths);

    void setWidths(core::DoubleArray widths);

    std::shared_ptr<KeyEdgeGeometry> clone() const override;

    /// Expects positions in object space.
    ///
    vacomplex::EdgeSampling computeSampling(
        const geometry::CurveSamplingParameters& params,
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition,
        vacomplex::EdgeSnapTransformationMode mode) const override;

    vacomplex::EdgeSampling
    computeSampling(const geometry::CurveSamplingParameters& params) const override;

    void startEdit() override;
    void resetEdit() override;
    void finishEdit() override;
    void abortEdit() override;

    /// Expects delta in object space.
    ///
    void translate(const geometry::Vec2d& delta) override;

    /// Expects transformation in object space.
    ///
    void transform(const geometry::Mat3d& transformation) override;

    /// Expects positions in object space.
    ///
    void snap(
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition,
        vacomplex::EdgeSnapTransformationMode mode) override;

    geometry::Vec2d sculptGrab(
        const geometry::Vec2d& startPosition,
        const geometry::Vec2d& endPosition,
        double radius,
        double strength,
        double tolerance,
        bool isClosed) override;

    geometry::Vec2d sculptWidth(
        const geometry::Vec2d& position,
        double delta,
        double radius,
        double tolerance,
        bool isClosed = false) override;

    geometry::Vec2d sculptSmooth(
        const geometry::Vec2d& position,
        double radius,
        double strength,
        double tolerance,
        bool isClosed) override;

    bool updateFromDomEdge_(dom::Element* element) override;
    void writeToDomEdge_(dom::Element* element) const override;
    void removeFromDomEdge_(dom::Element* element) const override;

private:
    

    // properties, only color for now

    static void computeSnappedLinearS_(
        geometry::Vec2dArray& outPoints,
        StrokeType* srcStroke,
        core::DoubleArray& srcArclengths,
        const geometry::Vec2d& snapStartPosition,
        const geometry::Vec2d& snapEndPosition);

    static void
    computeKnotArclengths_(core::DoubleArray& outArclengths, StrokeType* srcStroke);

    std::unique_ptr<StrokeType> createStroke_() const;

    std::shared_ptr<KeyEdgeGeometry>
    merge_(bool direction, KeyEdgeGeometry* other, bool otherDirection) const override;
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_FREEHANDEDGEGEOMETRY_H
