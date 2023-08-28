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

#include <vgc/vacomplex/keyedgedata.h>

#include <vgc/vacomplex/detail/operationsimpl.h>
#include <vgc/vacomplex/keyedge.h>

namespace vgc::vacomplex {

KeyEdgeDataPtr KeyEdgeData::clone() const {
    KeyEdgeDataPtr result = std::make_shared<KeyEdgeData>(isClosed_);
    result->assignClonedProperties(this);
    if (stroke_) {
        result->stroke_ = stroke_->clone();
    }
}

void KeyEdgeData::translate(const geometry::Vec2d& delta) {
    if (stroke_) {
        stroke_->translate(delta);
    }
    CellData::translate(delta);
}

void KeyEdgeData::transform(const geometry::Mat3d& transformation) {
    if (stroke_) {
        stroke_->transform(transformation);
    }
    CellData::transform(transformation);
}

void KeyEdgeData::snap(
    const geometry::Vec2d& snapStartPosition,
    const geometry::Vec2d& snapEndPosition,
    geometry::CurveSnapTransformationMode mode =
        geometry::CurveSnapTransformationMode::LinearInArclength) {

    if (stroke_) {
        std::array<geometry::Vec2d, 2> oldEndPositions = stroke_->endPositions();
        if (stroke_->snap(snapStartPosition, snapEndPosition)) {
            for (const auto& p : properties()) {
                p.second->onKeyEdgeStrokeChanged_(stroke());
            }
        }
    }
}

const geometry::AbstractStroke2d* KeyEdgeData::stroke() const {
    return stroke_.get();
}

void KeyEdgeData::setStroke(const geometry::AbstractStroke2d* newStroke) {
    if (!newStroke) {
        stroke_.reset();
    }
    else if (!stroke_->copyAssign(newStroke)) {
        stroke_ = newStroke->clone();
    }
    for (const auto& p : properties()) {
        p.second->onKeyEdgeStrokeChanged_(stroke());
    }
}

void KeyEdgeData::setStroke(std::unique_ptr<geometry::AbstractStroke2d>&& newStroke) {
    stroke_ = std::move(newStroke);
    for (const auto& p : properties()) {
        p.second->onKeyEdgeStrokeChanged_(stroke());
    }
}

/* static */
KeyEdgeDataPtr KeyEdgeData::glue_(
    core::Array<KeyHalfedgeData> khds,
    const geometry::AbstractStroke2d* gluedStroke) {

    struct PropertyTemplate {
        core::StringId id;
        const CellProperty* prop;
    };

    core::Array<PropertyTemplate> templates;
    for (const KeyHalfedgeData& khd : khds) {
        KeyEdgeData* ked = khd.edgeData();
        for (const auto& p : ked->properties()) {
            core::StringId id = p.first;
            if (!templates.search(
                    [id](const PropertyTemplate& p) { return p.id == id; })) {
                templates.append(PropertyTemplate{id, p.second.get()});
            }
        }
    }

    auto result = std::make_shared<KeyEdgeData>(khds[0].edgeData()->isClosed());
    result->setStroke(gluedStroke);
    for (const PropertyTemplate& p : templates) {
        std::unique_ptr<CellProperty> newProp = p.prop->onKeyEdgeGlue_(khds, gluedStroke);
        if (newProp) {
            result->setProperty(std::move(newProp));
        }
    }
    return result;
}

//
// IDEA: do conversion to common best stroke geometry to merge
//       then match cell properties by pairs (use null if not present)

KeyEdgeDataPtr KeyEdgeData::glue(core::Array<KeyHalfedgeData> khds) {

    struct ConvertedStroke {
        std::unique_ptr<geometry::AbstractStroke2d> converted;
        const geometry::AbstractStroke2d* st;
    };
    core::Array<ConvertedStroke> converteds;

    // Find best model first.
    const geometry::AbstractStroke2d* bestModelStroke = khds[0].edgeData()->stroke();
    core::Array<bool> directions;
    directions.reserve(khds.length());
    for (const KeyHalfedgeData& khd : khds) {
        const geometry::AbstractStroke2d* st = khd.edgeData()->stroke();
        const geometry::StrokeModelInfo& model2 = st->modelInfo();
        if (model2.name() != bestModelStroke->modelInfo().name()) {
            if (model2.defaultConversionRank()
                > bestModelStroke->modelInfo().defaultConversionRank()) {
                bestModelStroke = khd.edgeData()->stroke();
            }
        }
        converteds.emplaceLast().st = st;
        directions.append(khd.direction());
    }

    core::Array<const geometry::AbstractStroke2d*> strokes;
    for (ConvertedStroke& converted : converteds) {
        if (converted.st->modelInfo().name() != bestModelStroke->modelInfo().name()) {
            converted.converted = bestModelStroke->convert(converted.st);
            converted.st = converted.converted.get();
        }
        strokes.append(converted.st);
    }

    std::unique_ptr<geometry::AbstractStroke2d> gluedStroke =
        bestModelStroke->cloneEmpty();
    gluedStroke->assignAverage(strokes, directions);
}

/* static */
KeyEdgeDataPtr KeyEdgeData::concat(
    const KeyHalfedgeData& khd1,
    const KeyHalfedgeData& khd2,
    bool smoothJoin) {
    KeyEdgeData* ked1 = khd1.edgeData();
    KeyEdgeData* ked2 = khd2.edgeData();
    VGC_ASSERT(ked1 != nullptr && ked2 != nullptr);

    struct PropertyTemplate {
        core::StringId id;
        const CellProperty* prop;
    };

    core::Array<PropertyTemplate> templates;
    for (const auto& p : ked1->properties()) {
        core::StringId id = p.first;
        if (!templates.search([id](const PropertyTemplate& p) { return p.id == id; })) {
            templates.append(PropertyTemplate{id, p.second.get()});
        }
    }
    for (const auto& p : ked2->properties()) {
        core::StringId id = p.first;
        if (!templates.search([id](const PropertyTemplate& p) { return p.id == id; })) {
            templates.append(PropertyTemplate{id, p.second.get()});
        }
    }

    const geometry::AbstractStroke2d* st1 = ked1->stroke();
    const geometry::AbstractStroke2d* st2 = ked2->stroke();
    const geometry::StrokeModelInfo& model1 = st1->modelInfo();
    const geometry::StrokeModelInfo& model2 = st2->modelInfo();

    std::unique_ptr<geometry::AbstractStroke2d> converted;
    const geometry::AbstractStroke2d* model = st1;
    if (model1.name() != model2.name()) {
        if (model1.defaultConversionRank() >= model2.defaultConversionRank()) {
            converted = st1->convert(st2);
            st2 = converted.get();
        }
        else {
            converted = st2->convert(st1);
            st1 = converted.get();
        }
    }
    std::unique_ptr<geometry::AbstractStroke2d> concatStroke = st1->cloneEmpty();
    concatStroke->assignConcat(
        st1, !khd1.direction(), st2, !khd1.direction(), smoothJoin);

    auto result = std::make_shared<KeyEdgeData>(ked1->isClosed());
    result->setStroke(std::move(concatStroke));
    for (const PropertyTemplate& p : templates) {
        std::unique_ptr<CellProperty> newProp = p.prop->onKeyEdgeConcat_(khd1, khd2);
        if (newProp) {
            result->setProperty(std::move(newProp));
        }
    }
    return result;
}

//std::shared_ptr<KeyEdgeData>
//KeyEdgeData::merge(bool direction, KeyEdgeData* other, bool otherDirection) const {
//
//    // TODO: try both ways, if none works then convert both to a default geometry model for the merge
//    //       then rebuild with the best of the original models that supports being built from the default.
//    return merge_(direction, other, otherDirection);
//}
//
//void KeyEdgeData::dirtyEdgeSampling() const {
//    if (edge_) {
//        Complex* complex = edge_->complex();
//        detail::Operations ops(complex);
//        ops.onGeometryChanged_(edge_);
//    }
//}
//
//void KeyEdgeData::dirtyEdgeStyle() const {
//    if (edge_) {
//        Complex* complex = edge_->complex();
//        detail::Operations ops(complex);
//        ops.onStyleChanged_(edge_);
//    }
//}

} // namespace vgc::vacomplex
