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

#include <vgc/workspace/face.h>

#include <tesselator.h> // libtess2

#include <vgc/core/span.h>
#include <vgc/workspace/edge.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

namespace {

struct DomCycleComponent {
    dom::Path path = {};
    bool direction = false;
    bool isSteinerVertex = false;

    template<typename OStream>
    friend void write(OStream& out, const DomCycleComponent& component) {
        fmt::memory_buffer b;
        path.writeTo_(b);
        if (!isSteinerVertex) {
            b.push_back(direction ? '-', '+');
        }
        write(out, std::string_view(b.begin(), static_cast<std::streamsize>(b.size())));
    }
};

template<typename IStream>
void readTo(DomCycleComponent& v, IStream& in) {
    char c = core::readExpectedCharacter(in, {'#', '@'});
    if (c == '#') {
        std::string s;
        // appends '#' first
        s.push_back(c);
        bool allowed = in.get(c) && isValidIdFirstChar(c);
        while (allowed) {
            s.push_back(c);
            allowed = in.get(c) && isValidIdChar(c);
        }
        v = Path(s);
        return;
    }
    // else '@'
    core::skipExpectedCharacter(in, '\'');
    std::string s = core::readStringUntilExpectedCharacter(in, '\'');
    v = Path(s);
}

} // namespace

const VacFaceCellFrameData* VacFaceCell::computeGeometryAt(core::AnimTime t) {
    VacFaceCellFrameData* data = frameData(t);
    if (data) {
        computeGeometry(*data);
    }
    return data;
}

VacKeyFace::~VacKeyFace() {
}

geometry::Rect2d VacKeyFace::boundingBox(core::AnimTime /*t*/) const {
    return bbox_;
}

bool VacKeyFace::isSelectableAt(
    const geometry::Vec2d& p,
    bool outlineOnly,
    double tol,
    double* outDistance,
    core::AnimTime t) const {

    using Vec2d = geometry::Vec2d;

    if (bbox_.isEmpty()) {
        return false;
    }

    geometry::Rect2d inflatedBbox = bbox_;
    inflatedBbox.setPMin(inflatedBbox.pMin() - Vec2d(tol, tol));
    inflatedBbox.setPMax(inflatedBbox.pMax() + Vec2d(tol, tol));
    if (!inflatedBbox.contains(p)) {
        return false;
    }

    // todo
    return false;
}

ElementStatus VacKeyFace::updateFromDom_(Workspace* workspace) {
    namespace ds = dom::strings;
    dom::Element* const domElement = this->domElement();
    if (!domElement) {
        // todo: use owning composite when it is implemented
        return ElementStatus::Ok;
    }

    std::string_view cyclesDescription = domElement->getAttribute(ds::cycles).getString();
    core::Array<core::Array<std::string>> a;

    // save old dependencies

    core::Array<Element*> oldDependencies = dependencies();

    // update group
    vacomplex::Group* parentGroup = nullptr;
    Element* parentElement = parent();
    if (parentElement) {
        workspace->updateElementFromDom(parentElement);
        vacomplex::Node* parentNode = parentElement->vacNode();
        if (parentNode) {
            // checked cast to group, could be something invalid
            parentGroup = parentNode->toGroup();
        }
    }
    if (!parentGroup) {
        onUpdateError_();
        return ElementStatus::ErrorInParent;
    }

    // at the moment, any change that does not affect style still affects geometry
    // so it does not cost much to also update style.
    onStyleChanged_();

    onUpdateError_();
    return ElementStatus::InvalidAttribute;
}

void VacKeyFace::onDependencyChanged_(Element* dependency) {
    VacElement* ve = dependency->toVacElement();
    if (ve) {
        vacomplex::Node* vnode = ve->vacNode();
        if (vnode) {
            vacomplex::Cell* cell = vnode->toCell();
            if (cell->toKeyEdge()) {
                onBoundaryGeometryChanged();
            }
        }
    }
}

void VacKeyFace::onDependencyRemoved_(Element* dependency) {
}

void VacKeyFace::preparePaint_(core::AnimTime t, PaintOptions /*flags*/) {
    // todo, use paint options to not compute everything or with lower quality
    computeGeometryAt(t);
}

void VacKeyFace::paint_(graphics::Engine* engine, core::AnimTime t, PaintOptions flags)
    const {

    vacomplex::KeyFace* ke = vacKeyFaceNode();
    if (!ke || t != ke->time()) {
        return;
    }

    // if not already done (should we leave preparePaint_ optional?)
    const_cast<VacKeyFace*>(this)->computeGeometry(frameData_);

    using namespace graphics;
    namespace ds = dom::strings;

    const dom::Element* const domElement = this->domElement();
    // XXX "implicit" cells' domElement would be the composite ?

    constexpr PaintOptions fillOptions = {PaintOption::Selected, PaintOption::Draft};

    // XXX todo: reuse geometry objects, create buffers separately (attributes waiting in FaceGraphics).
    FaceGraphics& graphics = frameData_.graphics_;

    if ((flags.hasAny(fillOptions) || !flags.has(PaintOption::Outline))
        && !graphics.fillGeometry_) {

        graphics.fillGeometry_ =
            engine->createDynamicTriangleListView(BuiltinGeometryLayout::XY_iRGBA);

        core::Color color = domElement->getAttribute(ds::color).getColor();

        engine->updateBufferData(
            graphics.fillGeometry_->vertexBuffer(0), //
            frameData_.triangulation_);
        engine->updateBufferData(
            graphics.fillGeometry_->vertexBuffer(1), //
            core::Array<float>({color.r(), color.g(), color.b(), color.a()}));
    }

    if (flags.has(PaintOption::Selected)) {
        // TODO: use special shader for selected faces
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(graphics.fillGeometry_);
    }
    else if (!flags.has(PaintOption::Outline)) {
        engine->setProgram(graphics::BuiltinProgram::Simple);
        engine->draw(graphics.fillGeometry_);
    }

    // draws nothing if non-selected and in outline mode
}

VacFaceCellFrameData* VacKeyFace::frameData(core::AnimTime t) const {
    vacomplex::FaceCell* cell = vacFaceCellNode();
    if (!cell) {
        return nullptr;
    }
    if (frameData_.time() == t) {
        return &frameData_;
    }
    return nullptr;
}

void VacKeyFace::computeGeometry(VacFaceCellFrameData& data) {

    using namespace topology;
    using geometry::Vec2d;
    using geometry::Vec2f;

    if (data.isGeometryComputed_ || data.isComputing_) {
        return;
    }
    KeyFace* kf = vacKeyFaceNode();
    if (!kf) {
        return;
    }

    data.isComputing_ = true;

    geometry::Curves2d curves2d;

    for (const vacomplex::KeyCycle& cycle : kf->cycles()) {
        KeyVertex* kv = cycle.steinerVertex();
        if (kv) {
            Vec2d p = kv->position();
            curves2d.moveTo(p[0], p[1]);
        }
        else {
            bool isFirst = true;
            for (const KeyHalfedge& khe : cycle.halfedges()) {
                const KeyEdge* ke = khe.edge();
                Element* ve = workspace()->find(ke->id());
                VacKeyEdge* vke = dynamic_cast<VacKeyEdge*>(ve);
                const VacEdgeCellFrameData* data = nullptr;
                if (vke) {
                    data = vke->computeStandaloneGeometry();
                }
                if (data) {
                    const geometry::CurveSampleArray& samples = data->samples();
                    if (!khe.direction()) {
                        for (const geometry::CurveSample& s : samples) {
                            Vec2d p = s.position();
                            if (isFirst) {
                                curves2d.moveTo(p[0], p[1]);
                                isFirst = false;
                            }
                            else {
                                curves2d.lineTo(p[0], p[1]);
                            }
                        }
                    }
                    else {
                        for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
                            const geometry::CurveSample& s = *it;
                            Vec2d p = s.position();
                            if (isFirst) {
                                curves2d.moveTo(p[0], p[1]);
                                isFirst = false;
                            }
                            else {
                                curves2d.lineTo(p[0], p[1]);
                            }
                        }
                    }
                }
                else {
                    // TODO: handle error
                }
            }
        }
        curves2d.close();
    }

    data.triangulation_.clear();
    auto params = geometry::Curves2dSampleParams::adaptive();
    curves2d.fill(data.triangulation_, params, geometry::WindingRule::Odd);

    data.isGeometryComputed_ = true;
    data.isComputing_ = false;

    // XXX clear less ?
    data.graphics_.clear();
}

void VacKeyFace::onBoundaryGeometryChanged() {
    frameData_.clear();
    bbox_ = geometry::Rect2d::empty;
}

void VacKeyFace::onUpdateError_() {
    removeVacNode();
}

void VacKeyFace::onStyleChanged_() {
}

} // namespace vgc::workspace
