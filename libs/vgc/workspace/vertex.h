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

#ifndef VGC_WORKSPACE_VERTEX_H
#define VGC_WORKSPACE_VERTEX_H

#include <vgc/core/arithmetic.h>
#include <vgc/core/array.h>
#include <vgc/dom/element.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/element.h>

namespace vgc::workspace {

class VGC_WORKSPACE_API Vertex : public Element {
private:
    friend class Workspace;

protected:
    Vertex(dom::Element* domElement)
        : Element(domElement) {
    }

public:
    topology::VertexCell* vacVertex() const {
        topology::VacNode* n = vacNode();
        return n ? n->toCellUnchecked()->toVertexCellUnchecked() : nullptr;
    }
};

class VGC_WORKSPACE_API KeyVertex : public Vertex {
private:
    friend class Workspace;

public:
    ~KeyVertex() override = default;

    KeyVertex(dom::Element* domElement)
        : Vertex(domElement) {
    }

    topology::KeyVertex* vacKeyVertex() const {
        topology::VacNode* n = vacNode();
        return n ? n->toCellUnchecked()->toKeyVertexUnchecked() : nullptr;
    }

protected:
    geometry::Rect2d boundingBox(core::AnimTime t) override;

    void onDomAttributesChanged() override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) override;

private:
};

class VGC_WORKSPACE_API InbetweenVertex : public Vertex {
private:
    friend class Workspace;

public:
    ~InbetweenVertex() override = default;

    InbetweenVertex(dom::Element* domElement)
        : Vertex(domElement) {
    }

    topology::InbetweenVertex* vacInbetweenVertex() const {
        topology::VacNode* n = vacNode();
        return n ? n->toCellUnchecked()->toInbetweenVertexUnchecked() : nullptr;
    }

protected:
    geometry::Rect2d boundingBox(core::AnimTime t) override;

    void onDomAttributesChanged() override;

    void prepareForFrame_(core::AnimTime t = {}) override;

    void paint_(
        graphics::Engine* engine,
        core::AnimTime t,
        PaintOptions flags = PaintOption::None) override;

private:
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_VERTEX_H
