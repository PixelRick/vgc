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

#ifndef VGC_WORKSPACE_WORKSPACE_H
#define VGC_WORKSPACE_WORKSPACE_H

#include <map>
#include <unordered_map>

#include <vgc/core/animtime.h>
#include <vgc/core/id.h>
#include <vgc/core/object.h>
#include <vgc/dom/document.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/renderable.h>

namespace vgc::graphics {

class Engine;

} // namespace vgc::graphics

namespace vgc::workspace {

VGC_DECLARE_OBJECT(Workspace);

struct VGC_WORKSPACE_API ElementPointers {
protected:
    ElementPointers(dom::Element* domElement, topology::VacNode* vacNode)
        : domElement_(domElement)
        , vacNode_(vacNode) {
    }

    dom::Element* domElement() const {
        return domElement_;
    }

    topology::VacNode* vacNode() const {
        return vacNode_;
    }

private:
    dom::Element* domElement_;
    topology::VacNode* vacNode_;
};

/// \class vgc::workspace::Workspace
/// \brief Represents a workspace to manage, manipulate and visit a vector graphics scene.
///
class VGC_WORKSPACE_API Workspace : public core::Object {
private:
    VGC_OBJECT(Workspace, core::Object)

    template<typename T>
    using ByEngineMap = std::unordered_map<graphics::Engine*, T>;

    template<typename T>
    using ByTimeMap = std::map<core::AnimTime, T>;

    using RenderObjectMap = std::map<core::Id, std::unique_ptr<RenderObject>>;

public:
    Workspace(dom::DocumentPtr document);

    RenderObject*
    getOrCreateRenderObject(core::Id id, graphics::Engine* engine, core::AnimTime t);

    RenderObject* getOrCreateRenderObject(
        Renderable* renderable,
        graphics::Engine* engine,
        core::AnimTime t);

    VGC_SLOT(onDocumentChanged, onDocumentChanged_)
    VGC_SLOT(onVacChanged, onVacChanged_)

protected:
    RenderObjectMap* getRenderObjectMap(graphics::Engine* engine, core::AnimTime t) {
        return renderObjectByIdByTimeByEngine_[engine][t].get();
    }

    void onDocumentChanged_(const dom::Diff& diff);
    void onVacChanged_(const topology::VacDiff& diff);

    void initVacFromDocument();

private:
    ByEngineMap<ByTimeMap<std::unique_ptr<RenderObjectMap>>>
        renderObjectByIdByTimeByEngine_;

    std::unordered_map<core::Id, ElementPointers> idToPointers_;

    dom::DocumentPtr document_;
    topology::VacPtr vac_;
    bool isSyncOngoing_ = false;
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_WORKSPACE_H
