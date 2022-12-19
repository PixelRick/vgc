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
#include <vgc/core/flags.h>
#include <vgc/core/id.h>
#include <vgc/core/object.h>
#include <vgc/dom/document.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>
#include <vgc/workspace/element.h>

namespace vgc::workspace {

VGC_DECLARE_OBJECT(Workspace);

namespace detail {

struct VacElementLists;

} // namespace detail

// clang-format off
enum class ElementDiffFlag {
    Created             = 0x01,
    Reparented          = 0x02,
    ChildrenChanged     = 0x04,
    AttributeChanged    = 0x08,
    GeometryChanged     = 0x10,
    CellStarChanged     = 0x20,
    CellBoundaryChanged = 0x40
};
VGC_DEFINE_FLAGS(ElementDiffFlags, ElementDiffFlag)
// clang-format on

class VGC_WORKSPACE_API Diff {
public:
    Diff() = default;

    void reset() {
        removedElements_.clear();
        elementDiffs_.clear();
    }

    bool isEmpty() const {
        return removedElements_.empty() && elementDiffs_.empty();
    }

    const std::unordered_map<Element*, ElementDiffFlags>& nodeDiffs() const {
        return elementDiffs_;
    }

    const core::Array<core::Id>& removedNodes() const {
        return removedElements_;
    }

    // XXX ops helpers

    void onNodeRemoved(Element* element) {
        elementDiffs_.erase(element);
        removedElements_.append(element->id());
        // XXX can it happen that we re-add a node with a same id ?
    }

    void onElementDiff(Element* element, ElementDiffFlags diffFlags) {
        // XXX can it happen that we re-add a node with a same id as a removed node ?
        elementDiffs_[element].set(diffFlags);
    }

private:
    std::unordered_map<Element*, ElementDiffFlags> elementDiffs_;
    core::Array<core::Id> removedElements_;
};

/// \class vgc::workspace::Workspace
/// \brief Represents a workspace to manage, manipulate and visit a vector graphics scene.
///
class VGC_WORKSPACE_API Workspace : public core::Object {
private:
    VGC_OBJECT(Workspace, core::Object)

    Workspace(dom::DocumentPtr document);

public:
    static WorkspacePtr create(dom::DocumentPtr document);

    /*RenderObject*
    getOrCreateRenderObject(core::Id id, graphics::Engine* engine, core::AnimTime t);*/

    //RenderObject* getOrCreateRenderObject(
    //    Renderable* renderable,
    //    graphics::Engine* engine,
    //    core::AnimTime t);

    dom::Document* document() const {
        return document_.get();
    }

    const topology::Vac* vac() const {
        return vac_.get();
    }

    core::History* history() const {
        return document_.get()->history();
    }

    void sync();

    VGC_SIGNAL(changed, (const Diff&, diff));

    VGC_SLOT(onDocumentDiff, onDocumentDiff_)
    VGC_SLOT(onVacDiff, onVacDiff_)

protected:
    /*RenderObjectMap* getRenderObjectMap(graphics::Engine* engine, core::AnimTime t) {
        return renderObjectByIdByTimeByEngine_[engine][t].get();
    }*/

    void onDocumentDiff_(const dom::Diff& diff);
    void onVacDiff_(const topology::VacDiff& diff);

    void updateTreeAndVacFromDom_();
    void updateTreeAndDomFromVac_();

    void updateTreeFromDom_();

    void rebuildTreeFromDom_();
    void rebuildVacFromTree_();

    void
    fillVacElementListsUsingTagName(Element* root, detail::VacElementLists& ce) const;

private:
    std::unordered_map<core::Id, std::unique_ptr<Element>> elements_;
    Element* vgcElement_;

    Element* createElement(dom::Element* domElement, Element* parent);

    Element* getRefAttribute(
        dom::Element* domElement,
        core::StringId name,
        core::StringId tagNameFilter = {}) const;

    dom::DocumentPtr document_;
    topology::VacPtr vac_;
    bool isDomBeingUpdated_ = false;
    bool isVacBeingUpdated_ = false;
    core::Id lastSyncedDomVersionId_ = {};
    Int64 lastSyncedVacVersion_ = -1;
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_WORKSPACE_H
