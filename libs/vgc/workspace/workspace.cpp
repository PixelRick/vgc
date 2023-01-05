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

#include <functional>
#include <memory>

#include <vgc/dom/strings.h>
#include <vgc/topology/operations.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/edge.h>
#include <vgc/workspace/layer.h>
#include <vgc/workspace/logcategories.h>
#include <vgc/workspace/vertex.h>
#include <vgc/workspace/workspace.h>

namespace vgc::workspace {

namespace detail {

struct VacElementLists {
    // groups are in Dfs order
    core::Array<Element*> groups;
    core::Array<Element*> keyVertices;
    core::Array<Element*> keyEdges;
    core::Array<Element*> keyFaces;
    core::Array<Element*> inbetweenVertices;
    core::Array<Element*> inbetweenEdges;
    core::Array<Element*> inbetweenFaces;
};

class ScopedRecursiveBoolGuard {
public:
    ScopedRecursiveBoolGuard(bool& ref)
        : old_(ref)
        , ref_(ref) {

        ref_ = true;
    }

    ~ScopedRecursiveBoolGuard() {
        ref_ = old_;
    }

private:
    bool old_;
    bool& ref_;
};

} // namespace detail

namespace {

// Assumes (parent == it->parent()).
template<
    typename Node,
    typename TreeLinksGetter = topology::detail::TreeLinksGetter<Node>>
void iterDfsPreOrderSkipChildren(Node*& it, Int& depth, core::TypeIdentity<Node>* root) {
    Node* next = nullptr;
    // breadth next
    while (it) {
        next = TreeLinksGetter::next(it);
        if (next) {
            it = next;
            return;
        }
        // go up
        Node* p = TreeLinksGetter::parent(it);
        it = p;
        --depth;
        if (it == root) {
            it = nullptr;
            return;
        }
        if (p) {
            p = TreeLinksGetter::parent(p);
        }
    }
}

// Assumes (parent == it->parent()).
template<
    typename Node,
    typename TreeLinksGetter = topology::detail::TreeLinksGetter<Node>>
void iterDfsPreOrder(Node*& it, Int& depth, core::TypeIdentity<Node>* root) {
    Node* next = nullptr;
    // depth first
    next = TreeLinksGetter::firstChild(it);
    if (next) {
        ++depth;
        it = next;
        return;
    }
    // breadth next
    iterDfsPreOrderSkipChildren(it, depth, root);
}

template<
    typename Node,
    typename TreeLinksGetter = topology::detail::TreeLinksGetter<Node>>
void iterDfsPreOrder(
    Node*& it,
    Int& depth,
    core::TypeIdentity<Node>* root,
    bool skipChildren) {

    if (skipChildren) {
        iterDfsPreOrderSkipChildren(it, depth, root);
    }
    else {
        iterDfsPreOrder(it, depth, root);
    }
}

template<
    typename Node,
    typename TreeLinksGetter = topology::detail::TreeLinksGetter<Node>>
void visitDfsPreOrder(Node* root, std::function<void(core::TypeIdentity<Node>*, Int)> f) {
    Node* e = root;
    Int depth = 0;
    while (e) {
        f(e, depth);
        iterDfsPreOrder(e, depth, root);
    }
}

} // namespace

Workspace::Workspace(dom::DocumentPtr document)
    : document_(document) {

    document->changed().connect(onDocumentDiff());

    vac_ = topology::Vac::create();
    vac_->changed().connect(onVacDiff());
    vac_->onNodeAboutToBeRemoved().connect(onVacNodeAboutToBeRemoved());

    rebuildTreeFromDom_();
    rebuildVacFromTree_();
}

namespace {

std::once_flag initOnceFlag;

template<typename T>
std::unique_ptr<Element> makeUniqueElement(dom::Element* domElement) {
    return std::make_unique<T>(domElement);
}

} // namespace

/* static */
WorkspacePtr Workspace::create(dom::DocumentPtr document) {

    namespace ds = dom::strings;

    std::call_once(initOnceFlag, []() {
        registerElementClass(ds::vertex, &makeUniqueElement<KeyVertex>);
        //registerElementClass(ds::edge, &makeUniqueElement<KeyEdge>);
        registerElementClass(ds::edge, &makeUniqueElement<KeyEdge>);
    });

    return WorkspacePtr(new Workspace(document));
}

std::unordered_map<core::StringId, Workspace::ElementCreator>&
Workspace::elementCreators() {
    static std::unordered_map<core::StringId, Workspace::ElementCreator>* instance_ =
        new std::unordered_map<core::StringId, Workspace::ElementCreator>();
    return *instance_;
}

void Workspace::registerElementClass(
    core::StringId tagName,
    ElementCreator elementCreator) {

    elementCreators()[tagName] = elementCreator;
}

void Workspace::sync() {
    // slots do check if a rebuild is necessary
    document_->emitPendingDiff();
    vac_->emitPendingDiff();
}

void Workspace::rebuildFromDom() {
    // disable update-on-diff
    detail::ScopedRecursiveBoolGuard bgDom(isDomBeingUpdated_);
    detail::ScopedRecursiveBoolGuard bgVac(isVacBeingUpdated_);
    // flush dom diffs
    document_->emitPendingDiff();
    // rebuild
    rebuildTreeFromDom_();
    rebuildVacFromTree_();
    // flush vac diffs
    vac_->emitPendingDiff();
}

Element* Workspace::getElementFromPathAttribute(
    dom::Element* domElement,
    core::StringId attrName,
    core::StringId tagNameFilter) const {

    dom::Element* domTargetElement =
        domElement->getElementFromPathAttribute(attrName, tagNameFilter);
    if (!domTargetElement) {
        return nullptr;
    }

    auto it = elements_.find(domTargetElement->internalId());
    return it->second.get();
}

void Workspace::onDocumentDiff_(const dom::Diff& diff) {
    if (isDomBeingUpdated_) {
        // workspace is doing the changes, no need to see the diff.
        return;
    }

    bool vacChanged = vac_->version() != lastSyncedVacVersion_;
    if (vacChanged) {
        VGC_ERROR(
            LogVgcWorkspace,
            "Both DOM and VAC of workspace have been edited since last synchronization. "
            "Rebuilding VAC from DOM.");
        rebuildFromDom();
        return;
    }

    updateTreeAndVacFromDom_(diff);
}

void Workspace::onVacDiff_(const topology::VacDiff& diff) {
    if (isVacBeingUpdated_) {
        // workspace is doing the changes, no need to see the diff.
        return;
    }

    bool domChanged = document_ && (document_->versionId() != lastSyncedDomVersionId_);
    if (domChanged) {
        VGC_ERROR(
            LogVgcWorkspace,
            "Both DOM and VAC of workspace have been edited since last synchronization. "
            "Rebuilding VAC from DOM.");
        rebuildFromDom();
        return;
    }

    updateTreeAndDomFromVac_(diff);
}

Element* Workspace::createAppendElement_(dom::Element* domElement, Element* parent) {
    if (!domElement) {
        return nullptr;
    }

    std::unique_ptr<Element> u = {};
    auto& creators = elementCreators();
    auto it = creators.find(domElement->tagName());
    if (it != creators.end()) {
        u = it->second(domElement);
        if (!u) {
            VGC_ERROR(
                LogVgcWorkspace,
                "Element creator for \"{}\" failed to create the element.",
                domElement->tagName());
            // XXX throw or fallback to UnknownElement or nullptr ?
            u = std::make_unique<UnknownElement>(domElement);
        }
    }
    else {
        u = std::make_unique<UnknownElement>(domElement);
    }

    Element* e = u.get();
    const auto& p = elements_.emplace(domElement->internalId(), std::move(u));
    if (!p.second) {
        // XXX should probably throw
        return nullptr;
    }

    e->id_ = domElement->internalId();
    if (parent) {
        parent->appendChild(e);
    }

    return e;
}

void Workspace::onVacNodeAboutToBeRemoved_(topology::VacNode* node) {
    auto it = elements_.find(node->id());
    if (it != elements_.end()) {
        Element* e = it->second.get();
        if (e->isVacElement()) {
            static_cast<VacElement*>(e)->vacNode_ = nullptr;
        }
    }
}

void Workspace::fillVacElementListsUsingTagName_(
    Element* root,
    detail::VacElementLists& ce) const {

    namespace ds = dom::strings;

    Element* e = root->firstChild();
    Int depth = 1;

    while (e) {
        bool skipChildren = true;

        core::StringId tagName = e->domElement_->tagName();
        if (tagName == ds::vertex) {
            ce.keyVertices.append(e);
        }
        else if (tagName == ds::edge) {
            ce.keyEdges.append(e);
        }
        else if (tagName == ds::layer) {
            ce.groups.append(e);
            skipChildren = false;
        }

        iterDfsPreOrder(e, depth, root, skipChildren);
    }
}

namespace {

// Updates parent.
// Assumes (parent == it->parent()).
dom::Element* rebuildTreeFromDomIter(Element* it, Element*& parent) {
    dom::Element* e = it->domElement();
    dom::Element* next = nullptr;
    // depth first
    next = e->firstChildElement();
    if (next) {
        parent = it;
        return next;
    }
    // breadth next
    while (e) {
        next = e->nextSiblingElement();
        if (next) {
            return next;
        }
        // go up
        if (!parent) {
            return nullptr;
        }
        e = parent->domElement();
        parent = parent->parent();
    }
    return next;
}

} // namespace

void Workspace::rebuildTreeFromDom_() {

    namespace ds = dom::strings;

    // reset tree
    elements_.clear();
    vgcElement_ = nullptr;

    // reset vac
    {
        detail::ScopedRecursiveBoolGuard bgVac(isVacBeingUpdated_);
        vac_->clear();
        vac_->emitPendingDiff();
        lastSyncedVacVersion_ = -1;
    }

    if (!document_) {
        return;
    }

    // flush dom diff
    {
        detail::ScopedRecursiveBoolGuard bgDom(isDomBeingUpdated_);
        document_->emitPendingDiff();
    }

    dom::Element* domVgcElement = document_->rootElement();
    if (!domVgcElement || domVgcElement->tagName() != ds::vgc) {
        return;
    }

    Element* vgcElement = createAppendElement_(domVgcElement, nullptr);
    VGC_ASSERT(vgcElement->isVacElement());
    vgcElement_ = static_cast<VacElement*>(vgcElement);

    Element* p = nullptr;
    Element* e = vgcElement_;
    dom::Element* domElement = rebuildTreeFromDomIter(e, p);
    while (domElement) {
        e = createAppendElement_(domElement, p);
        domElement = rebuildTreeFromDomIter(e, p);
    }

    // children should already be in the correct order.
}

void Workspace::rebuildDomFromTree_() {
    // todo later
    throw core::RuntimeError("not implemented");
}

void Workspace::rebuildTreeFromVac_() {
    // todo later
    throw core::RuntimeError("not implemented");
}

void Workspace::rebuildVacFromTree_() {
    if (!document_ || !vgcElement_) {
        return;
    }

    //namespace ds = dom::strings;

    detail::ScopedRecursiveBoolGuard bgVac(isVacBeingUpdated_);

    // reset vac
    vac_->clear();
    vac_->emitPendingDiff();

    vac_->clear();
    lastSyncedVacVersion_ = -1;

    vgcElement_->vacNode_ = vac_->rootGroup();

    // all workspace elements vac node pointers should be null
    // thanks to `onVacNodeAboutToBeRemoved`

    //detail::VacElementLists ce;
    //fillVacElementListsUsingTagName_(vgcElement_, ce);
    //
    //for (Element* e : ce.groups) {
    //    e->updateFromDom();
    //}
    //
    //for (Element* e : ce.keyVertices) {
    //    e->updateFromDom();
    //}
    //
    //for (Element* e : ce.keyEdges) {
    //    e->updateFromDom();
    //}

    Element* root = vgcElement_->firstChild();
    Element* e = root->firstChild();
    Int depth = 1;
    while (e) {
        e->updateFromDom(this);
        iterDfsPreOrder(e, depth, root);
    }

    updateVacHierarchyFromTree_();

    lastSyncedDomVersionId_ = document_->versionId();
    lastSyncedVacVersion_ = vac_->version();
}

void Workspace::updateVacHierarchyFromTree_() {
    // todo: sync children order in all groups
    Element* root = vgcElement_;
    Element* e = root;
    Int depth = 0;
    while (e) {
        topology::VacNode* node = e->vacNode();
        if (!node) {
            continue;
        }

        if (node->isGroup()) {
            VacElement* child = e->firstChildVacElement();
            if (child) {
                topology::VacGroup* g = static_cast<topology::VacGroup*>(node);
                topology::ops::moveToGroup(child->vacNode(), g, g->firstChild());
            }
        }

        if (e->parent()) {
            VacElement* next = e->nextVacElement();
            topology::ops::moveToGroup(
                node, node->parentGroup(), (next ? next->vacNode() : nullptr));
        }

        iterDfsPreOrder(e, depth, root);
    }
}

//bool Workspace::haveKeyEdgeBoundaryPathsChanged_(Element* e) {
//    topology::VacNode* node = e->vacNode();
//    if (!node) {
//        return false;
//    }
//
//    namespace ds = dom::strings;
//    dom::Element* const domElem = e->domElement();
//
//    Element* ev0 = getElementFromPathAttribute(domElem, ds::startvertex, ds::vertex);
//    Element* ev1 = getElementFromPathAttribute(domElem, ds::endvertex, ds::vertex);
//
//    topology::KeyEdge* kv = node->toCellUnchecked()->toKeyEdgeUnchecked();
//    if (ev0) {
//        if (ev0->vacNode() != kv->startVertex()) {
//            return true;
//        }
//    }
//    else if (kv->startVertex()) {
//        return true;
//    }
//    if (ev1) {
//        if (ev1->vacNode() != kv->endVertex()) {
//            return true;
//        }
//    }
//    else if (kv->endVertex()) {
//        return true;
//    }
//
//    return false;
//}

void Workspace::updateTreeAndVacFromDom_(const dom::Diff& diff) {
    if (!document_) {
        return;
    }

    detail::ScopedRecursiveBoolGuard bgVac(isVacBeingUpdated_);

    VGC_ASSERT(vac_->version() == lastSyncedVacVersion_);

    //namespace ds = dom::strings;

    namespace ds = dom::strings;

    /*
    core::Array<Node*> createdNodes_;
    core::Array<Node*> removedNodes_;
    std::set<Node*> reparentedNodes_;
    std::set<Node*> childrenReorderedNodes_;
    std::unordered_map<Element*, std::set<core::StringId>> modifiedElements_;
    */

    // impl goal: we want to keep as much cached data as possible.
    //            we want the vac to be valid -> using only its operators
    //            limits bugs to their implementation.
    // the current function isn't to be called a lot so it probably does not
    // have to be optimized.

    // first we remove what has to be removed
    // this can remove dependent vac nodes (star).
    for (dom::Node* n : diff.removedNodes()) {
        dom::Element* domElement = dom::Element::cast(n);
        if (!domElement) {
            continue;
        }
        elements_.erase(domElement->internalId());
    }

    // next create everything
    for (dom::Node* n : diff.createdNodes()) {
        dom::Element* domElement = dom::Element::cast(n);
        if (!domElement) {
            continue;
        }
        dom::Element* domParentElement = domElement->parentElement();
        if (!domParentElement) {
            continue;
        }
        Element* parent = find(domParentElement);
        if (!parent) {
            // XXX warn ? createdNodes should be in valid build order
            //            and vgc element should already exist.
            continue;
        }
        // will be reordered afterwards
        Element* e = createAppendElement_(domElement, parent);
        e->updateFromDom(this);
    }

    // now reparent+reorder tree from dom
    for (dom::Node* n : diff.reparentedNodes()) {
        dom::Element* domElement = dom::Element::cast(n);
        if (!domElement) {
            continue;
        }
        dom::Element* domParentElement = domElement->parentElement();
        Element* e = find(domElement);
        if (!e) {
            // XXX error ?
            continue;
        }
        Element* parent = find(domParentElement);
        if (!parent) {
            // XXX error ?
            continue;
        }
    }

    // reorder
    //Element* root = vgcElement_;
    //Element* e = root;
    //Int depth = 0;
    //while (e) {
    //    topology::VacNode* node = e->vacNode();
    //    if (!node) {
    //        continue;
    //    }
    //
    //    if (node->isGroup()) {
    //        VacElement* child = e->firstChildVacElement();
    //        if (child) {
    //            topology::VacGroup* g = static_cast<topology::VacGroup*>(node);
    //            topology::ops::moveToGroup(child->vacNode(), g, g->firstChild());
    //        }
    //    }
    //
    //    if (e->parent()) {
    //        VacElement* next = e->nextVacElement();
    //        topology::ops::moveToGroup(
    //            node, node->parentGroup(), (next ? next->vacNode() : nullptr));
    //    }
    //
    //    iterDfsPreOrder(e, depth, root);
    //}

    // problem: there are modified nodes that could need new nodes
    //          and there are new nodes that could need modified nodes

    //const auto& modifiedElements = diff.modifiedElements();
    //
    //// XXX we have an update order, maybe we could generalize it to
    ////     other and custom elements ?
    //
    //detail::VacElementLists ce;
    //fillVacElementListsUsingTagName_(vgcElement_, ce);
    //
    //for (Element* e : ce.groups) {
    //    if (!e->vacNode()) {
    //        rebuildVacGroup_(e);
    //    }
    //}
    //
    //for (Element* e : ce.keyVertices) {
    //    if (!e->vacNode()) {
    //        rebuildKeyVertex_(e);
    //    }
    //}
    //
    //for (Element* e : ce.keyEdges) {
    //    // this can remove vac nodes!
    //    // it could also required nodes that don't exist yet.
    //    rebuildKeyEdge_(e, false);
    //}
    //
    //for (const auto& me : diff.modifiedElements()) {
    //    dom::Element* domElement = me.first;
    //    //const std::set<core::StringId>& attrNames = me.second;
    //    //if (attrNames.find(ds::id) != attrNames.end()) {
    //    //}
    //
    //    Element* e = find(domElement->internalId());
    //
    //    // call virtual update ?
    //}

    // XXX later we could have a map of dependencies in dom directly ?
    //     looking at all Path attributes of a node gives its dependency nodes

    // XXX it becomes tricky with relative paths, we should also check star when reparented
    // haveKeyEdgeBoundaryPathsChanged_

    // first create everything new
    // processing order:
    //
    //  std::unordered_map<Element*, std::set<core::StringId>> modifiedElements_;
    //  -> invalidate and delete desynced vac parts.
    //  -> add the destroyed cells to a pending rebuild and link list.
    //
    //  core::Array<Node*> createdNodes_;
    //  -> but don't link them yet. add
    //  -> add the destroyed cells to a pending link list.
    //
    //  std::unordered_map<Element*, std::set<core::StringId>> modifiedElements_;
    //  std::set<Node*> reparentedNodes_;
    //  core::Array<Node*> removedNodes_;
    //  std::set<Node*> childrenReorderedNodes_;
    //

    //core::Array<std::unique_ptr<topology::VacCell>> createdCells;

    //for (dom::Node* n : diff.createdNodes()) {
    //    dom::Element* e = dom::Element::cast(n);
    //    if (!e) {
    //        continue;
    //    }
    //    if (e->tagName() == ss::vertex) {
    //        std::unique_ptr<topology::KeyVertex> v =
    //            topology::ops::createUnlinkedKeyVertex(e->internalId(), core::AnimTime());
    //        /*
    //            {"color", core::colors::black},
    //            {"position", geometry::Vec2d()},
    //        */
    //        topology::ops::setKeyVertexPosition(
    //            v.get(), e->getAttribute(ss::position).getVec2d());

    //        // XXX create graphics obj
    //    }
    //    else if (e->tagName() == ss::edge) {
    //        std::unique_ptr<topology::KeyEdge> v =
    //            topology::ops::createUnlinkedKeyEdge(e->internalId(), core::AnimTime());
    //        /*
    //            {"color", core::colors::black},
    //            {"positions", geometry::Vec2dArray()},
    //            {"widths", core::DoubleArray()},
    //            {"startVertex", dom::Path()},
    //            {"endVertex", dom::Path()},
    //        */

    //        // will we deal with scripts here or will dom do it itself ?
    //    }
    //    else if (e->tagName() == ss::layer) {
    //        std::unique_ptr<topology::VacGroup> g =
    //            topology::ops::createUnlinkedVacGroup(e->internalId());
    //    }
    //}

    //for (const auto& me : diff.modifiedElements()) {
    //    dom::Element* e = me.first;
    //    const std::set<core::StringId>& attrs = me.second;
    //    if (!e) {
    //        continue;
    //    }
    //    if (e->tagName() == ss::vertex) {
    //        //
    //    }
    //    else if (e->tagName() == ss::edge) {
    //        // What if startVertex or endVertex changes ?
    //        // It is not really a valid vac operation.
    //        //
    //    }
    //    else if (e->tagName() == ss::layer) {
    //        //
    //    }
    //}

    //std::set<topology::VacCell*> aliveStar;
}

void Workspace::updateTreeAndDomFromVac_(const topology::VacDiff& /*diff*/) {
    if (!document_) {
        VGC_ERROR(LogVgcWorkspace, "DOM is null.")
        return;
    }

    detail::ScopedRecursiveBoolGuard bgDom(isDomBeingUpdated_);

    // todo later
    throw core::RuntimeError("not implemented");
}

void Workspace::debugPrintTree_() {
    visitDfsPreOrder<Element>(vgcElement_, [](Element* e, Int depth) {
        VGC_DEBUG_TMP("{:>{}}<{} id=\"{}\">", "", depth * 2, e->tagName(), e->id());
    });
}

} // namespace vgc::workspace
