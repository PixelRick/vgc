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

#include <vgc/workspace/workspace.h>

#include <functional>
#include <memory>

#include <vgc/dom/strings.h>
#include <vgc/topology/operations.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/edge.h>
#include <vgc/workspace/face.h>
#include <vgc/workspace/layer.h>
#include <vgc/workspace/logcategories.h>
#include <vgc/workspace/vertex.h>

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

// This is a helper to manage a shared boolean status flag.
//
// Sets the given shared boolean to true on construction and restores it to its previous
// value on destruction.
//
// For instance if function A and B want to signal that they are being executed, you can
// construct a ScopedTemporaryBoolSet with the same boolean in both scopes. Whenever this
// boolean is true, it means that either A or B is in the callstack.
//
class ScopedTemporaryBoolSet {
public:
    ScopedTemporaryBoolSet(bool& ref)
        : old_(ref)
        , ref_(ref) {

        ref_ = true;
    }

    ~ScopedTemporaryBoolSet() {
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

template<
    typename Node,
    typename TreeLinksGetter = topology::detail::TreeLinksGetter<Node>>
void visitDfs(
    Node* root,
    const std::function<bool(core::TypeIdentity<Node>*, Int)>& preOrderFn,
    const std::function<void(core::TypeIdentity<Node>*, Int)>& postOrderFn) {

    Int depth = 0;
    Node* node = root;
    while (node) {
        if (preOrderFn(node, depth)) {
            // depth first, go down
            Node* firstChild = TreeLinksGetter::firstChild(node);
            if (firstChild) {
                ++depth;
                node = firstChild;
                continue;
            }
        }
        postOrderFn(node, depth); // post-order leaf
        // breadth next
        Node* next = nullptr;
        while (node) {
            next = TreeLinksGetter::next(node);
            if (next) {
                node = next;
                break;
            }
            // go up
            Node* parent = TreeLinksGetter::parent(node);
            if (parent == root) {
                node = nullptr;
                depth = 0;
                break;
            }
            --depth;
            node = parent;
            if (node) {
                postOrderFn(node, depth); // post-order parent
                parent = TreeLinksGetter::parent(node);
            }
        }
    }
}

} // namespace

Workspace::Workspace(dom::DocumentPtr document)
    : document_(document) {

    document->changed().connect(onDocumentDiff());

    vac_ = vacomplex::Complex::create();
    //vac_->changed().connect(onVacDiff());
    vac_->onNodeAboutToBeRemoved().connect(onVacNodeAboutToBeRemoved());

    rebuildTreeFromDom_();
    rebuildVacFromTree_();
}

void Workspace::onDestroyed() {

    for (const auto& p : elements_) {
        Element* e = p.second.get();
        VacElement* ve = e->toVacElement();
        if (ve) {
            // the whole vac is cleared afterwards
            ve->vacNode_ = nullptr;
        }
        while (!e->dependents_.isEmpty()) {
            Element* dependent = e->dependents_.pop();
            dependent->dependencies_.removeOne(e);
            dependent->onDependencyRemoved_(e);
            e->onDependentElementRemoved_(dependent);
        }
    }
    elements_.clear();
    vgcElement_ = nullptr;
    elementsWithError_.clear();
    elementsToUpdateFromDom_.clear();
    vac_ = nullptr;
    document_ = nullptr;

    SuperClass::onDestroyed();
}

namespace {

std::once_flag initOnceFlag;

template<typename T>
std::unique_ptr<Element> makeUniqueElement(Workspace* workspace) {
    return std::make_unique<T>(workspace);
}

} // namespace

/* static */
WorkspacePtr Workspace::create(dom::DocumentPtr document) {

    namespace ds = dom::strings;

    std::call_once(initOnceFlag, []() {
        registerElementClass(ds::vgc, &makeUniqueElement<Layer>);
        registerElementClass(ds::layer, &makeUniqueElement<Layer>);
        registerElementClass(ds::vertex, &makeUniqueElement<VacKeyVertex>);
        //registerElementClass(ds::edge, &makeUniqueElement<KeyEdge>);
        registerElementClass(ds::edge, &makeUniqueElement<VacKeyEdge>);
        registerElementClass(ds::face, &makeUniqueElement<VacKeyFace>);
    });

    return WorkspacePtr(new Workspace(document));
}

void Workspace::registerElementClass(
    core::StringId tagName,
    ElementCreator elementCreator) {

    elementCreators_()[tagName] = elementCreator;
}

void Workspace::sync() {
    document_->emitPendingDiff();
}

void Workspace::rebuildFromDom() {
    { // flush dom diffs
        shouldSkipNextDomDiff_ = true;
        document_->emitPendingDiff();
    }
    rebuildTreeFromDom_();
    { // rebuild vac
        detail::ScopedTemporaryBoolSet sbVac(isCreatingVacElementsFromDom_);
        rebuildVacFromTree_();
    }
}

bool Workspace::updateElementFromDom(Element* element) {
    if (element->isBeingUpdated_) {
        VGC_ERROR(LogVgcWorkspace, "Cyclic update dependency detected.");
        return false;
    }
    if (element->hasPendingUpdate_) {
        element->isBeingUpdated_ = true;
        const ElementStatus oldStatus = element->status_;
        const ElementStatus newStatus = element->updateFromDom_(this);

        if (!newStatus) {
            if (oldStatus == ElementStatus::Ok) {
                elementsWithError_.emplaceLast(element);
            }
        }
        else if (!oldStatus) {
            elementsWithError_.removeOne(element);
        }

        element->status_ = newStatus;
        element->isBeingUpdated_ = false;
        element->hasPendingUpdate_ = false;
        elementsToUpdateFromDom_.removeOne(element);
    }
    return true;
}

Element* Workspace::getElementFromPathAttribute(
    dom::Element* domElement,
    core::StringId attrName,
    core::StringId tagNameFilter) const {

    dom::Element* domTargetElement =
        domElement->getElementFromPathAttribute(attrName, tagNameFilter)
            .value_or(nullptr);
    if (!domTargetElement) {
        return nullptr;
    }

    auto it = elements_.find(domTargetElement->internalId());
    return it->second.get();
}

void Workspace::visitDepthFirstPreOrder(
    const std::function<void(Element*, Int)>& preOrderFn) {
    workspace::visitDfsPreOrder<Element>(vgcElement(), preOrderFn);
}

void Workspace::visitDepthFirst(
    const std::function<bool(Element*, Int)>& preOrderFn,
    const std::function<void(Element*, Int)>& postOrderFn) {
    workspace::visitDfs<Element>(vgcElement(), preOrderFn, postOrderFn);
}

std::unordered_map<core::StringId, Workspace::ElementCreator>&
Workspace::elementCreators_() {
    static std::unordered_map<core::StringId, Workspace::ElementCreator>* instance_ =
        new std::unordered_map<core::StringId, Workspace::ElementCreator>();
    return *instance_;
}

void Workspace::removeElement_(Element* element) {
    removeElement_(element->id());
}

void Workspace::removeElement_(core::Id id) {
    auto it = elements_.find(id);
    if (it != elements_.end()) {
        Element* element = it->second.get();
        if (vgcElement_ == element) {
            vgcElement_ = nullptr;
        }
        if (element->hasError()) {
            elementsWithError_.removeOne(element);
        }
        if (element->hasPendingUpdate()) {
            elementsToUpdateFromDom_.removeOne(element);
        }
        // Note:  don't simply erase it since elements' destructor can indirectly use
        // elements_ via callbacks (e.g. onVacNodeAboutToBeRemoved_).
        std::unique_ptr<Element> uptr = std::move(it->second);
        elements_.erase(it);
        while (!element->dependents_.isEmpty()) {
            Element* dependent = element->dependents_.pop();
            dependent->dependencies_.removeOne(element);
            dependent->onDependencyRemoved_(element);
            element->onDependentElementRemoved_(dependent);
        }
        uptr.reset();
    }
}

void Workspace::clearElements_() {
    // Note: elements_.clear() indirectly calls onVacNodeAboutToBeRemoved_() and thus fills
    // elementsToUpdateFromDom_. So it is important to clear the latter after the former.
    for (const auto& p : elements_) {
        Element* e = p.second.get();
        while (!e->dependents_.isEmpty()) {
            Element* dependent = e->dependents_.pop();
            dependent->dependencies_.removeOne(e);
            dependent->onDependencyRemoved_(e);
            e->onDependentElementRemoved_(dependent);
        }
    }
    elements_.clear();
    vgcElement_ = nullptr;
    elementsWithError_.clear();
    elementsToUpdateFromDom_.clear();
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

void Workspace::debugPrintTree_() {
    visitDepthFirstPreOrder([](Element* e, Int depth) {
        VGC_DEBUG_TMP("{:>{}}<{} id=\"{}\">", "", depth * 2, e->tagName(), e->id());
    });
}

void Workspace::preElementUpdateFromVacToDom_(Element* /*element*/) {
    shouldSkipNextDomDiff_ = true;
    document_->emitPendingDiff();
}

void Workspace::postElementUpdateFromVacToDom_(Element* /*element*/) {
    // TODO: delay for batch vac-to-dom updates
    document_->emitPendingDiff();
}

void Workspace::rebuildDomFromTree_() {
    // todo later
    throw core::RuntimeError("not implemented");
}

void Workspace::onVacNodeAboutToBeRemoved_(vacomplex::Node* node) {
    // Note: Should we bypass the following logic if deletion comes from workspace ?
    //       Currently it is done by erasing the workspace element from elements_
    //       before doing the vac element removal so that this whole callback is not called.
    auto it = elements_.find(node->id());
    if (it != elements_.end()) {
        Element* element = it->second.get();
        VacElement* vacElement = element->toVacElement();
        if (vacElement && vacElement->vacNode_) {
            vacElement->vacNode_ = nullptr;
            if (!element->isBeingUpdated_) {
                // TODO: only clear graphics and append to corrupt list (elementsWithError_)
                element->hasPendingUpdate_ = true;
                elementsToUpdateFromDom_.emplaceLast(element);
            }
        }
    }
}

void Workspace::onVacNodeCreated_(
    vacomplex::Node* node,
    core::Span<vacomplex::Node*> operationSourceNodes) {

    // TODO: check id conflict

    Element* parent = find(node->parentGroup()->id());
    if (!parent) {
        VGC_ERROR(LogVgcWorkspace, "Unexpected vacomplex::Node parent.");
        return;
    }

    // create the workspace element
    std::unique_ptr<Element> u = {};
    if (node->isGroup()) {
        vacomplex::Group* group = node->toGroup();
        u = makeUniqueElement<Layer>(this);
    }
    else {
        vacomplex::Cell* cell = node->toCell();
        switch (cell->cellType()) {
        case vacomplex::CellType::KeyVertex:
            u = makeUniqueElement<VacKeyVertex>(this);
            break;
        case vacomplex::CellType::KeyEdge:
            u = makeUniqueElement<VacKeyEdge>(this);
            break;
        case vacomplex::CellType::KeyFace:
            u = makeUniqueElement<VacKeyFace>(this);
            break;
        case vacomplex::CellType::InbetweenVertex:
        case vacomplex::CellType::InbetweenEdge:
        case vacomplex::CellType::InbetweenFace:
            break;
        }
    }

    VacElement* e = u->toVacElement();
    if (!e) {
        // TODO: error ?
        return;
    }

    const auto& p = elements_.emplace(node->id(), std::move(u));
    if (!p.second) {
        // TODO: should probably throw
        return;
    }

    e->id_ = node->id();
    if (parent) {
        parent->appendChild(e);
    }

    preElementUpdateFromVacToDom_(e);
    // TODO: e->updateFromVac
    postElementUpdateFromVacToDom_(e);
}

void Workspace::onVacCellGeometryChanged_(vacomplex::Cell* cell) {
    if (isCreatingVacElementsFromDom_) {
    }

    Element* element = find(cell->id());
    if (!element) {
        VGC_ERROR(LogVgcWorkspace, "Unexpected vacomplex::Cell");
        // TODO: recover from error by creating the Cell in workspace and DOM ?
        return;
    }
    preElementUpdateFromVacToDom_(element);
    // TODO
    postElementUpdateFromVacToDom_(element);
}

Element*
Workspace::createAppendElementFromDom_(dom::Element* domElement, Element* parent) {
    if (!domElement) {
        return nullptr;
    }

    std::unique_ptr<Element> u = {};
    auto& creators = elementCreators_();
    auto it = creators.find(domElement->tagName());
    if (it != creators.end()) {
        u = it->second(this);
        if (!u) {
            VGC_ERROR(
                LogVgcWorkspace,
                "Element creator for \"{}\" failed to create the element.",
                domElement->tagName());
            // XXX throw or fallback to UnsupportedElement or nullptr ?
            u = std::make_unique<UnsupportedElement>(this);
        }
    }
    else {
        u = std::make_unique<UnsupportedElement>(this);
    }

    Element* e = u.get();
    e->domElement_ = domElement;
    const auto& p = elements_.emplace(domElement->internalId(), std::move(u));
    if (!p.second) {
        // TODO: should probably throw
        return nullptr;
    }

    e->id_ = domElement->internalId();
    if (parent) {
        parent->appendChild(e);
    }

    return e;
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
    clearElements_();

    // reset vac
    {
        detail::ScopedTemporaryBoolSet bgVac(isCreatingVacElementsFromDom_);
        vac_->clear();
        vac_->emitPendingDiff();
    }

    if (!document_) {
        return;
    }

    // flush dom diff
    {
        detail::ScopedTemporaryBoolSet bgDom(isCreatingDomElementsFromVac_);
        document_->emitPendingDiff();
    }

    dom::Element* domVgcElement = document_->rootElement();
    if (!domVgcElement || domVgcElement->tagName() != ds::vgc) {
        return;
    }

    Element* vgcElement = createAppendElementFromDom_(domVgcElement, nullptr);
    VGC_ASSERT(vgcElement->isVacElement());
    vgcElement_ = static_cast<VacElement*>(vgcElement);

    Element* p = nullptr;
    Element* e = vgcElement_;
    dom::Element* domElement = rebuildTreeFromDomIter(e, p);
    while (domElement) {
        e = createAppendElementFromDom_(domElement, p);
        domElement = rebuildTreeFromDomIter(e, p);
    }

    // children should already be in the correct order.
}

void Workspace::rebuildVacFromTree_() {
    if (!document_ || !vgcElement_) {
        return;
    }

    //namespace ds = dom::strings;

    detail::ScopedTemporaryBoolSet bgVac(isCreatingVacElementsFromDom_);

    // reset vac
    vac_->clear();
    vac_->resetRoot(vgcElement_->id());
    vgcElement_->vacNode_ = vac_->rootGroup();

    Element* root = vgcElement_;
    Element* element = root->firstChild();
    Int depth = 1;
    while (element) {
        updateElementFromDom(element);
        iterDfsPreOrder(element, depth, root);
    }

    updateVacHierarchyFromTree_();

    lastSyncedDomVersionId_ = document_->versionId();
    changed().emit();
}

void Workspace::updateVacHierarchyFromTree_() {
    // todo: sync children order in all groups
    Element* root = vgcElement_;
    Element* e = root;
    Int depth = 0;
    while (e) {
        vacomplex::Node* node = e->vacNode();
        if (node) {
            if (node->isGroup()) {
                VacElement* child = e->firstChildVacElement();
                if (child) {
                    vacomplex::Group* g = static_cast<vacomplex::Group*>(node);
                    topology::ops::moveToGroup(child->vacNode(), g, g->firstChild());
                }
            }

            if (e->parent()) {
                VacElement* next = e->nextVacElement();
                topology::ops::moveToGroup(
                    node, node->parentGroup(), (next ? next->vacNode() : nullptr));
            }
        }

        iterDfsPreOrder(e, depth, root);
    }
}

//bool Workspace::haveKeyEdgeBoundaryPathsChanged_(Element* e) {
//    vacomplex::Node* node = e->vacNode();
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
//    vacomplex::Node* node = vacNode();
//    vacomplex::KeyEdge* kv = node ? node->toCellUnchecked()->toKeyEdgeUnchecked() : nullptr;
//    if (!kv) {
//        return false;
//    }
//
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

    detail::ScopedTemporaryBoolSet bgVac(isCreatingVacElementsFromDom_);

    namespace ds = dom::strings;

    // impl goal: we want to keep as much cached data as possible.
    //            we want the vac to be valid -> using only its operators
    //            limits bugs to their implementation.

    bool hasModifiedPaths =
        (diff.removedNodes().length() > 0) || (diff.reparentedNodes().size() > 0);
    bool hasNewPaths = (diff.createdNodes().length() > 0);

    std::set<Element*> parentsToOrderSync;

    // first we remove what has to be removed
    // this can remove dependent vac nodes (star).
    for (dom::Node* node : diff.removedNodes()) {
        dom::Element* domElement = dom::Element::cast(node);
        if (!domElement) {
            continue;
        }
        Element* element = find(domElement);
        if (element) {
            Element* parent = element->parent();
            VGC_ASSERT(parent);
            // reparent children to element's parent
            for (Element* child : *element) {
                parent->appendChild(child);
            }
            element->unlink();
            removeElement_(element);
        }
    }

    // create new elements
    for (dom::Node* node : diff.createdNodes()) {
        dom::Element* domElement = dom::Element::cast(node);
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
            //            and <vgc> element should already exist.
            continue;
        }
        // will be reordered afterwards
        Element* element = createAppendElementFromDom_(domElement, parent);
        element->hasPendingUpdate_ = true;
        elementsToUpdateFromDom_.emplaceLast(element);
        parentsToOrderSync.insert(parent);
    }

    // Collect all parents with reordered children.
    for (dom::Node* node : diff.reparentedNodes()) {
        dom::Element* domElement = dom::Element::cast(node);
        if (!domElement) {
            continue;
        }
        dom::Element* domParentElement = domElement->parentElement();
        if (!domParentElement) {
            continue;
        }
        Element* parent = find(domParentElement);
        if (parent) {
            parentsToOrderSync.insert(parent);
        }
    }
    for (dom::Node* node : diff.childrenReorderedNodes()) {
        dom::Element* domElement = dom::Element::cast(node);
        if (!domElement) {
            continue;
        }
        Element* element = find(domElement);
        if (!element) {
            // XXX error ?
            continue;
        }
        parentsToOrderSync.insert(element);
    }

    // Update tree hierarchy from dom.
    for (Element* element : parentsToOrderSync) {
        Element* child = element->firstChild();
        dom::Element* domChild = element->domElement()->firstChildElement();
        while (domChild) {
            if (!child || child->domElement() != domChild) {
                Element* firstChild = find(domChild);
                VGC_ASSERT(firstChild);
                element->insertChildUnchecked(child, firstChild);
                child = firstChild;
            }
            child = child->nextVacElement();
            domChild = domChild->nextSiblingElement();
        }
    }

    if (hasNewPaths || hasModifiedPaths) {
        // Flag all elements with error for update.
        for (Element* element : elementsWithError_) {
            if (!element->hasPendingUpdate_) {
                element->hasPendingUpdate_ = true;
                elementsToUpdateFromDom_.emplaceLast(element);
            }
        }
    }

    if (hasModifiedPaths) {
        // Update everything for now.
        // TODO: An element dependent on a Path should have it in its dependencies
        // so we could force path reevaluation to the dependents of an element that moved,
        // as well as errored elements.
        Element* root = vgcElement_;
        Element* element = root;
        Int depth = 0;
        while (element) {
            if (!element->hasPendingUpdate_) {
                element->hasPendingUpdate_ = true;
                elementsToUpdateFromDom_.emplaceLast(element);
            }
            iterDfsPreOrder(element, depth, root);
        }
    }
    else {
        // Otherwise we update the elements flagged as modified
        for (const auto& it : diff.modifiedElements()) {
            Element* element = find(it.first);
            // If the element has already an update pending it will be
            // taken care of in the update loop further below.
            if (element && !element->hasPendingUpdate_) {
                element->hasPendingUpdate_ = true;
                // TODO: pass the set of modified attributes ids to the Element
                updateElementFromDom(element);
            }
        }
    }

    // An update can schedule another so we try to exhaust the list
    // instead of simply traversing it.
    while (!elementsToUpdateFromDom_.isEmpty()) {
        // There is no need to pop the element since updateElementFromDom is
        // in charge of removing it from the list when updated.
        Element* element = elementsToUpdateFromDom_.last();
        updateElementFromDom(element);
    }

    updateVacHierarchyFromTree_();

    lastSyncedDomVersionId_ = document_->versionId();
    changed().emit();
}

//void Workspace::updateTreeAndDomFromVac_(const vacomplex::Diff& /*diff*/) {
//    if (!document_) {
//        VGC_ERROR(LogVgcWorkspace, "DOM is null.")
//        return;
//    }
//
//    detail::ScopedTemporaryBoolSet bgDom(isCreatingDomElementsFromVac_);
//
//    // todo later
//    throw core::RuntimeError("not implemented");
//}

void Workspace::onDocumentDiff_(const dom::Diff& diff) {
    if (shouldSkipNextDomDiff_) {
        // It means workspace did the changes, no need to see the diff.
        shouldSkipNextDomDiff_ = false;
        return;
    }
    updateTreeAndVacFromDom_(diff);
}

} // namespace vgc::workspace
