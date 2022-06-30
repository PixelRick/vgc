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

#ifndef VGC_GRAPHICS_RESOURCE_H
#define VGC_GRAPHICS_RESOURCE_H

#include <atomic>
#include <mutex>
#include <unordered_set>
#include <utility>

#include <vgc/core/array.h>
#include <vgc/core/arithmetic.h>
#include <vgc/core/object.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/logcategories.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

class Resource;

namespace detail {

// Used to list all resources so that when the engine is stopped we can release
// them all. It is important if the engine is a temporary wrapper (e.g. around
// Qt OpenGL), better not leak resources.
//
class VGC_GRAPHICS_API ResourceList {
protected:
    friend Engine;
    friend Resource;

    ResourceList() = default;

    ~ResourceList();

    std::unordered_set<Resource*> resources_;

    // Resources must be kept alive until all commands using them are executed.
    // The engine is responsible for properly scheduling the release of these resources.
    //
    core::Array<Resource*> danglingResources_;
};

} // namespace detail

/// \class vgc::graphics::Resource
/// \brief Abstract graphics resource.
///
class VGC_GRAPHICS_API Resource {
protected:
    using ResourceList = detail::ResourceList;

    // should be called in the user thread, not the rendering thread.
    explicit Resource(ResourceList* owningList)
        : owningList_(owningList)
    {
        owningList_->resources_.insert(this);
    }

public:
    virtual ~Resource() {}

    Resource(const Resource&) = delete;
    Resource& operator=(const Resource&) = delete;

protected:
    friend detail::ResourceList;
    friend class Engine;

    template<typename T>
    friend class ResourcePtr;

    // Called in the rendering thread.
    // Override this to release the actual resource.
    //
    virtual void release_(Engine*) = 0;

    // Called in the user thread (by decRef_).
    // Override this to reset all inner ResourcePtr.
    // This is required because decRef_ is not thread safe, it keeps other
    // things simpler.
    virtual void clearSubResources_() {}

private:
    void initRef_() {
        int64_t noCount = core::Int64Max;
        if (refCount_ == noCount) {
            refCount_ = 1;
        }
        else {
            throw core::LogicError("ResourcePtr: reference count already initialized.");
        }
    }

    void incRef_() {
        int64_t newCount = ++refCount_;
#ifdef VGC_DEBUG
        if (newCount <= 1) {
            throw core::LogicError("ResourcePtr: trying to take shared ownership of an already garbaged resource.");
        }
#endif
    }

    void decRef_() {
        if (--refCount_ == 0) {
            if (owningList_) {
                owningList_->resources_.erase(this);
                owningList_->danglingResources_.append(this);
                clearSubResources_();
            }
            else {
                delete this;
            }
        }
    }

    ResourceList* owningList_;
    Int64 refCount_ = core::Int64Max;
};

namespace detail {

inline ResourceList::~ResourceList() {
    if (!resources_.empty() || !danglingResources_.isEmpty()) {
        VGC_ERROR(LogVgcGraphics, "Some resources were not released yet.");
        for (Resource* res : resources_) {
            res->owningList_ = nullptr;
        }
        for (Resource* res : danglingResources_) {
            res->owningList_ = nullptr;
            delete res;
        }
    }
}

} // namespace detail

/// \class vgc::graphics::ResourcePtr<T>
/// \brief Shared pointer to a graphics `Resource`.
///
/// When the reference count reaches zero, the resource gets queued for release
/// and destruction in the rendering thread by the Engine that created it.
///
template<typename T>
class ResourcePtr {
public:
    template<typename U>
    friend class ResourcePtr;

    static_assert(std::is_base_of_v<Resource, T>);

    constexpr ResourcePtr() noexcept
        : p_(nullptr) {}

    constexpr ResourcePtr(std::nullptr_t) noexcept
        : p_(nullptr) {}

    explicit ResourcePtr(T* p)
        : p_(p)
    {
        if (p_) {
            p_->initRef_();
        }
    }

    ~ResourcePtr() {
        reset();
    }


    template<typename U>
    ResourcePtr(const ResourcePtr<U>& other)
        : p_(other.p_)
    {
        if (p_) {
            p_->incRef_();
        }
    }

    template<typename U>
    ResourcePtr(ResourcePtr<U>&& other)
        : p_(other.p_)
    {
        other.p_ = nullptr;
    }

    ResourcePtr(const ResourcePtr& other)
        : p_(other.p_)
    {
        if (p_) {
            p_->incRef_();
        }
    }

    ResourcePtr(ResourcePtr&& other)
        : p_(other.p_)
    {
        other.p_ = nullptr;
    }

    template<typename U>
    ResourcePtr& operator=(const ResourcePtr<U>& other) {
        if (p_ == other.p_) {
            return *this;
        }
        if (other.p_) {
            other.p_->incRef_();
        }
        if (p_) {
            p_->decRef_();
        }
        p_ = other.p_;
        return *this;
    }

    template<typename U>
    ResourcePtr& operator=(ResourcePtr<U>&& other) {
        std::swap(p_, other.p_);
        return *this;
    }

    ResourcePtr& operator=(const ResourcePtr& other) {
        return operator=<T>(other);
    }

    ResourcePtr& operator=(ResourcePtr&& other) {
        std::swap(p_, other.p_);
        return *this;
    }

    void reset() {
        if (p_) {
            p_->decRef_();
#ifdef VGC_DEBUG
            p_ = nullptr;
#endif
        }
    }

    void reset(T* p) {
        if (p) {
            p->initRef_();
        }
        if (p_) {
            p_->decRef_();
        }
        p_ = p;
    }

    T* get() const {
        return p_;
    }

    Int64 useCount() const {
        return p_ ? p_->refCount_ : 0;
    }

    explicit operator bool() const noexcept {
        return p_ != nullptr;
    }

    T& operator*() const noexcept {
        return *p_;
    }

    T* operator->() const noexcept {
        return p_;
    }

private:
    T* p_ = nullptr;
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_RESOURCE_H
