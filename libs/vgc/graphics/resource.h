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

#include <unordered_set>

#include <vgc/core/array.h>
#include <vgc/core/arithmetic.h>
#include <vgc/core/object.h>
#include <vgc/graphics/api.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

class Resource;

namespace detail {

// Used to collect resources so that the owning engine can release them when it can.
//
class VGC_GRAPHICS_API ResourceList {
protected:
    friend Engine;
    friend Resource;

    ResourceList() = default;

    // XXX assert lists are empty to prevent leaks
    virtual ~ResourceList() = default;

    // XXX add thread safety for deferred contexts creating resources..

    std::unordered_set<Resource*> resources_; // user thread
    core::Array<Resource*> resourcesPendingForRelease_; // user thread
    core::Array<Resource*> resourcesToRelease_; // engine thread (rendering)
};

} // namespace detail

/// \class vgc::graphics::Resource
/// \brief Abstract graphics resource.
///
class VGC_GRAPHICS_API Resource {
protected:
    using ResourceList = detail::ResourceList;

    explicit Resource(ResourceList* owningList)
        : owningList_(owningList) {}

public:
    virtual ~Resource() {}

    Resource(const Resource&) = delete;
    Resource& operator=(const Resource&) = delete;

    bool isValid() const {
        return owningList_ != nullptr;
    }

private:
    friend class Engine;

    template<typename T>
    friend class ResourcePtr;

    // called by the engine and in its context
    virtual void release_(Engine*) = 0;

    void incRefOrInit_() {
        if (refCount_ == core::Int64Max) {
            owningList_->resources_.insert(this);
            refCount_ = 1;
        }
        else {
            incRef_();
        }
    }

    void incRef_() {
#ifdef VGC_DEBUG
        if (refCount_ < 1) {

            throw core::LogicError("ResourcePtr: trying to take shared ownership of an already garbaged resource!");
        }
#endif
        ++refCount_;
    }

    void decRef_() {
        if (--refCount_ == 0) {
            if (owningList_) {
                owningList_->resourcesPendingForRelease_.append(this);
                owningList_->resources_.erase(this);
            }
            else {
                delete this;
            }
        }
    }

    ResourceList* owningList_;
    Int64 refCount_ = 0;
};

/// \class vgc::graphics::ResourcePtr<T>
/// \brief Reference-counting pointer to a graphics `Resource`.
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
            p_->incRefOrInit_();
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

    template<typename U>
    ResourcePtr& operator=(const ResourcePtr<U>& other) {
        if (this == &other) {
            return *this;
        }
        if (p_) {
            p_->decRef_();
        }
        p_ = other.p_;
        if (p_) {
            p_->incRef_();
        }
        return *this;
    }

    template<typename U>
    ResourcePtr& operator=(ResourcePtr<U>&& other) {
        if (this == &other) {
            return *this;
        }
        if (p_) {
            p_->decRef_();
        }
        p_ = other.p_;
        other.p_ = nullptr;
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
