// Copyright 2021 The VGC Developers
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

#ifndef VGC_GRAPHICS_ENGINE_H
#define VGC_GRAPHICS_ENGINE_H

#include <memory>
#include <list>
#include <unordered_set>

#include <vgc/core/assert.h>
#include <vgc/core/color.h>
#include <vgc/core/innercore.h>
#include <vgc/geometry/mat4f.h>
#include <vgc/graphics/api.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

class Resource;
class ResourcePtr;

class PrimitiveBuffer;
using PrimitiveBufferPtr = std::shared_ptr<PrimitiveBuffer>;

using SharedReadOnlyDataPtr = std::shared_ptr<const float>;

//class Command;
//using CommandUPtr = std::unique_ptr<Command>;
///// \class vgc::graphics::Command
///// \brief Abstract graphics command.
/////
//class VGC_GRAPHICS_API Command {
//public:
//    virtual ~Command() {}
//
//    virtual void execute(Engine* engine) = 0;
//};
//
//class VGC_GRAPHICS_API CommandList {
//public:
//    template<typename TCommand, typename... Args>
//    void emplace_back(Args&&... args) {
//        commands_.emplace_back(new TCommand(std::forward<Args>(args)...));
//    };
//
//    void execute(Engine* engine) {
//        for (const CommandUPtr& command : commands_) {
//            command->execute(engine);
//        }
//    }
//
//private:
//    std::list<CommandUPtr> commands_;
//};


/// \class vgc::graphics::Resource
/// \brief Abstract graphics resource.
///
class VGC_GRAPHICS_API Resource {
protected:
    explicit Resource(Engine* owner);

public:
    virtual ~Resource() {}

    Resource(const Resource&) = delete;
    Resource& operator=(const Resource&) = delete;

    Engine* owner() const {
        return owner_;
    }

private:
    friend Engine;
    friend ResourcePtr;

    // called by the engine and in its context
    virtual void release_() = 0;

    void incRefOrInit_();
    void incRef_();
    void decRef_();

    Engine* owner_;
    Int64 refCount_ = 0;
};

class VGC_GRAPHICS_API ResourcePtr {
public:
    constexpr ResourcePtr() noexcept
        : p_(nullptr) {}

    constexpr ResourcePtr(std::nullptr_t) noexcept
        : p_(nullptr) {}

    explicit ResourcePtr(Resource* p)
        : p_(p) {
        p_->incRefOrInit_();
    }

    ~ResourcePtr() {
        if (p_) {
            p_->decRef_();
#ifdef VGC_DEBUG
            p_ = nullptr;
#endif
        }
    }

    ResourcePtr(const ResourcePtr& other)
        : p_(other.p_) {
        p_->incRef_();
    }

    ResourcePtr(ResourcePtr&& other)
        : p_(other.p_) {
        other.p_ = nullptr;
    }

    ResourcePtr& operator=(const ResourcePtr& other) {
        if (this == &other) {
            return *this;
        }
        if (p_) {
            p_->decRef_();
        }
        p_ = other.p_;
        p_->incRef_();
    }

    ResourcePtr operator=(ResourcePtr&& other) {
        if (this == &other) {
            return *this;
        }
        if (p_) {
            p_->decRef_();
        }
        p_ = other.p_;
        other.p_ = nullptr;
    }

    Resource* get() const {
        return p_;
    }

    Int64 useCount() const {
        return p_ ? p_->refCount_ : 0;
    }

    explicit operator bool() const noexcept {
        return p_ != nullptr;
    }

    Resource& operator*() const noexcept {
        return *p_;
    }

    Resource* operator->() const noexcept {
        return p_;
    }

private:
    Resource* p_ = nullptr;
};


enum class PrimitiveType {
    Point,
    LineList,
    LineStrip,
    TriangleList,
    TriangleStrip,
};

/// \class vgc::graphics::PrimitivesBuffer
/// \brief Abstract primitives data buffer.
///
class VGC_GRAPHICS_API PrimitivesBuffer : public Resource {
public:
    PrimitivesBuffer(Engine* owner, PrimitiveType type)
        : Resource(owner), type_(type) {}

    /// Loads the given data. Unless this engine is a software renderer, this
    /// typically sends the data from RAM to the GPU.
    /// This operation is often expensive, and therefore the number of calls to
    /// this function should be minimized.
    /// The given data must be in the following format :
    ///
    /// ```
    /// [x1, y1, r1, g1, b1,     // First vertex
    ///  ...]
    /// ```
    ///
    /// To be called in graphics context.
    ///
    void load(const float* data, Int length) {
        load_(data, length);
    }

    /// Draws the given triangles.
    ///
    void draw(Engine* engine) {
        draw_(engine);
    }

    PrimitiveType type() const {
        return type_;
    }

private:
    virtual void load_(const float* data, Int length) = 0;
    virtual void draw_(Engine* engine) = 0;

    PrimitiveType type_;
};

/// \class vgc::graphics::Engine
/// \brief Abstract interface for graphics rendering.
///
/// This class is an abstract base class defining a common shared API for
/// graphics rendering. Implementations of this abstraction may provide
/// backends such as OpenGL, Vulkan, Direct3D, Metal, or software rendering.
///
/// The graphics engine is responsible for managing two matrix stacks: the
/// projection matrix stack, and the view matrix stack. When the engine is
/// constructed, each of these stacks is initialized with the identity matrix
/// as the only matrix in the stack. It is undefined behavior for clients to
/// call "pop" more times than "push": some implementations may emit an
/// exception (instantly or later), others may cause a crash (instantly or
/// later), or the drawing operations may fail.
///
class VGC_GRAPHICS_API Engine : public core::Object {
private:
    VGC_OBJECT(Engine, core::Object)
    VGC_PRIVATIZE_OBJECT_TREE_MUTATORS

protected:
    /// Constructs an Engine. This constructor is an implementation detail only
    /// available to derived classes.
    ///
    Engine();

public:
    // MAIN THREAD functions

    /// Creates a float buffer for storing primitives data, and returns a shared
    /// pointer to the new resource.
    ///
    /// Once created, you can load triangles data using
    /// resource->load(...), and you can draw the loaded triangles
    /// using resource->draw().
    ///
    /// When you don't need the buffer anymore (e.g., in the
    /// vgc::ui::Widget::cleanup() function), you must reset the resource pointer.
    ///
    // Note: in the future, we may add overloads to this function to allow
    // specifying the vertex format (i.e., XYRGB, XYZRGBA, etc.), but for now
    // the format is fixed (XYRGB).
    //
    virtual PrimitiveBufferPtr createPrimitivesBuffer(PrimitiveType type) = 0;

public:
    // RENDER THREAD functions

    /// Clears the whole render area with the given color.
    ///
    virtual void clear(const core::Color& color) = 0;

    /// Returns the current projection matrix.
    ///
    virtual geometry::Mat4f projectionMatrix() = 0;

    /// Sets the current projection matrix.
    ///
    virtual void setProjectionMatrix(const geometry::Mat4f& m) = 0;

    /// Adds a copy of the current projection matrix to the matrix stack.
    ///
    virtual void pushProjectionMatrix() = 0;

    /// Removes the current projection matrix from the matrix stack.
    ///
    /// The behavior is undefined if there is only one matrix in the stack
    /// before calling this function.
    ///
    virtual void popProjectionMatrix() = 0;

    /// Returns the current view matrix.
    ///
    virtual geometry::Mat4f viewMatrix() = 0;

    /// Sets the current view matrix.
    ///
    virtual void setViewMatrix(const geometry::Mat4f& m) = 0;

    /// Adds a copy of the current view matrix to the matrix stack.
    ///
    virtual void pushViewMatrix() = 0;

    /// Removes the current view matrix from the matrix stack.
    ///
    /// The behavior is undefined if there is only one matrix in the stack
    /// before calling this function.
    ///
    virtual void popViewMatrix() = 0;

private:
    friend Resource;

    std::unordered_set<Resource*> resources_;
    core::Array<Resource*> resourcesPendingForRelease_; // main thread
    core::Array<Resource*> resourcesToRelease_; // rendering thread
};

inline void Resource::incRefOrInit_() {
    if (refCount_ == core::Int64Max) {
        owner_->resources_.insert(this);
        refCount_ = 1;
    }
    else {
        incRef_();
    }
}

inline void Resource::incRef_() {
#ifdef VGC_DEBUG
    if (refCount_ < 1) {
        throw core::LogicError("ResourcePtr: trying to increment null reference count!");
    }
#endif
    ++refCount_;
}

inline void Resource::decRef_() {
    if (--refCount_ == 0) {
        owner_->resourcesPendingForRelease_.append(this);
        owner_->resources_.erase(this);
    }
}

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_ENGINE_H
