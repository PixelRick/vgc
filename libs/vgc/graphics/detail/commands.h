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

#ifndef VGC_GRAPHICS_COMMANDS_H
#define VGC_GRAPHICS_COMMANDS_H

#include <any>

#include <vgc/core/array.h>
#include <vgc/core/arithmetic.h>
#include <vgc/geometry/mat4f.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/buffers.h>
#include <vgc/graphics/targets.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

} // namespace vgc::graphics

namespace vgc::graphics::detail {

// XXX use the polymorphism of lambdas instead of creating a structure for each command as below
class VGC_GRAPHICS_API Command2 {
    std::function<void(Engine*)> fn_;
    // XXX add name
};

// Abstract render command.
//
class VGC_GRAPHICS_API Command {
protected:
    Command() = default;

public:
    virtual ~Command() = default;

    virtual bool execute(Engine* engine) = 0;
};

class VGC_GRAPHICS_API WriteBufferCommand : public Command {
public:
    WriteBufferCommand(const BufferPtr& buffer, const float* data, Int length)
        : WriteBufferCommand(buffer, data, length, std::any(0)) {}

    WriteBufferCommand(BufferPtr buffer, const float* data, Int length, std::any&& dataOwner)
        : buffer_(buffer)
        , data_(data)
        , length_(length)
        , dataOwner_(std::move(dataOwner)) {}

    bool execute(Engine* engine) override;

private:
    BufferPtr buffer_;
    const float* data_;
    Int length_;
    std::any dataOwner_;
};

class VGC_GRAPHICS_API ClearCommand : public Command {
public:
    explicit ClearCommand(const core::Color& color)
        : color_(color) {}

    bool execute(Engine* engine) override;

private:
    core::Color color_;
};

class VGC_GRAPHICS_API PresentCommand : public Command {
public:
    PresentCommand(const SwapChainPtr& swapChain, UInt32 syncInterval)
        : swapChain_(swapChain)
        , syncInterval_(syncInterval) {}

    bool execute(Engine* engine) override;

private:
    SwapChainPtr swapChain_;
    UInt32 syncInterval_;
};

class VGC_GRAPHICS_API SetProjectionMatrixCommand : public Command {
public:
    explicit SetProjectionMatrixCommand(const geometry::Mat4f& m)
        : m_(m) {}

    bool execute(Engine* engine) override;

private:
    geometry::Mat4f m_;
};

class VGC_GRAPHICS_API SetViewMatrixCommand : public Command {
public:
    explicit SetViewMatrixCommand(const geometry::Mat4f& m)
        : m_(m) {}

    bool execute(Engine* engine) override;

private:
    geometry::Mat4f m_;
};


//class VGC_GRAPHICS_API DrawPrimitivesCommand : public Command {
//public:
//    DrawPrimitivesCommand() = default;
//    bool execute(Engine* engine) override;
//};

} // namespace vgc::graphics::detail

#endif // VGC_GRAPHICS_COMMANDS_H
