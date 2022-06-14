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

#ifndef VGC_GRAPHICS_ENUMS_H
#define VGC_GRAPHICS_ENUMS_H

#include <vgc/core/enum.h>

namespace vgc::graphics {

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_bind_flag
//
enum class BindFlags : UInt16 {
    None = 0,
    VertexBuffer = 1,
    IndexBuffer = 2,
    UniformBuffer = 4,
    ShaderResource = 8,
    StreamOutput = 0x10,
    RenderTarget = 0x20,
    DepthStencil = 0x40,
    UnorderedAccess = 0x80,
    DecoderOutput = 0x200,
    DecoderInput = 0x400
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(BindFlags)

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_cpu_access_flag
//
enum class CpuAccessFlags : UInt8 {
    None = 0,
    Read = 1,
    Write = 2
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(CpuAccessFlags)

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_usage
//
enum class Usage : UInt8 {
    Default = 0,
    Immutable,
    Dynamic,
    Staging
};

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_cpu_access_flag
//
enum class Mapping : UInt8 {
    None = 0,
    Read,
    Write,
    ReadWrite,
    WriteDiscard,
    WriteNoOverwrite
};

enum class PrimitiveType : UInt8 {
    Point,
    LineList,
    LineStrip,
    TriangleList,
    TriangleStrip
};

enum class SwapChainTargetFormat : UInt8 {
    R32G32B32A32_FLOAT,
    R32G32B32A32_UINT,
    R32G32B32_FLOAT,
    R32G32B32_UINT
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_ENUMS_H
