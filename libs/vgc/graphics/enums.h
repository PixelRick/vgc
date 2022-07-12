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

#include <vgc/core/arithmetic.h>
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
    Image = 0x20,
    DepthStencil = 0x40,
    UnorderedAccess = 0x80,
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(BindFlags)

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_resource_misc_flag
//
enum class ResourceMiscFlags : UInt32 {
    None = 0,
    GenerateMips = 1,
    Shared = 2,
    TextureCube = 4,
    DrawIndirectArgs = 0x10,
    BufferRaw = 0x20,
    BufferStructured = 0x40,
    ResourceClamp = 0x80,
    SharedKeyedMutex = 0x100,
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(ResourceMiscFlags)

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_cpu_access_flag
//
enum class CpuAccessFlags : UInt8 {
    None = 0,
    Read = 1,
    Write = 2,
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(CpuAccessFlags)

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_usage
//
enum class Usage : UInt8 {
    Default,
    Immutable,
    Dynamic,
    Staging,
};

// See https://docs.microsoft.com/en-us/windows/win32/api/d3d11/ne-d3d11-d3d11_cpu_access_flag
//
enum class Mapping : UInt8 {
    None,
    Read,
    Write,
    ReadWrite,
    WriteDiscard,
    WriteNoOverwrite,
};

enum class PrimitiveType : UInt8 {
    Point,
    LineList,
    LineStrip,
    TriangleList,
    TriangleStrip,
};

enum class SwapChainTargetFormat : UInt8 {
    RGBA_8_UNORM,
    BGRA_8_SRGB,
};

enum class PresentFlags : UInt32 {
    None,
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(PresentFlags)

enum class ImageRank : UInt8 {
    _1D,
    _2D,
    // XXX future
    //_3D,
    //_CubeMap, // OpenGL doesn't support cubemap textures from 2d image array.
};

enum class ImageFormat : UInt8 {
    None,
    // Depth
    D_16_UNORM,
    D_32_FLOAT,
    // Depth + Stencil
    DS_24_UNORM_8_UINT,
    DS_32_FLOAT_8_UINT_24_X,
    // Red
    R_8_UNORM,
    R_8_SNORM,
    R_8_UINT,
    R_8_SINT,
    R_16_UNORM,
    R_16_SNORM,
    R_16_UINT,
    R_16_SINT,
    R_16_FLOAT,
    R_32_UINT,
    R_32_SINT,
    R_32_FLOAT,
    // RG
    RG_8_UNORM,
    RG_8_SNORM,
    RG_8_UINT,
    RG_8_SINT,
    RG_16_UNORM,
    RG_16_SNORM,
    RG_16_UINT,
    RG_16_SINT,
    RG_16_FLOAT,
    RG_32_UINT,
    RG_32_SINT,
    RG_32_FLOAT,
    // RGB
    RGB_11_11_10_FLOAT,
    RGB_32_UINT,
    RGB_32_SINT,
    RGB_32_FLOAT,
    // RGBA
    RGBA_8_UNORM,
    RGBA_8_UNORM_SRGB,
    RGBA_8_SNORM,
    RGBA_8_UINT,
    RGBA_8_SINT,
    RGBA_10_10_10_2_UNORM,
    RGBA_10_10_10_2_UINT,
    RGBA_16_UNORM,
    RGBA_16_UINT,
    RGBA_16_SINT,
    RGBA_16_FLOAT,
    RGBA_32_UINT,
    RGBA_32_SINT,
    RGBA_32_FLOAT,
    // XXX future
    // Compressed
    //BC1_UNORM,
    //BC1_UNORM_SRGB,
    //BC2_UNORM,
    //BC2_UNORM_SRGB,
    //BC3_UNORM,
    //BC3_UNORM_SRGB,
    //BC4_UNORM,
    //BC4_SNORM,
    //BC5_UNORM,
    //BC5_SNORM,
    //BC7_UNORM,
    //BC7_UNORM_SRGB,
};

enum class BlendFactor : UInt8 {
    One,
    Zero,
    SourceColor,
    OneMinusSourceColor,
    SourceAlpha,
    OneMinusSourceAlpha,
    TargetColor,
    OneMinusTargetColor,
    TargetAlpha,
    OneMinusTargetAlpha,
    SourceAlphaSaturated,
    Constant,
    OneMinusConstant,
    SecondSourceColor,
    OneMinusSecondSourceColor,
    SecondSourceAlpha,
    OneMinusSecondSourceAlpha,
};

enum class BlendOp : UInt8 {
    Add,
    SourceMinusTarget,
    TargetMinusSource,
    Min,
    Max,
};

enum class BlendWriteMask : UInt8 {
    None = 0,
    R = 1,
    G = 2,
    B = 4,
    A = 8,
    All = R | G | B | A,
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(BlendWriteMask)

enum class FillMode : UInt8 {
    Solid,
    Wireframe,
};

enum class CullMode : UInt8 {
    None,
    Front,
    Back,
};

enum class ShaderType : UInt8 {
    Vertex,
    //Hull,
    //Domain,
    Geometry,
    Pixel,
    //Compute,
    Max_ = Pixel
};

enum class BuiltinProgram : UInt8 {
    Simple,
    // XXX publicize ?
    //GlyphAtlas,
    //IconsAtlas,
    //RoundedRectangle,
};

enum class BuiltinGeometryLayout : UInt8 {
    None,
    XY,
    XYRGB,
    XYZ,
};

enum class PipelineParameters : UInt32 {
    None = 0,
    Viewport            = 0x00000001,
    Program             = 0x00000002,
    BlendState          = 0x00000004,
    DepthStencilState   = 0x00000008,
    RasterizerState     = 0x00000010,
    ConstantBuffersVS   = 0x00001000,
    ConstantBuffersGS   = 0x00002000,
    ConstantBuffersPS   = 0x00004000,
    ImageViewsVS        = 0x00010000,
    ImageViewsGS        = 0x00020000,
    ImageViewsPS        = 0x00040000,
    SamplerStatesVS     = 0x00100000,
    SamplerStatesGS     = 0x00200000,
    SamplerStatesPS     = 0x00400000,

    BoundConstantBuffers = VSConstantBuffers | GSConstantBuffers | PSConstantBuffers,
    BoundImageViews = VSImageViews | GSImageViews | PSImageViews,
    SamplerStates = VSSamplerStates | GSSamplerStates | PSSamplerStates,
    Views = BoundConstantBuffers | BoundImageViews,
    All = Viewport | Program | BlendState | DepthStencilState | Views | SamplerStates,
};
VGC_DEFINE_SCOPED_ENUM_FLAGS_OPERATORS(PipelineParameters)

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_ENUMS_H
