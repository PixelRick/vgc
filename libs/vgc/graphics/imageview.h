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

#ifndef VGC_GRAPHICS_IMAGEVIEW_H
#define VGC_GRAPHICS_IMAGEVIEW_H

#include <vgc/core/arithmetic.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>

namespace vgc::graphics {

VGC_DECLARE_OBJECT(Engine);

/// \class vgc::graphics::ImageView
/// \brief Abstract view of an image buffer attachable to some stage of the graphics pipeline.
///
// Since a swap chain's render target view represents different buffers over
// time, a Vulkan implementation should probably cache a view for each
// back buffer.
//
// Concept mapping:
//  D3D11  -> Shader Resource View (SRV)
//  OpenGL -> Texture
//  Vulkan -> Image View
// Looks like all three support buffers as image.
//
class VGC_GRAPHICS_API ImageView : public Resource {
protected:
    ImageView(ResourceList* gcList)
        : Resource(gcList) {}

    using Resource::Resource;
};
using ImageViewPtr = ResourcePtr<ImageView>;

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_IMAGEVIEW_H
