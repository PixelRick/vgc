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

#ifndef VGC_WORKSPACE_RENDERABLE_H
#define VGC_WORKSPACE_RENDERABLE_H

#include <vgc/dom/element.h>
#include <vgc/graphics/engine.h>
#include <vgc/topology/vac.h>
#include <vgc/workspace/api.h>

namespace vgc::workspace {

// transforms only available on groups, composites, text, but not cells (to keep vac computations reasonably fast).
// a vac needs all of the transforms from its root.

// We want our workspace to provide a way to visit the scene for rendering.
// This brings a few questions:
//     - geometry is different depending on time, where and when should we cache it ?
//     - layers can thus be different too depending on time, when and where should we cache the textures ?

// if a layer is constant we probably want to keep it for other frames.
// during edition we can keep the composition of contiguous sequences of elements with specific blends.

// a previewer would cache its end frames directly, but a re-render when changing a few items should be reasonably fast.

// we also want to be able to draw different times simultaneously, it means synchronized cache or should we copy ?

// we don't know at what speed geometry can be generated for a given frame, maybe we could cache based on perf (time / size) ?
// scripted values for instance may be worth caching.

// reusable buffers (all times or specific time but all renderers):
//    mesh vertices
//    mesh indices
//    parametrization
//    outline strip
//    color buffers
//    gradient params (gradient palette ?)
//    effect params (effect group ?)

class RenderObject;

// called "rendered element" in SVG spec
class VGC_WORKSPACE_API Renderable {
protected:
    virtual ~Renderable() = default;

public:
    virtual std::shared_ptr<RenderObject>
    createRenderObject(graphics::Engine* engine, core::AnimTime t) = 0;

private:
    dom::Element* domElement_;
    topology::VacNode* vacNode_;
};

class VGC_WORKSPACE_API RenderObject {

    // the base virtual interface is meant to draw everything with the same parameters.
    // for instance: everything in wireframe, everything with gradients disabled.

    // A custom renderer should be able to hide all texts for instance, or draw them
    // wireframe but the rest normally.

    // Is selection/highlight style part of the default rendering ?
};

} // namespace vgc::workspace

#endif // VGC_WORKSPACE_RENDERABLE_H
