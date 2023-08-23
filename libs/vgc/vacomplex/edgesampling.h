// Copyright 2023 The VGC Developers
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

#ifndef VGC_VACOMPLEX_EDGESAMPLING_H
#define VGC_VACOMPLEX_EDGESAMPLING_H

#include <array>

#include <vgc/geometry/curve.h> // StrokeSample2d
#include <vgc/geometry/rect2d.h>
#include <vgc/geometry/vec2d.h>
#include <vgc/vacomplex/api.h>
#include <vgc/vacomplex/dataobject.h>

// how to share edge shape correctly ?
// an inbetween edge that doesn't change should have the same shape for all times
// we also need edge shape source/def, which can be different curve types
// -> EdgeParameters ?

namespace vgc::vacomplex {

class KeyEdge;

namespace detail {

class Operations;

// generic parameters for all models
class VGC_VACOMPLEX_API SamplingParameters {
public:
    SamplingParameters() noexcept = default;

    //size_t hash() {
    //    return ...;
    //}

private:
    /*
            UInt8 Lod_ = 0;
            Int16 maxSamples_ = core::tmax<Int16>;
            double maxAngularError_ = 7.0;
            double pixelSize_ = 1.0;
            geometry::Mat3d viewMatrix_ = geometry::Mat3d::identity;
            */
    // mode, uniform s, uniform u -> overload
};

} // namespace detail



} // namespace vgc::vacomplex

#endif // VGC_VACOMPLEX_EDGESAMPLING_H
