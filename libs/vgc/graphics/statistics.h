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

#ifndef VGC_GRAPHICS_STATISTICS_H
#define VGC_GRAPHICS_STATISTICS_H

#include <vgc/core/arithmetic.h>
#include <vgc/geometry/vec4f.h>
#include <vgc/graphics/api.h>
#include <vgc/graphics/enums.h>
#include <vgc/graphics/resource.h>

namespace vgc::graphics {

class FrameStatistics {
public:
    UInt64 frameBeginTimestamp() const {
        return frameBeginTimestamp_;
    }

    void setFrameBeginTimestamp(UInt64 frameBeginTimestamp) {
        frameBeginTimestamp_ = frameBeginTimestamp;
    }

    UInt64 frameEndTimestamp() const {
        return frameEndTimestamp_;
    }

    void setFrameEndTimestamp(UInt64 frameEndTimestamp) {
        frameEndTimestamp_ = frameEndTimestamp;
    }

    UInt64 translationStartTimestamp() const {
        return translationStartTimestamp_;
    }

    void setTranslationStartTimestamp(UInt64 translationStartTimestamp) {
        translationStartTimestamp_ = translationStartTimestamp;
    }

    UInt64 translationEndTimestamp() const {
        return translationEndTimestamp_;
    }

    void setTranslationEndTimestamp(UInt64 translationEndTimestamp) {
        translationEndTimestamp_ = translationEndTimestamp;
    }

    UInt64 presentCallTimestamp() const {
        return presentCallTimestamp_;
    }

    void setPresentCallTimestamp(UInt64 presentCallTimestamp) {
        presentCallTimestamp_ = presentCallTimestamp;
    }

    UInt64 presentedTimestamp() const {
        return presentedTimestamp_;
    }

    void setPresentedTimestamp(UInt64 presentedTimestamp) {
        presentedTimestamp_ = presentedTimestamp;
    }

private:
    UInt64 frameBeginTimestamp_ = 0;
    UInt64 frameEndTimestamp_ = 0;
    UInt64 translationStartTimestamp_ = 0;
    UInt64 translationEndTimestamp_ = 0;
    UInt64 presentCallTimestamp_ = 0;
    UInt64 presentedTimestamp_ = 0;
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_SAMPLERSTATE_H
