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

#ifndef VGC_CORE_ANIMTIME_H
#define VGC_CORE_ANIMTIME_H

#include <functional>

#include <vgc/core/api.h>

namespace vgc::core {

class VGC_CORE_API AnimTime {
public:
    constexpr AnimTime() noexcept = default;

    bool operator==(const AnimTime& other) const {
        return x_ == other.x_;
    }

    bool operator!=(const AnimTime& other) const {
        return !operator==(other);
    }

    bool operator<(const AnimTime& other) const {
        return x_ < other.x_;
    }

    size_t hash() const {
        return std::hash<double>()(x_);
    }

private:
    double x_ = 0.0;
};

} // namespace vgc::core

template<>
struct std::hash<vgc::core::AnimTime> {
    std::size_t operator()(const vgc::core::AnimTime& x) const noexcept {
        return x.hash();
    }
};

#endif // VGC_CORE_ANIMTIME_H
