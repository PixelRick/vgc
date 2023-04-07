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

#include <vgc/topology/exceptions.h>

#include <vgc/core/format.h>

#include <vgc/topology/vac.h>

namespace vgc::topology {

VGC_CORE_EXCEPTIONS_DEFINE_ANCHOR(LogicError)
VGC_CORE_EXCEPTIONS_DEFINE_ANCHOR(RuntimeError)
VGC_CORE_EXCEPTIONS_DEFINE_ANCHOR(NotAChildError)

namespace detail {

std::string notAChildMsg(const VacNode* node, const VacNode* expectedParent) {
    return core::format(
        "VacNode {} (id: {}) is not a child of {} (id: {})",
        core::asAddress(node),
        node->id(),
        core::asAddress(expectedParent),
        expectedParent->id());
}

} // namespace detail

} // namespace vgc::topology
