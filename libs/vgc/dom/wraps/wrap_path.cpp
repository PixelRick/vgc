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

#include <vgc/core/wraps/common.h>

#include <vgc/core/arithmetic.h>
#include <vgc/core/stringid.h>
#include <vgc/dom/document.h>
#include <vgc/dom/element.h>
#include <vgc/dom/value.h>

using This = vgc::dom::Path;

void wrap_path(py::module& m) {
    py::class_<This>(m, "Path")
        /*
        Path(Element* element);
    Path(Element* element, core::StringId attributeName);
    Path(Element* element, core::StringId attributeName, Int arrayIndex);

    Path(std::string_view path);
        */
        .def(py::init<>())
        .def(py::init<vgc::dom::Element*>())
        .def(py::init<vgc::dom::Element*, vgc::core::StringId>())
        .def(py::init<vgc::dom::Element*, vgc::core::StringId, vgc::Int>())
        .def(py::init<std::string_view>())
        .def(
            "__str__",
            [](const This& self) -> std::string {
                fmt::memory_buffer mbuf;
                mbuf.push_back('@');
                mbuf.push_back('\'');
                mbuf.append(self.toString());
                mbuf.push_back('\'');
                return std::string(mbuf.begin(), mbuf.size());
            });
}
