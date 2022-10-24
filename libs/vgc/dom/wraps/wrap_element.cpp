// Copyright 2021 The VGC Developers
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
#include <vgc/core/wraps/object.h>

#include <string>

#include <vgc/core/format.h>
#include <vgc/dom/document.h>
#include <vgc/dom/element.h>
#include <vgc/dom/strings.h>

using This = vgc::dom::Element;
using Parent = vgc::dom::Node;

using vgc::dom::Document;
using vgc::dom::Element;

void wrap_element(py::module& m) {
    vgc::core::wraps::ObjClass<This>(m, "Element")
        .def_create<This*, Document*, const std::string&>()
        .def_create<This*, Element*, const std::string&>()
        //.def(py::init([](Document* parent, const std::string& name) { return This::create(parent, name); } ))
        //.def(py::init([](Element* parent, const std::string& name) { return This::create(parent, name); } ))
        .def_property_readonly(
            "tagName", [](const This& self) { return self.tagName().string(); })
        .def_property("name", &Element::name, &Element::setName)
        .def_property_readonly(
            "id", [](const This& self) { return self.id().string(); })
        .def(
            "__str__",
            [](const This& self) -> std::string {
                fmt::memory_buffer mbuf;
                mbuf.push_back('<');
                mbuf.append(self.tagName().string());

                vgc::dom::Value v = self.getAuthoredAttribute(vgc::dom::strings::name);
                if (v.hasValue()) {
                    mbuf.append(std::string_view(" name=\""));
                    mbuf.append(v.getStringId().string());
                    mbuf.push_back('\"');
                }

                v = self.getAuthoredAttribute(vgc::dom::strings::id);
                if (v.hasValue()) {
                    mbuf.append(std::string_view(" id=\""));
                    mbuf.append(v.getStringId().string());
                    mbuf.push_back('\"');
                }

                for (const vgc::dom::AuthoredAttribute& attr : self.authoredAttributes()) {
                    const vgc::core::StringId name = attr.name();
                    if (name != vgc::dom::strings::name && name != vgc::dom::strings::id) {
                        mbuf.push_back(' ');
                        mbuf.append(name.string());
                        mbuf.append(std::string_view("=\""));
                        // XXX write to memory_buffer when possible
                        std::string val;
                        vgc::core::StringWriter sw(val);
                        write(sw, attr.value());
                        mbuf.append(val);
                        mbuf.push_back('\"');
                    }
                }

                mbuf.push_back('>');
                return std::string(mbuf.begin(), mbuf.size());
            })
        .def(
            "__repr__", [](const This& self) -> std::string {
                fmt::memory_buffer mbuf;
                mbuf.append(std::string_view("<vgc.dom.Element at "));
                mbuf.append(vgc::core::format("{}", vgc::core::asAddress(&self)));
                mbuf.append(std::string_view(" tagName=\""));
                mbuf.append(self.tagName().string());
                mbuf.push_back('\"');

                vgc::dom::Value v = self.getAuthoredAttribute(vgc::dom::strings::name);
                if (v.hasValue()) {
                    mbuf.append(std::string_view(" name=\""));
                    mbuf.append(v.getStringId().string());
                    mbuf.push_back('\"');
                }

                v = self.getAuthoredAttribute(vgc::dom::strings::id);
                if (v.hasValue()) {
                    mbuf.append(std::string_view(" id=\""));
                    mbuf.append(v.getStringId().string());
                    mbuf.push_back('\"');
                }

                for (const vgc::dom::AuthoredAttribute& attr : self.authoredAttributes()) {
                    const vgc::core::StringId name = attr.name();
                    if (name != vgc::dom::strings::name && name != vgc::dom::strings::id) {
                        mbuf.push_back(' ');
                        mbuf.append(name.string());
                        mbuf.append(std::string_view("=\""));
                        // XXX write to memory_buffer when possible
                        std::string val;
                        vgc::core::StringWriter sw(val);
                        write(sw, attr.value());
                        mbuf.append(val);
                        mbuf.push_back('\"');
                    }
                }

                mbuf.push_back('>');
                return std::string(mbuf.begin(), mbuf.size());
            });
}
