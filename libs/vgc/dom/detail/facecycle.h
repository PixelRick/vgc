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

#ifndef VGC_DOM_DETAIL_FACECYCLE_H
#define VGC_DOM_DETAIL_FACECYCLE_H

#include <vgc/core/format.h>
#include <vgc/dom/document.h>
#include <vgc/dom/path.h>
#include <vgc/dom/value.h>

namespace vgc::dom::detail {

/*
virtual bool read(StreamReader& in) = 0;
virtual bool write(StreamWriter& out) const = 0;

virtual FormatterBufferIterator
format(FormatterBufferCtx& ctx, std::string_view fmtString) const = 0;

*/

class CycleComponent {
public:
    CycleComponent() noexcept = default;

    CycleComponent(Path path, bool direction)
        : path_(std::move(path))
        , direction_(direction) {
    }

    const Path& path() const {
        return path_;
    }

    bool direction() const {
        return direction_;
    }

    void write(StreamWriter& out) const;
    void read(StreamReader& in);

    friend void write(StreamWriter& out, const CycleComponent& component) {
        component.write(out);
    }

    friend void readTo(CycleComponent& component, StreamReader& in) {
        component.read(in);
    }

private:
    Path path_ = {};
    bool direction_ = true;
};

class Cycle {
public:
    explicit Cycle(core::Array<CycleComponent> components)
        : components_(std::move(components)) {
    }

    const core::Array<CycleComponent>& components() const {
        return components_;
    }

    void write(StreamWriter& out) const;
    void read(StreamReader& in);

    friend void write(StreamWriter& out, const Cycle& cycle) {
        cycle.write(out);
    }

    friend void readTo(Cycle& cycle, StreamReader& in) {
        cycle.read(in);
    }

private:
    core::Array<CycleComponent> components_;
};

// todo: CustomValueArray<TCustomValue>

class FaceCycles final : public CustomValue {
public:
    FaceCycles() noexcept
        : CustomValue(true) {
    }

    const core::Array<Cycle>& cycles() const {
        return cycles_;
    }

protected:
    void preparePathsForUpdate_(const Element* owner) const override;
    void updatePaths_(const Element* owner, const PathUpdateData& data) override;

    std::unique_ptr<CustomValue> clone_() const override;

    bool compareEqual_(CustomValue* rhs) const override;
    bool compareLess_(CustomValue* rhs) const override;

    void read_(StreamReader& in) override;
    void write_(StreamWriter& out) const override;

    FormatterBufferIterator format_(FormatterBufferCtx& ctx) const override;

private:
    core::Array<Cycle> cycles_;

    // todo: keep original string from parse
};

} // namespace vgc::dom::detail

template<>
struct fmt::formatter<vgc::dom::detail::Cycle> : fmt::formatter<double> {
    auto format(const vgc::dom::detail::Cycle& v, fmt::buffer_context<char>& ctx)
        -> decltype(ctx.out()) {
        std::string s;
        vgc::core::StringWriter sr(s);
        v.write(sr);
        return vgc::core::formatTo(ctx.out(), "{}", s);
    }
};

#endif // VGC_DOM_DETAIL_FACECYCLE_H
