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

#ifndef VGC_GRAPHICS_DETAIL_COMPTR_H
#define VGC_GRAPHICS_DETAIL_COMPTR_H

namespace vgc::graphics {

template<typename T>
class ComPtr {
public:
    ComPtr() noexcept = default;
    ComPtr(T* p) : p_(p) {
        if (p_) {
            p_->AddRef();
        }
    }

    ~ComPtr() {
        if (p_) {
            p_->Release();
        }
    }

    ComPtr(const ComPtr&) = delete;
    ComPtr& operator=(const ComPtr&) = delete;

    ComPtr& operator=(T* p) {
        reset();
        p_ = p;
        if (p_) {
            p_->AddRef();
        }
    }

    T* get() const {
        return p_;
    }

    void reset() {
        if (p_) {
            p_->Release();
            p_ = nullptr;
        }
    }

    T** addressOf() {
        reset();
        return &p_;
    }

    explicit operator bool() const noexcept {
        return p_ != nullptr;
    }

    T& operator*() const noexcept {
        return *p_;
    }

    T* operator->() const noexcept {
        return p_;
    }

    T* const* operator&() {
        return &p_;
    }

private:
    T* p_ = nullptr;
};

} // namespace vgc::graphics

#endif // VGC_GRAPHICS_DETAIL_COMPTR_H
