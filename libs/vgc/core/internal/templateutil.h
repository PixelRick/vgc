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

#ifndef VGC_CORE_INTERNAL_TEMPLATEUTIL_H
#define VGC_CORE_INTERNAL_TEMPLATEUTIL_H

#include <tuple>
#include <type_traits>

namespace vgc::core::internal {

// Used to inline sfinae-based tests on type ArgType.
// see VGC_CONSTEXPR_IS_ID_ADDRESSABLE_IN_CLASS_ for an example.
//
template<typename ArgType>
struct LambdaSfinae {
    static constexpr bool check(...) { return false; }
    template <class Lambda>
    static constexpr auto check(Lambda lambda)
        -> decltype(lambda(std::declval<ArgType>()), bool{}) {
        return true;
    }
};


// compile-time evaluates to true only if &cls::id is a valid expression.
#define VGC_CONSTEXPR_IS_ID_ADDRESSABLE_IN_CLASS_(cls, id)                    \
  ::vgc::core::internal::LambdaSfinae<cls*>::check(                           \
      [](auto* v)                                                             \
          -> std::void_t<decltype(&std::remove_pointer_t<decltype(v)>::id)> { \
      })

#define VGC_CONSTEXPR_IS_TYPE_DEFINED_IN_CLASS_(cls, tname)                    \
  ::vgc::core::internal::LambdaSfinae<cls*>::check(                            \
      [](auto* v)                                                              \
          -> std::void_t<typename std::remove_pointer_t<decltype(v)>::tname> { \
      })


template<std::size_t I, typename... T, std::size_t... Is>
constexpr std::tuple<std::tuple_element_t<I + Is, std::tuple<T...>>...>
SubPackAsTuple_(std::index_sequence<Is...>);

template<size_t I, size_t N, typename... T>
using SubPackAsTuple = decltype(SubPackAsTuple_<I, T...>(std::make_index_sequence<N>{}));

} // namespace vgc::core::internal

namespace vgc::core {

// Available in C++20: std::type_identity
// Used to establish non-deduced contexts in template argument deduction.
// e.g. throwLengthError's IntType is deduced from first argument only,
// then second argument must be convertible to it.
// todo: move it to some common header or adopt c++20.
template<typename U>
struct TypeIdentity_ {
    using type = U;
};
template<typename U>
using TypeIdentity = typename TypeIdentity_<U>::type;

} // namespace vgc::core

#endif // VGC_CORE_INTERNAL_TEMPLATEUTIL_H
