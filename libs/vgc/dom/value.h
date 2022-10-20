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

#ifndef VGC_DOM_VALUE_H
#define VGC_DOM_VALUE_H

#include <memory>
#include <string>
#include <variant>

#include <vgc/core/array.h>
#include <vgc/core/color.h>
#include <vgc/core/enum.h>
#include <vgc/dom/api.h>
#include <vgc/geometry/vec2d.h>

namespace vgc::dom {

/// \enum vgc::dom::ValueType
/// \brief Specifies the type of an attribute Value
///
/// Although W3C DOM attribute values are always strings, VGC DOM attribute
/// values are typed, and this class enumerates all allowed types. The type of
/// a given attribute Value can be retrieved at runtime via Value::type().
///
/// In order to write out compliant XML files, we define conversion rules
/// from/to strings in order to correcty encode and decode the types of
/// attributes.
///
/// Typically, in the XML file, types are not explicitly specified, since
/// built-in element types (e.g., "path") have built-in attributes (e.g.,
/// "positions") with known built-in types (in this case, "Vec2dArray").
/// Therefore, the VGC parser already knows what types all of these
/// built-in attributes have.
///
/// However, it is also allowed to add custom data to any element, and the type
/// of this custom data must be encoded in the XML file by following a specific
/// naming scheme. More specifically, the name of custom attributes must be
/// data-<t>-<name>, where <name> is a valid XML attribute name chosen by the
/// user (which cannot be equal to any of the builtin attribute names of this
/// element), and <t> is a one-letter code we call "type-hint". This type-hint
/// allows the parser to unambiguously deduce the type of the attribute from
/// the XML string value. The allowed type-hints are:
///
/// - c: color
/// - d: 64bit floating point
/// - i: 32bit integer
/// - s: string
///
/// For example, if you wish to store a custom sequence of indices (that is,
/// some data of type IntArray) into a given path, you can do as follows:
///
/// \code
/// <path
///     data-i-myIndices="[12, 42, 10]"
/// />
/// \endcode
///
/// If you wish to store a custom sequence of 2D coordinates (Vec2dArray):
///
/// \code
/// <path
///     data-d-myCoords="[(0, 0), (12, 42), (10, 34)]"
/// />
/// \endcode
///
/// Note how in the example above, the type "Vec2dArray" is automatically
/// deduced from the type-hint 'd' and the XML string value. However, without
/// the type-hint, the parser wouldn't be able to determine whether the
/// attribute has the type Vec2dArray or Vec2iArray.
///
/// XXX What if it's an empty array? data-d-myData="[]"? How to determine the
/// type from the type-hint? -> maybe we should forget about this "type-hint"
/// idea altogether, and just have authors always give the full type:
/// data-Vec2dArray-myCoords="[]"
///
enum class ValueType {
    // XXX TODO: complete the list of types
    None,
    Invalid,
    Int,
    String,
    Color,
    DoubleArray,
    Vec2dArray,
};

VGC_DOM_API
VGC_DECLARE_ENUM(ValueType)

namespace detail {

/*

ValueRef is the type-erased pointer to a type-erased Value, how can it copy into Value ?
Neither std::variant nor std::any can really help here right ?

Let's use a custom virtual table, and std::variant until we have all operators in it.

*/

// Derive variant types from single value types.
// e.g.: int, double -> int, array<int>, shared_ptr<array<int>, double, array<double>..

using SingleValueTypes = std::tuple<std::string, Int, double, core::Color, geometry::Vec2d>;

template<typename TmpTuple, typename... ValueTypes>
struct ValueVariantTypesTuple_;

template<typename... Processed, typename T, typename... Rest>
struct ValueVariantTypesTuple_<std::tuple<Processed...>, T, Rest...> {
    using typesTuple = ValueVariantTypesTuple_<
        std::tuple<Processed..., T, core::Array<T>/*, std::shared_ptr<core::Array<T>>*/>,
        Rest...>::typesTuple;
};

template<typename... Processed>
struct ValueVariantTypesTuple_<std::tuple<Processed...>> {
    using typesTuple = std::tuple<Processed...>;
};

template<typename VariantTypesTuple>
struct ValueVariantType2_;

template<typename... VariantTypes>
struct ValueVariantType2_<std::tuple<VariantTypes...>> {
    using type = std::variant<std::monostate, VariantTypes...>;
};

template<typename ValueTypesTuple>
struct ValueVariantType_;

template<typename... ValueTypes>
struct ValueVariantType_<std::tuple<ValueTypes...>> {
    using type = ValueVariantType2_<ValueVariantTypesTuple_<std::tuple<>, ValueTypes...>::typesTuple>::type;
};

using ValueVariantType = ValueVariantType_<SingleValueTypes>;

// Define our custom vtbl structure.

class ConstValueRef;

//template<typename T>
//struct ValueTypeFunctions {
//    static void copyDataToValue(void* ptr, Value& to) {
//        to.value_ = static_cast<T*>(ptr);
//    }
//
//    static ConstValueRef getItem(void* ptr, Int index) {
//        return {};
//    }
//};

struct ValueVtbl {
    ValueType publicType;
    std::type_index realType;
    std::type_index itemType;
    bool isArray;
    void (* fnCopyDataToValue)(void* ptr, Value& to);
    void* (* fnGetItem)(void* ptr, Int index);
    void (* fnShrinkToFit)(void* ptr);
};

template<typename T, ValueType valueType>
struct ValueVtblSingleton {
    ValueVtbl* get() {
        static ValueVtbl vtbl = {
            valueType,
            std::type_index(typeid(T)),
            std::type_index(typeid(void)),
            false,
            [](void* ptr, Value& to){},
            [](void* ptr, Int index, std::type_index getType){ return nullptr; },
            [](void* ptr){} };
        }
    }
};

#define VGC_DEFINE_VALUE_FUNCTIONS_FOR_SINGLE_TYPE(Type, Value)

/*
getters:
value.get<Color>() -> checks type is color
value.get<Color>(12) -> checks type is part of color array types 
*/

} // namespace detail

/// Writes the given ValueType to the output stream.
///
template<typename OStream>
void write(OStream& out, ValueType v) {
    core::formatTo(out, "{}", v);
}

/// \class vgc::dom::ValueRef
/// \brief References the value or an indexed value of an attribute.
///
class VGC_DOM_API ValueRef {
private:
    friend Value;

public:
    ValueRef(const ValueRef&) = delete;
    ValueRef& operator=(const ValueRef&) = delete;

    /// Returns the ValueType of the referenced value.
    ///
    ValueType type() const {
        return valueVtbl_ ? valueVtbl_->publicType : ValueType::None;
    }

    /// Returns whether this Value is Valid, that is, whether type() is not
    /// ValueType::Invalid, which means that it does hold one of the correct
    /// values.
    ///
    bool isValid() const {
        return type() != ValueType::Invalid;
    }

    /// Reclaims unused memory in container values.
    ///
    void shrinkToFit() {
        if (valueVtbl_ && ptr_) {
            valueVtbl_->fnShrinkToFit(ptr_);
        }
    }

    template<typename T>
    T& get() {
        if (valueVtbl_ && valueVtbl_->realType == std::type_index(typeid(T)) {
            return *static_cast<T*>(ptr_);
        }
    }

    // must check if item is not const..
    template<typename T>
    T& getItem(Int index) {
        if (valueVtbl_ && valueVtbl_->itemType == std::type_index(typeid(T)) {
            return *static_cast<T*>(valueVtbl_->fnGetItem(index));
        }
    }

    template<typename T>
    const T& get() const {
        if (valueVtbl_ && valueVtbl_->realType == std::type_index(typeid(T)) {
            return *static_cast<T*>(ptr_);
        }
    }

    template<typename T>
    const T& getItem(Int index) const {
        if (valueVtbl_ && valueVtbl_->itemType == std::type_index(typeid(T)) {
            return *static_cast<T*>(valueVtbl_->fnGetItem(index));
        }
    }

    template<typename T>
    void set(const T&) {
        // 
    }

    template<typename T>
    void set(T&&) {
        //
    }

private:
    detail::ValueVtbl* valueVtbl_;
    void* ptr_;
};

/// \class vgc::dom::Value
/// \brief Holds the value of an attribute
///
class VGC_DOM_API Value {
public:
    /// Constructs an empty value, that is, whose ValueType is None.
    ///
    constexpr Value()
        : type_(ValueType::None) {
    }

    /// Returns a const reference to an empty value. This is useful for error
    /// handling in methods that must return a Value by const reference.
    ///
    static const Value& none();

    /// Returns a const reference to an invalid value. This is useful for error
    /// handling in methods that must return a Value by const reference.
    ///
    static const Value& invalid();

    /// Constructs a Value holding a Color.
    ///
    Value(const core::Color& color)
        : type_(ValueType::Color)
        , var_(color) {
    }

    /// Constructs a Value holding an Int.
    ///
    Value(Int integer)
        : type_(ValueType::Int)
        , var_(integer) {
    }

    /// Constructs a Value holding a string.
    ///
    Value(const std::string& s)
        : type_(ValueType::String)
        , var_(s) {
    }

    /// Constructs a Value holding a string.
    ///
    Value(std::string&& s)
        : type_(ValueType::String)
        , var_(std::move(s)) {
    }

    /// Constructs a Value holding a DoubleArray.
    ///
    Value(const core::DoubleArray& doubleArray)
        : type_(ValueType::DoubleArray)
        , var_(std::make_shared<core::DoubleArray>(doubleArray)) {
    }

    /// Constructs a Value holding a DoubleArray.
    ///
    Value(core::DoubleArray&& doubleArray)
        : type_(ValueType::DoubleArray)
        , var_(std::make_shared<core::DoubleArray>(std::move(doubleArray))) {
    }

    /// Constructs a Value holding a Vec2dArray.
    ///
    Value(const geometry::Vec2dArray& vec2dArray)
        : type_(ValueType::Vec2dArray)
        , var_(std::make_shared<geometry::Vec2dArray>(vec2dArray)) {
    }

    /// Constructs a Value holding a Vec2dArray.
    ///
    Value(geometry::Vec2dArray&& vec2dArray)
        : type_(ValueType::Vec2dArray)
        , var_(std::make_shared<geometry::Vec2dArray>(std::move(vec2dArray))) {
    }

    /// Returns the ValueType of this Value.
    ///
    ValueType type() const {
        return type_;
    }

    /// Returns whether this Value is Valid, that is, whether type() is not
    /// ValueType::Invalid, which means that it does hold one of the correct
    /// values.
    ///
    bool isValid() const {
        return type() != ValueType::Invalid;
    }

    /// Stops holding any Value. This makes this Value empty.
    ///
    void clear();

    /// Reclaims unused memory.
    ///
    void shrinkToFit();

    /// Returns the `core::Color` held by this `Value`.
    /// The behavior is undefined if `type() != ValueType::Color`.
    ///
    core::Color getColor() const {
        return std::get<core::Color>(var_);
    }

    /// Copies the `core::Color` held by this `Value` to `reference`.
    /// The behavior is undefined if type() != ValueType::Color.
    ///
    void get(core::Color& reference) const {
        reference = std::get<core::Color>(var_);
    }

    /// Sets this `Value` to the given `color`.
    ///
    void set(const core::Color& color) {
        type_ = ValueType::Color;
        var_ = color;
    }

    /// Returns the integer held by this `Value`.
    /// The behavior is undefined if `type() != ValueType::Int`.
    ///
    Int getInt() const {
        return std::get<Int>(var_);
    }

    /// Copies the integer held by this `Value` to `ref`.
    /// The behavior is undefined if `type() != ValueType::Int`.
    ///
    void get(Int& ref) const {
        ref = std::get<Int>(var_);
    }

    /// Sets this `Value` to the given integer `value`.
    ///
    void set(Int value) {
        type_ = ValueType::Int;
        var_ = value;
    }

    /// Returns the string held by this `Value`.
    /// The behavior is undefined if `type() != ValueType::String`.
    ///
    const std::string& getString() const {
        return std::get<std::string>(var_);
    }

    /// Copies the string held by this `Value` to `ref`.
    /// The behavior is undefined if `type() != ValueType::String`.
    ///
    void get(std::string& reference) const {
        reference = std::get<std::string>(var_);
    }

    /// Sets this `Value` to the given string `s`.
    ///
    void set(std::string s) {
        type_ = ValueType::String;
        var_ = std::move(s);
    }

    /// Returns the `Vec2dArray` held by this `Value`.
    /// The behavior is undefined if `type() != ValueType::Vec2dArray`.
    ///
    const geometry::Vec2dArray& getVec2dArray() const {
        return *std::get<std::shared_ptr<geometry::Vec2dArray>>(var_);
    }

    /// Copies the `Vec2dArray` held by this `Value` to `reference`.
    /// The behavior is undefined if `type() != ValueType::Vec2dArray`.
    ///
    void get(geometry::Vec2dArray& reference) const {
        reference = getVec2dArray();
    }

    /// Sets this `Value` to the given `vec2dArray`.
    ///
    void set(geometry::Vec2dArray vec2dArray) {
        type_ = ValueType::Vec2dArray;
        var_ = std::make_shared<geometry::Vec2dArray>(std::move(vec2dArray));
    }

    /// Returns the `DoubleArray` held by this `Value`.
    /// The behavior is undefined if `type() != ValueType::DoubleArray`.
    ///
    const core::DoubleArray& getDoubleArray() const {
        return *std::get<std::shared_ptr<core::DoubleArray>>(var_);
    }

    /// Copies the `DoubleArray` held by this `Value` to `reference`.
    /// The behavior is undefined if `type() != ValueType::DoubleArray`.
    ///
    void get(core::DoubleArray& reference) const {
        reference = getDoubleArray();
    }

    /// Sets this `Value` to the given `doubleArray`.
    ///
    void set(core::DoubleArray doubleArray) {
        type_ = ValueType::DoubleArray;
        var_ = std::make_shared<core::DoubleArray>(std::move(doubleArray));
    }

private:
    /// For the different valueless ValueType.
    ///
    explicit constexpr Value(ValueType type)
        : type_(type)
        , var_() {
    }

    ValueType type_ = ValueType::Invalid;
    std::variant<
        std::monostate,
        core::Color,
        Int,
        std::string,
        std::shared_ptr<core::DoubleArray>,
        std::shared_ptr<geometry::Vec2dArray>>
        var_;
};

/// Writes the given Value to the output stream.
///
template<typename OStream>
void write(OStream& out, const Value& v) {
    switch (v.type()) {
    case ValueType::None:
        write(out, "None");
        break;
    case ValueType::Invalid:
        write(out, "Invalid");
        break;
    case ValueType::Color:
        write(out, v.getColor());
        break;
    case ValueType::DoubleArray:
        write(out, v.getDoubleArray());
        break;
    case ValueType::Vec2dArray:
        write(out, v.getVec2dArray());
        break;
    }
}

/// Converts the given string into a Value. Raises vgc::dom::VgcSyntaxError if
/// the given string does not represent a Value of the given ValueType.
///
VGC_DOM_API
Value parseValue(const std::string& s, ValueType t);

} // namespace vgc::dom

#endif // VGC_DOM_VALUE_H
