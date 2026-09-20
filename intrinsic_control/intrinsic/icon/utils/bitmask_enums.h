// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file allows enum clases and enums to be combined using bit operators:
// operator| operator& and operator~.  It also enables operator! for converting
// to a boolean.  This is enabled on selected enums by overloading the
// EnableBitmaskEnum trait function with a function returning std::true_type.
//
// Example:
//
//  // MyEnum - an enum class.  (Also works on regular non-class enums)
//  enum class MyEnum {
//    A = 1,
//    B = 2,
//    C = 4,
//    D = 8,
//  };
//
//  // Enable operators on MyEnum (define in same namespace as MyEnum).
//  std::true_type EnableBitmaskEnum(MyEnum);
//
//  // Use operators.  No casting!!
//  MyEnum foo(MyEnum a) {
//    MyEnum b = a | MyEnum::C;
//    MyEnum c = a & (MyEnum::A | MyEnum::B);
//    if (!c) {
//      ...
//    }
//    return b & ~c;
//  }
//
// If the enum is nested inside a class, the 'friend' attribute can be used
// to be able to declare the EnableBitmaskEnum function next to the enum:
//
// class Foo {
//  public:
//   enum class MyEnum { ... };
//   friend std::true_type EnableBitmaskEnum(MyEnum);
// };
//

#ifndef INTRINSIC_ICON_UTILS_BITMASK_ENUMS_H_
#define INTRINSIC_ICON_UTILS_BITMASK_ENUMS_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <type_traits>

namespace intrinsic {

inline constexpr uint16_t BitMask(uint8_t bit) { return 1 << bit; }

// A trait function for enabling bitmask operators on an enum.
template <typename Enum>
std::false_type EnableBitmaskEnum(Enum&&);

template <typename Enum>
constexpr bool EnableBitmaskEnumV =
    decltype(EnableBitmaskEnum(std::declval<Enum>()))::value;

// Returns the bitwise OR of the enum values, cast back to the enum type.
template <typename Enum, std::enable_if_t<EnableBitmaskEnumV<Enum>, int> = 0>
constexpr Enum operator|(Enum a, Enum b) {
  using itype = std::underlying_type_t<Enum>;
  return static_cast<Enum>(static_cast<itype>(a) | static_cast<itype>(b));
}

// Implement |= assignment.
template <typename Enum, std::enable_if_t<EnableBitmaskEnumV<Enum>, int> = 0>
constexpr Enum operator|=(Enum& a, Enum b) {
  return a = a | b;
}

// Returns the bitwise AND of the enum values, cast back to the enum type.
template <typename Enum, std::enable_if_t<EnableBitmaskEnumV<Enum>, int> = 0>
constexpr Enum operator&(Enum a, Enum b) {
  using itype = std::underlying_type_t<Enum>;
  return static_cast<Enum>(static_cast<itype>(a) & static_cast<itype>(b));
}

// Implement &= assignment.
template <typename Enum, std::enable_if_t<EnableBitmaskEnumV<Enum>, int> = 0>
constexpr Enum operator&=(Enum& a, Enum b) {
  return a = a & b;
}

// Returns the boolean NOT of the enum value.  True iff value is 0.
template <typename Enum, std::enable_if_t<EnableBitmaskEnumV<Enum>, int> = 0>
constexpr bool operator!(Enum a) {
  using itype = std::underlying_type_t<Enum>;
  return static_cast<itype>(a) == 0;
}

// Returns true if a given `bit` is set in the enum value `a`.
// This is equivalent to static_cast<bool>(a & bit).
template <typename Enum, std::enable_if_t<EnableBitmaskEnumV<Enum>, int> = 0>
constexpr bool HasBitSet(Enum a, Enum bit) {
  return static_cast<bool>(a & bit);
}

}  // namespace intrinsic

#endif  // INTRINSIC_ICON_UTILS_BITMASK_ENUMS_H_
