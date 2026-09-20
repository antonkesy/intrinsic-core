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

// Some handy utilities around converting between native types and protobuf
// messages.
//
// Most importantly, this module establishes common patterns for exposing these
// conversions.
//
// For converting between ProtoType and NativeType, we define the following
// functions:
// absl::StatusOr<ProtoType> ToProto(const NativeType&);
// absl::StatusOr<NativeType> FromProto(const ProtoType&);
// These functions can be declared with the DECLARE_CONVERSIONS macro below.
// Ideally, these functions will be in the same namespace as either ProtoType
// or NativeType, so that argument-dependent lookup can be applied and the user
// doesn't have to qualify the calls.
//
// Since we have some existing code, under blue/shared/messages, which uses a
// different convention:
// bool ToProto(const NativeType&, ProtoType*);
// bool FromProto(const ProtoType&, NativeType*);
// We provide a convenient macro for wrapping these legacy functions with our
// own convention, namely DEFINE_CONVERSIONS_BLUE_CONVENTION.
// We consider the pattern presented here to be superior since:
// - More elaborate error messages with Status than with bool.
// - More natural flow at the call site when using return value for the output.
// - Support for immutable native types (the Blue convention assumes you can
//   construct an empty instance of the native type, then fill it up, which is
//   not always the case).

#ifndef THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_PROTO_CONVERSION_H_
#define THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_PROTO_CONVERSION_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/repeated_field.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace proto_conversion {

namespace details {

////////////////////////////////////////////////////////////////////////////////
// Declare (no definition) conversion functions between NativeType and
// ProtoType.

#define DECLARE_TO_PROTO(NativeType, ProtoType) \
  absl::StatusOr<ProtoType> ToProto(const NativeType&);

#define DECLARE_FROM_PROTO(NativeType, ProtoType) \
  absl::StatusOr<NativeType> FromProto(const ProtoType&);

#define DECLARE_CONVERSIONS(NativeType, ProtoType) \
  DECLARE_TO_PROTO(NativeType, ProtoType)          \
  DECLARE_FROM_PROTO(NativeType, ProtoType)

////////////////////////////////////////////////////////////////////////////////
// An adapter between the Blue-convention ToProto to our convention.
// Prefer using through the DEFINE_TO_PROTO_BLUE_CONVENTION macro below.
}  // namespace details

}  // namespace proto_conversion

////////////////////////////////////////////////////////////////////////////////
// Define templates for extracting the corresponding type. Depends on the
// conversion functions being in scope.

template <typename NativeType>
using ProtoTypeOf =
    ::std::decay_t<decltype(ToProto(std::declval<NativeType>()).value())>;

template <typename ProtoType>
using NativeTypeOf =
    ::std::decay_t<decltype(FromProto(std::declval<ProtoType>()).value())>;

////////////////////////////////////////////////////////////////////////////////
// These templates, for any type pair NativeType and ProtoType, enables
// conversions between RepeatedPtrField<ProtoType> and vector<NativeType>.
// They rely on ADL to deduce the relation of ProtoType to NativeType and vice
// versa by calling the ToProto and FromProto function unqualified.

template <typename NativeType>
std::enable_if_t<!std::is_arithmetic_v<NativeType>,
                 absl::StatusOr<::google::protobuf::RepeatedPtrField<
                     ::intrinsic::ProtoTypeOf<NativeType>>>>
ToProto(const ::std::vector<NativeType>& vec) {
  ::google::protobuf::RepeatedPtrField<::intrinsic::ProtoTypeOf<NativeType>>
      result;
  for (const auto& n : vec) {
    INTR_ASSIGN_OR_RETURN(*result.Add(), ToProto(n));
  }
  return result;
}

template <typename ProtoType>
absl::StatusOr<std::vector<::intrinsic::NativeTypeOf<ProtoType>>> FromProto(
    const ::google::protobuf::RepeatedPtrField<ProtoType>& rep) {
  ::std::vector<::intrinsic::NativeTypeOf<ProtoType>> result;
  result.reserve(rep.size());
  for (const auto& p : rep) {
    INTR_ASSIGN_OR_RETURN(auto n, FromProto(p));
    result.push_back(::std::move(n));
  }
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// These templates enable conversions between RepeatedField<T> and vector<T>.

template <typename BuiltInType>
std::enable_if_t<std::is_arithmetic_v<BuiltInType>,
                 absl::StatusOr<::google::protobuf::RepeatedField<BuiltInType>>>
ToProto(const ::std::vector<BuiltInType>& vec) {
  ::google::protobuf::RepeatedField<BuiltInType> result;
  for (const auto& n : vec) {
    *result.Add() = n;
  }
  return result;
}

template <typename BuiltInType>
absl::StatusOr<std::vector<BuiltInType>> FromProto(
    const ::google::protobuf::RepeatedField<BuiltInType>& rep) {
  std::vector<BuiltInType> result(rep.begin(), rep.end());
  return result;
}

}  // namespace intrinsic

#endif  // THIRD_PARTY_INTRINSIC_UTILS_PROTO_CONVERSION_PROTO_CONVERSION_H_
