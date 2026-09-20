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

#ifndef INTRINSIC_PROTO_TOOLS_COMPATIBILITY_PROTO_COMPAT_H_
#define INTRINSIC_PROTO_TOOLS_COMPATIBILITY_PROTO_COMPAT_H_

#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/status/statusor.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "google/protobuf/message.h"

namespace intrinsic {

// Result of IsMessageAssignable() below.
struct IsMessageAssignableResult {
  enum class DiffSeverityLevel {
    // Marks diffs that have been detected but are not problematic (if they have
    // been detected at all).
    kInfo,
    // Marks diffs that can cause problems (e.g., can cause information loss or
    // unexpected behavior) but won't cause proto deserialization to fail.
    kWarning,
    // Marks incompatible diffs which *can* cause proto deserialization to fail
    // or produce undefined results.
    kError,
  };

  // The type of a diff between the two versions of a message type. In several
  // cases we differentiate by field presence which refers to the presence of a
  // field in the optional message instance passed to IsMessageAssignable()
  // (i.e., 'message_b'). If no 'message_b' is given the field presence is
  // considered unknown.
  enum class DiffType {
    // The name of a used message has changed.
    kMessageNameChanged,

    // The name of a field has changed.
    kFieldNameChanged,              // field presence: unknown
    kFieldNameChangedValuePresent,  // field presence: present
    kFieldNameChangedValueAbsent,   // field presence: absent

    // The type of a field has changed.
    kFieldTypeChanged,              // field presence: unknown
    kFieldTypeChangedValuePresent,  // field presence: present
    kFieldTypeChangedValueAbsent,   // field presence: absent

    // The name and type of a field have changed.
    kFieldNameAndTypeChanged,              // field presence: unknown
    kFieldNameAndTypeChangedValuePresent,  // field presence: present
    kFieldNameAndTypeChangedValueAbsent,   // field presence: absent

    // The number of a field has changed.
    kFieldNumberChanged,              // field presence: unknown
    kFieldNumberChangedValuePresent,  // field presence: present
    kFieldNumberChangedValueAbsent,   // field presence: absent

    // A field has been deleted.
    kFieldDeleted,              // field presence: unknown
    kFieldDeletedValuePresent,  // field presence: present
    kFieldDeletedValueAbsent,   // field presence: absent

    // A field has been added. Note that the question of field presence does not
    // apply to added fields.
    kFieldAdded,
  };

  // A diff between the two given versions of a message type (e.g., a field
  // was added, removed or changed).
  struct Diff {
    DiffSeverityLevel severity;
    DiffType type;

    // A short summary of the diff.
    std::string summary;
  };

  // True if 'diffs' is empty or does not contain any diff with severity ==
  // 'kError'.
  bool is_assignable;
  std::vector<Diff> diffs;
};

std::ostream& operator<<(
    std::ostream& os,
    const IsMessageAssignableResult::DiffSeverityLevel& level);
std::ostream& operator<<(std::ostream& os,
                         const IsMessageAssignableResult::DiffType& type);
std::ostream& operator<<(std::ostream& os,
                         const IsMessageAssignableResult::Diff& diff);
std::ostream& operator<<(std::ostream& os,
                         const IsMessageAssignableResult& report);

// Compares a message type in two file descriptor sets A and B and returns
// whether a proto MB created with file descriptor set B can safely be assigned
// to a proto MA created with file descriptor set A. I.e., is it safe to assign
// "MA = MB" using binary serialization/deserialization
// ("MA.ParseFromString(MB.SerializeToString())")?
//
// 'message_full_name_a' and 'message_full_name_b' are the names of the
// message types to compare. These should typically be equal but are kept as
// separate parameters to allow the caller the convenience of not checking this
// explicitly. If they are not equal, the result will always be "unassignable".
//
// As another convenience, the caller can omit passing one or both file
// descriptor sets. In this case, the corresponding message type(s) is/are
// assumed to be empty (i.e., messages with no fields).
//
// Optionally, a concrete message instance MB can be provided as 'message_b'.
// Providing MB will generally reduce the number of cases which are considered
// "unassignable" and diffs will often be reported with a lower severity. E.g.,
// if the type of a field gets changed this is generally considered an
// incompatible change. The overall result will be "unassignable" and a diff
// with severity "Error" will be reported. However, if a concrete MB is provided
// and the field is not present in MB, then MB is considered "assignable" and
// the field type change will be reported as a diff with severity "Info".
absl::StatusOr<IsMessageAssignableResult> IsMessageAssignable(
    std::string_view message_full_name_a,
    const google::protobuf::FileDescriptorSet* absl_nullable fds_a,
    std::string_view message_full_name_b,
    const google::protobuf::FileDescriptorSet* absl_nullable fds_b,
    const google::protobuf::Message* absl_nullable message_b = nullptr);

// Result of IsMessageDeserializable() below.
struct IsMessageDeserializableResult {
  enum class IssueSeverityLevel {
    // Marks issues which can cause problems such as information loss that is
    // possibly harmless.
    kWarning,
    // Marks issues which either make deserialization impossible or which can
    // cause information loss that is - with high confidence - undesired and/or
    // harmful.
    kError,
  };

  enum class IssueType {
    // The binary represention of the message is invalid and cannot be
    // deserialized.
    kInvalidBinaryRepresentation,

    // The field for a deserialized value is not defined in the corresponding
    // message.
    kFieldMissing,

    // The field for a deserialized value in the corresponding message does not
    // have a type which is compatible with the wire-type of the value.
    kFieldTypeMismatch,

    // The type URL of the message is incorrectly formatted or refers to a
    // non-existing type.
    kInvalidTypeUrl,
  };

  struct Issue {
    IssueSeverityLevel severity;
    IssueType type;
    std::string summary;
  };

  // True if issues contains no error.
  bool is_deserializable;

  std::vector<Issue> issues;
};

std::ostream& operator<<(
    std::ostream& os,
    const IsMessageDeserializableResult::IssueSeverityLevel& level);
std::ostream& operator<<(std::ostream& os,
                         const IsMessageDeserializableResult::IssueType& type);
std::ostream& operator<<(std::ostream& os,
                         const IsMessageDeserializableResult::Issue& issue);
std::ostream& operator<<(std::ostream& os,
                         const IsMessageDeserializableResult& result);

// Checks whether a serialized proto (=the given Any) can be deserialized
// "without dangerous information loss" into a message using the given
// descriptor pool and associated message factory.
//
// This check is best effort - it does not require the descriptor pool which was
// used to serialize the original message and thus cannot detect all possible
// issues. It is solely based on the "unknown fields" which are generated by
// Protobuf during deserialization if a) the wire-type does not match the target
// field type or if b) the target field is not defined in the corresponding
// message descriptor, e.g., because the field was removed.
//
// A message is deemed deserializable "without dangerous information loss" if
//   1. deserialization succeeds in general and
//   2. there are no unknown fields due to type mismatches (case a)).
//
// For each unknown field this check returns an issue containing a best effort
// description of the problem (including, e.g., wire type, value, target field
// type/name/number) and a severity (case a) -> error, case b) -> warning).
//
// Note that if the given message is deemed deserializable it is not guaranteed
// that the message was serialized with a compatible message descriptor and that
// the deserialized message contains meaningful values. In particular, some type
// changes and other corner cases do not result in unknown fields to be
// generated and hence cannot be detected by this check. E.g., message and
// string fields have the same wire type: If the value "value: 'foo'" for a
// message field "StringValue value = 1;" gets serialized and then deserialized
// again into a field "string value = 1;", the deserialization will succeed just
// fine (no unknown field is generated) but the value of the deserialized string
// field will be "value: '\n\003foo'".
absl::StatusOr<IsMessageDeserializableResult> IsMessageDeserializable(
    const google::protobuf::Any& any,
    const google::protobuf::DescriptorPool& desc_pool,
    google::protobuf::MessageFactory& msg_factory);

// Variant of IsMessageDeserializable() above which takes a file descriptor set
// proto. Prefer the above overload if you already have a descriptor pool.
// Otherwise, this method creates a temporary pool from the given file
// descriptor set which can be expensive.
absl::StatusOr<IsMessageDeserializableResult> IsMessageDeserializable(
    const google::protobuf::Any& any,
    const google::protobuf::FileDescriptorSet& fds);

}  // namespace intrinsic

#endif  // INTRINSIC_PROTO_TOOLS_COMPATIBILITY_PROTO_COMPAT_H_
