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

#include "intrinsic_runtime/intrinsic/proto_tools/compatibility/proto_compat.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/attributes.h"
#include "absl/base/nullability.h"
#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/escaping.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/strip.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "google/protobuf/descriptor_database.h"
#include "google/protobuf/dynamic_message.h"
#include "google/protobuf/message.h"
#include "google/protobuf/unknown_field_set.h"
#include "google/protobuf/util/message_differencer.h"
#include "intrinsic/util/proto/descriptor_pools.h"
#include "intrinsic/util/proto/descriptors.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace {

// Map from full descriptor name (package + name, without leading dot) to
// descriptor proto.
template <typename DescriptorT>
using DescriptorMap = absl::flat_hash_map<std::string, DescriptorT>;

struct DescriptorMaps {
  DescriptorMap<google::protobuf::DescriptorProto> message_types;
  DescriptorMap<google::protobuf::EnumDescriptorProto> enum_types;
  DescriptorMap<google::protobuf::ServiceDescriptorProto> services;
  DescriptorMap<google::protobuf::FieldDescriptorProto> extensions;

  // Returned pointer is not stable and must not be stored by the caller.
  absl::StatusOr<const google::protobuf::DescriptorProto*>
  FindMessageDescriptor(std::string_view message_full_name) const {
    if (auto it = message_types.find(message_full_name);
        it != message_types.end()) {
      return &it->second;
    }
    return absl::NotFoundError(absl::StrFormat(
        "Message %s not found in file descriptor set", message_full_name));
  }
};

// Pair of DescriptorMaps using the "A/B" naming scheme which is also used by
// the functions below (in contrast to std::pair which would use
// "first/second").
struct DescriptorMapsPair {
  DescriptorMaps a;
  DescriptorMaps b;
};

void AddNestedDescriptors(const google::protobuf::DescriptorProto& desc,
                          std::string_view path, DescriptorMaps& descriptors) {
  for (const google::protobuf::DescriptorProto& nested_desc :
       desc.nested_type()) {
    std::string full_name = absl::StrCat(path, ".", nested_desc.name());
    descriptors.message_types[full_name] = nested_desc;
    AddNestedDescriptors(nested_desc, full_name, descriptors);
  }

  for (const google::protobuf::EnumDescriptorProto& enum_desc :
       desc.enum_type()) {
    descriptors.enum_types[absl::StrCat(path, ".", enum_desc.name())] =
        enum_desc;
  }
}

DescriptorMaps CollectDescriptors(
    const google::protobuf::FileDescriptorSet& fds,
    const std::vector<std::string>& ignore_path_suffixes) {
  DescriptorMaps descriptors;
  for (const google::protobuf::FileDescriptorProto& file : fds.file()) {
    bool ignore_file = false;
    for (const std::string& ignore_suffix : ignore_path_suffixes) {
      if (absl::EndsWith(file.name(), ignore_suffix)) {
        ignore_file = true;
        break;
      }
    }
    if (ignore_file) {
      continue;
    }
    for (const google::protobuf::DescriptorProto& desc : file.message_type()) {
      std::string full_name = absl::StrCat(
          file.package(), file.package().empty() ? "" : ".", desc.name());
      descriptors.message_types[full_name] = desc;
      AddNestedDescriptors(desc, full_name, descriptors);
    }
    for (const google::protobuf::EnumDescriptorProto& desc : file.enum_type()) {
      std::string full_name = absl::StrCat(
          file.package(), file.package().empty() ? "" : ".", desc.name());
      descriptors.enum_types[full_name] = desc;
    }
    for (const google::protobuf::ServiceDescriptorProto& desc :
         file.service()) {
      std::string full_name = absl::StrCat(
          file.package(), file.package().empty() ? "" : ".", desc.name());
      descriptors.services[full_name] = desc;
    }
    for (const google::protobuf::FieldDescriptorProto& desc :
         file.extension()) {
      std::string full_name = absl::StrCat(
          file.package(), file.package().empty() ? "" : ".", desc.name());
      descriptors.extensions[full_name] = desc;
    }
  }
  return descriptors;
}

// Wrapper for the different parts of a message name.
struct MessageName {
  std::string package;    // e.g. "", "intrinsic_proto", "intrinsic_proto.math"
  std::string name;       // e.g. "Pose"
  std::string full_name;  // e.g. "intrinsic_proto.Pose", "MyGlobalMessage"

  explicit MessageName(std::string_view message_full_name)
      : full_name(message_full_name) {
    if (auto dot_pos = message_full_name.find_last_of('.');
        dot_pos != std::string::npos) {
      package = message_full_name.substr(0, dot_pos);
      name = message_full_name.substr(dot_pos + 1);
    } else {
      package = "";
      name = message_full_name;
    }
  }
};

absl::StatusOr<std::string> VerifyAndGetFieldTypeName(
    const google::protobuf::FieldDescriptorProto& field) {
  if (!field.type_name().starts_with(".")) {
    return absl::UnimplementedError(absl::StrFormat(
        "Encountered field descriptor with type_name = '%s'. Relative type "
        "names (without leading dot) are not supported.",
        field.type_name()));
  }

  return std::string(absl::StripPrefix(field.type_name(), "."));
}

// Converts a snake_case string to camelCase.
std::string ToJsonName(const std::string& input) {
  bool capitalize_next = false;
  std::string result;
  result.reserve(input.size());

  for (char character : input) {
    if (character == '_') {
      capitalize_next = true;
    } else if (capitalize_next) {
      result.push_back(absl::ascii_toupper(character));
      capitalize_next = false;
    } else {
      result.push_back(character);
    }
  }

  return result;
}

class IgnoreDefaultJsonNameDifference
    : public google::protobuf::util::MessageDifferencer::IgnoreCriteria {
 public:
  bool IsIgnored(const google::protobuf::Message& message1,
                 const google::protobuf::Message& message2,
                 const google::protobuf::FieldDescriptor* field,
                 const std::vector<
                     google::protobuf::util::MessageDifferencer::SpecificField>&
                     parent_fields) override {
    if (message1.GetDescriptor()->full_name() ==
        google::protobuf::FieldDescriptorProto::descriptor()->full_name()) {
      if (field->name() != "json_name") {
        return false;
      }
      const google::protobuf::FieldDescriptor* name_field =
          message1.GetDescriptor()->FindFieldByName("name");
      if (name_field == nullptr) {
        return false;
      }
      const google::protobuf::Reflection* refl = message1.GetReflection();

      if (!refl->HasField(message1, name_field) ||
          !refl->HasField(message2, name_field)) {
        return false;
      }

      std::string value1 = refl->GetString(message1, field);
      std::string value2 = refl->GetString(message2, field);
      if (value1.empty() &&
          value2 == ToJsonName(refl->GetString(message2, name_field))) {
        return true;
      }
      if (value2.empty() &&
          value1 == ToJsonName(refl->GetString(message1, name_field))) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace

std::ostream& operator<<(
    std::ostream& os,
    const IsMessageAssignableResult::DiffSeverityLevel& level) {
  using enum IsMessageAssignableResult::DiffSeverityLevel;

  switch (level) {
    case kInfo:
      os << "kInfo";
      break;
    case kWarning:
      os << "kWarning";
      break;
    case kError:
      os << "kError";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const IsMessageAssignableResult::DiffType& type) {
  using enum IsMessageAssignableResult::DiffType;

  switch (type) {
    case kMessageNameChanged:
      os << "kMessageNameChanged";
      break;
    case kFieldNameChanged:
      os << "kFieldNameChanged";
      break;
    case kFieldNameChangedValuePresent:
      os << "kFieldNameChangedValuePresent";
      break;
    case kFieldNameChangedValueAbsent:
      os << "kFieldNameChangedValueAbsent";
      break;
    case kFieldTypeChanged:
      os << "kFieldTypeChanged";
      break;
    case kFieldTypeChangedValuePresent:
      os << "kFieldTypeChangedValuePresent";
      break;
    case kFieldTypeChangedValueAbsent:
      os << "kFieldTypeChangedValueAbsent";
      break;
    case kFieldNameAndTypeChanged:
      os << "kFieldNameAndTypeChanged";
      break;
    case kFieldNameAndTypeChangedValuePresent:
      os << "kFieldNameAndTypeChangedValuePresent";
      break;
    case kFieldNameAndTypeChangedValueAbsent:
      os << "kFieldNameAndTypeChangedValueAbsent";
      break;
    case kFieldNumberChanged:
      os << "kFieldNumberChanged";
      break;
    case kFieldNumberChangedValuePresent:
      os << "kFieldNumberChangedValuePresent";
      break;
    case kFieldNumberChangedValueAbsent:
      os << "kFieldNumberChangedValueAbsent";
      break;
    case kFieldDeleted:
      os << "kFieldDeleted";
      break;
    case kFieldDeletedValuePresent:
      os << "kFieldDeletedValuePresent";
      break;
    case kFieldDeletedValueAbsent:
      os << "kFieldDeletedValueAbsent";
      break;
    case kFieldAdded:
      os << "kFieldAdded";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const IsMessageAssignableResult::Diff& diff) {
  os << "{.severity = " << diff.severity << ", ";
  os << ".type = " << diff.type << ", ";
  os << ".summary = \"" << diff.summary << "\"}";
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const IsMessageAssignableResult& report) {
  os << "{" << std::boolalpha;
  os << ".is_assignable = " << report.is_assignable << ", ";
  os << ".diffs = {";
  os << absl::StrJoin(report.diffs, ", ", absl::StreamFormatter());
  os << "}}";
  return os;
}

namespace {

google::protobuf::FileDescriptorSet FileDescriptorSetWithEmptyMessageType(
    const MessageName& message_name) {
  google::protobuf::FileDescriptorSet result;
  auto* file = result.add_file();
  file->set_name("__synthetic_file.proto");
  file->set_package(message_name.package);
  file->add_message_type()->set_name(message_name.name);
  file->set_syntax("proto3");
  return result;
}

template <typename... Args>
void AddInfoDiff(std::vector<IsMessageAssignableResult::Diff>* diffs,
                 IsMessageAssignableResult::DiffType type,
                 const absl::FormatSpec<Args...>& format, const Args&... args) {
  diffs->emplace_back(IsMessageAssignableResult::DiffSeverityLevel::kInfo, type,
                      absl::StrFormat(format, args...));
}

template <typename... Args>
void AddWarningDiff(std::vector<IsMessageAssignableResult::Diff>* diffs,
                    IsMessageAssignableResult::DiffType type,
                    const absl::FormatSpec<Args...>& format,
                    const Args&... args) {
  diffs->emplace_back(IsMessageAssignableResult::DiffSeverityLevel::kWarning,
                      type, absl::StrFormat(format, args...));
}

template <typename... Args>
void AddErrorDiff(std::vector<IsMessageAssignableResult::Diff>* diffs,
                  IsMessageAssignableResult::DiffType type,
                  const absl::FormatSpec<Args...>& format,
                  const Args&... args) {
  diffs->emplace_back(IsMessageAssignableResult::DiffSeverityLevel::kError,
                      type, absl::StrFormat(format, args...));
}

template <typename... Args>
void AddDiff(std::vector<IsMessageAssignableResult::Diff>* diffs,
             IsMessageAssignableResult::DiffSeverityLevel severity,
             IsMessageAssignableResult::DiffType type,
             const absl::FormatSpec<Args...>& format, const Args&... args) {
  diffs->emplace_back(severity, type, absl::StrFormat(format, args...));
}

// Returns whether the given field is a map field and, if true, also returns the
// fields with the map's key and value types.
//
// Background: In a parsed descriptor, a map field 'MyMessage.my_map' can be
// identified by having the 'repeated' label and being of an auto-generated type
// called 'MyMessage.MyMapEntry'. The nested message type 'MyMessage.MyMapEntry'
// has the 'map_entry' option set and has 'two fields 'key' and 'value' whose
// types correspond to the types of the keys and values of the map. See
// https://protobuf.dev/programming-guides/proto3/#backwards.
absl::StatusOr<bool> IsMapField(
    const google::protobuf::FieldDescriptorProto& field,
    const DescriptorMaps& descriptors,
    const google::protobuf::FieldDescriptorProto*& key_field,
    const google::protobuf::FieldDescriptorProto*& value_field) {
  key_field = nullptr;
  value_field = nullptr;

  if (field.label() != google::protobuf::FieldDescriptorProto::LABEL_REPEATED ||
      field.type() != google::protobuf::FieldDescriptorProto::TYPE_MESSAGE) {
    return false;
  }

  INTR_ASSIGN_OR_RETURN(std::string type_name,
                        VerifyAndGetFieldTypeName(field));
  INTR_ASSIGN_OR_RETURN(
      const google::protobuf::DescriptorProto* map_entry_descriptor,
      descriptors.FindMessageDescriptor(type_name));

  if (!map_entry_descriptor->options().map_entry()) {
    return false;
  }

  for (const google::protobuf::FieldDescriptorProto& field :
       map_entry_descriptor->field()) {
    if (field.name() == "key") {
      key_field = &field;
    }
    if (field.name() == "value") {
      value_field = &field;
    }
  }

  if (key_field == nullptr || value_field == nullptr) {
    return absl::InvalidArgumentError(
        "Map entry descriptor does not have 'key' and 'value' field");
  }

  return true;
}

// Returns the type of the given field as a string as it would appear in a
// .proto file. Example outputs: "int32", "optional intrinsic_proto.Pose",
// "repeated string", "map<string, intrinsic_proto.Pose>".
absl::StatusOr<std::string> FormatFieldType(
    const google::protobuf::FieldDescriptorProto& field,
    const DescriptorMaps& descriptors) {
  {
    const google::protobuf::FieldDescriptorProto *key, *value;
    INTR_ASSIGN_OR_RETURN(bool is_map,
                          IsMapField(field, descriptors, key, value));

    if (is_map) {
      INTR_ASSIGN_OR_RETURN(std::string key_string,
                            FormatFieldType(*key, descriptors));
      INTR_ASSIGN_OR_RETURN(std::string value_string,
                            FormatFieldType(*value, descriptors));
      return absl::StrCat("map<", key_string, ", ", value_string, ">");
    }
  }

  std::string type;
  if (field.type() == google::protobuf::FieldDescriptorProto::TYPE_MESSAGE ||
      field.type() == google::protobuf::FieldDescriptorProto::TYPE_ENUM) {
    INTR_ASSIGN_OR_RETURN(type, VerifyAndGetFieldTypeName(field));
  } else {
    // Convert enum name to scalar type name, e.g., "TYPE_INT32" -> "int32".
    type = absl::AsciiStrToLower(absl::StripPrefix(
        google::protobuf::FieldDescriptorProto::Type_Name(field.type()),
        "TYPE_"));
  }

  if (field.proto3_optional()) {
    return absl::StrCat("optional ", type);
  }
  if (field.label() == google::protobuf::FieldDescriptorProto::LABEL_REPEATED) {
    return absl::StrCat("repeated ", type);
  }
  return type;
}

// Collects the names of all fields that are present in the given message and
// any nested messages and adds them to `present_fields` in the format
// "message_full_name.field_name" (e.g., "intrinsic_proto.Pose.x"). Recurses
// through all nested messages which includes singular/repeated message fields
// and maps with message values.
//
// For example, if the message definition is
//     package foo;
//     message M {
//       repeated intrinsic_proto.Pose poses = 1;
//     }
// and `message` is
//     {
//       poses {x: 1}
//       poses {z: 1}
//     }
// then this method will add {"foo.M.poses", "intrinsic_proto.Pose.x",
// "intrinsic_proto.Pose.z"} to `present_fields`.
absl::Status CollectPresentFields(
    const google::protobuf::Message& message,
    absl::flat_hash_set<std::string>& present_fields) {
  const google::protobuf::Descriptor* descriptor = message.GetDescriptor();
  const google::protobuf::Reflection* reflection = message.GetReflection();

  for (int i = 0; i < descriptor->field_count(); i++) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);

    if (field->is_repeated()) {
      if (reflection->FieldSize(message, field) > 0) {
        present_fields.insert(
            absl::StrCat(descriptor->full_name(), ".", field->name()));
      }

      if (field->is_map()) {
        for (int j = 0; j < reflection->FieldSize(message, field); ++j) {
          const google::protobuf::Message& map_entry_message =
              reflection->GetRepeatedMessage(message, field, j);
          const google::protobuf::FieldDescriptor* value_field =
              map_entry_message.GetDescriptor()->FindFieldByName("value");

          if (value_field == nullptr) {
            return absl::InvalidArgumentError(
                "Map entry descriptor does not have 'value' field");
          }

          if (value_field->type() ==
              google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
            const google::protobuf::Message& value_message =
                map_entry_message.GetReflection()->GetMessage(map_entry_message,
                                                              value_field);
            INTR_RETURN_IF_ERROR(
                CollectPresentFields(value_message, present_fields));
          }
        }
      } else {
        // Repeated non-map field.
        if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
          for (int j = 0; j < reflection->FieldSize(message, field); ++j) {
            const google::protobuf::Message& nested_message =
                reflection->GetRepeatedMessage(message, field, j);
            INTR_RETURN_IF_ERROR(
                CollectPresentFields(nested_message, present_fields));
          }
        }
      }
    } else if (reflection->HasField(message, field)) {
      present_fields.insert(
          absl::StrCat(descriptor->full_name(), ".", field->name()));

      if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        const google::protobuf::Message& nested_message =
            reflection->GetMessage(message, field);
        INTR_RETURN_IF_ERROR(
            CollectPresentFields(nested_message, present_fields));
      }
    }
  }

  return absl::OkStatus();
}

// Ordered from least to most compatible so that, e.g., "<"-comparisons make
// sense or std::min/std::max can be used to get the "weaker"/"stronger" of two
// compatibilities.
enum TypeCompatibility {
  kIncompatible = 0,  // Neither assignable nor equal
  kAssignable = 1,    // Assignable but not equal
  kEqual = 2,         // Equal (in all checked properties)
};

// Compares the types of two fields. If the fields reference the same message
// type, the full name of that referenced message type is returned in
// `child_message_full_name`.
absl::StatusOr<TypeCompatibility> CompareFieldType(
    const google::protobuf::FieldDescriptorProto& field_a,
    const google::protobuf::FieldDescriptorProto& field_b,
    const DescriptorMapsPair& descriptors,
    std::optional<MessageName>* child_message_name = nullptr) {
  if (child_message_name != nullptr) {
    child_message_name->reset();
  }

  if (field_a.type() != field_b.type() || field_a.label() != field_b.label()) {
    return kIncompatible;
  }

  if (field_a.proto3_optional() != field_b.proto3_optional()) {
    return kAssignable;
  }

  const google::protobuf::FieldDescriptorProto *key_a, *value_a;
  const google::protobuf::FieldDescriptorProto *key_b, *value_b;
  INTR_ASSIGN_OR_RETURN(bool is_map_a,
                        IsMapField(field_a, descriptors.a, key_a, value_a));
  INTR_ASSIGN_OR_RETURN(bool is_map_b,
                        IsMapField(field_b, descriptors.b, key_b, value_b));

  // Map fields.
  if (is_map_a && is_map_b) {
    INTR_ASSIGN_OR_RETURN(TypeCompatibility key_compatibility,
                          CompareFieldType(*key_a, *key_b, descriptors));
    INTR_ASSIGN_OR_RETURN(
        TypeCompatibility value_compatibility,
        CompareFieldType(*value_a, *value_b, descriptors, child_message_name));
    // The overall compatibility is the "weaker" compatibility of the two. E.g.,
    // if the key type is "assignable" (1) but the value type is "incompatible"
    // (0) the map type as a whole is "incompatible" (0).
    return std::min(key_compatibility, value_compatibility);
  } else if (is_map_a != is_map_b) {
    return kIncompatible;
  }

  // Singular or repeated fields.
  if (field_a.type() == google::protobuf::FieldDescriptorProto::TYPE_ENUM ||
      field_a.type() == google::protobuf::FieldDescriptorProto::TYPE_MESSAGE) {
    INTR_ASSIGN_OR_RETURN(std::string type_name_a,
                          VerifyAndGetFieldTypeName(field_a));
    INTR_ASSIGN_OR_RETURN(std::string type_name_b,
                          VerifyAndGetFieldTypeName(field_b));

    if (type_name_a != type_name_b) {
      return kIncompatible;
    }

    if (field_a.type() ==
            google::protobuf::FieldDescriptorProto::TYPE_MESSAGE &&
        child_message_name != nullptr) {
      child_message_name->emplace(type_name_b);
    }
    return kEqual;
  }

  return kEqual;
}

absl::Status AddDiffsForFieldsHavingSameNumber(
    bool field_name_equal, TypeCompatibility type_compatibility,
    const MessageName& message_name,
    const google::protobuf::FieldDescriptorProto& field_a,
    const google::protobuf::FieldDescriptorProto& field_b,
    const DescriptorMapsPair& descriptors,
    const std::optional<absl::flat_hash_set<std::string>>& present_fields_b,
    std::vector<IsMessageAssignableResult::Diff>* diffs) {
  if (type_compatibility == TypeCompatibility::kEqual) {
    if (!field_name_equal) {
      // Only field name changed
      if (present_fields_b.has_value()) {
        bool is_field_present = present_fields_b->contains(
            absl::StrCat(message_name.full_name, ".", field_b.name()));

        if (is_field_present) {
          AddInfoDiff(diffs,
                      IsMessageAssignableResult::DiffType::
                          kFieldNameChangedValuePresent,
                      "Field '%s.%s' renamed to '%s.%s' and the field had a "
                      "value assigned (full names: '%s.%s' renamed to '%s.%s')",
                      message_name.name, field_b.name(), message_name.name,
                      field_a.name(), message_name.full_name, field_b.name(),
                      message_name.full_name, field_a.name());
        } else {
          AddInfoDiff(
              diffs,
              IsMessageAssignableResult::DiffType::kFieldNameChangedValueAbsent,
              "Field '%s.%s' renamed to '%s.%s' but the field had no value "
              "assigned (full names: '%s.%s' renamed to '%s.%s')",
              message_name.name, field_b.name(), message_name.name,
              field_a.name(), message_name.full_name, field_b.name(),
              message_name.full_name, field_a.name());
        }
      } else {
        AddInfoDiff(diffs,
                    IsMessageAssignableResult::DiffType::kFieldNameChanged,
                    "Field '%s.%s' renamed to '%s.%s' (full names: '%s.%s' "
                    "renamed to '%s.%s')",
                    message_name.name, field_b.name(), message_name.name,
                    field_a.name(), message_name.full_name, field_b.name(),
                    message_name.full_name, field_a.name());
      }
    }
    return absl::OkStatus();
  }

  // If we reach this point, the types are "assignable" or "incompatible" (but
  // not equal).
  INTR_ASSIGN_OR_RETURN(std::string type_a_string,
                        FormatFieldType(field_a, descriptors.a));
  INTR_ASSIGN_OR_RETURN(std::string type_b_string,
                        FormatFieldType(field_b, descriptors.b));

  IsMessageAssignableResult::DiffSeverityLevel
      warning_if_assignable_else_error =
          type_compatibility == TypeCompatibility::kAssignable
              ? IsMessageAssignableResult::DiffSeverityLevel::kWarning
              : IsMessageAssignableResult::DiffSeverityLevel::kError;

  if (field_name_equal) {
    // Only field type changed
    if (present_fields_b.has_value()) {
      bool is_field_present = present_fields_b->contains(
          absl::StrCat(message_name.full_name, ".", field_b.name()));

      if (is_field_present) {
        AddDiff(
            diffs, warning_if_assignable_else_error,
            IsMessageAssignableResult::DiffType::kFieldTypeChangedValuePresent,
            "Type of field '%s.%s' changed from '%s' to '%s' and the field had "
            "a value assigned (full field name: '%s.%s')",
            message_name.name, field_b.name(), type_b_string, type_a_string,
            message_name.full_name, field_b.name());
      } else {
        AddInfoDiff(
            diffs,
            IsMessageAssignableResult::DiffType::kFieldTypeChangedValueAbsent,
            "Type of field '%s.%s' changed from '%s' to '%s' but the field had "
            "no value assigned (full field name: '%s.%s')",
            message_name.name, field_b.name(), type_b_string, type_a_string,
            message_name.full_name, field_b.name());
      }
    } else {
      AddDiff(diffs, warning_if_assignable_else_error,
              IsMessageAssignableResult::DiffType::kFieldTypeChanged,
              "Type of field '%s.%s' changed from '%s' to '%s' (full field "
              "name: '%s.%s')",
              message_name.name, field_b.name(), type_b_string, type_a_string,
              message_name.full_name, field_b.name());
    }
  } else {
    // Field name and type changed
    if (present_fields_b.has_value()) {
      bool is_field_present = present_fields_b->contains(
          absl::StrCat(message_name.full_name, ".", field_b.name()));

      if (is_field_present) {
        AddDiff(
            diffs, warning_if_assignable_else_error,
            IsMessageAssignableResult::DiffType::
                kFieldNameAndTypeChangedValuePresent,
            "Type and name of field %d in '%s' changed from '%s %s' to '%s %s' "
            "and the field had a value assigned (full message name: '%s')",
            field_a.number(), message_name.name, type_b_string, field_b.name(),
            type_a_string, field_a.name(), message_name.full_name);
      } else {
        AddInfoDiff(
            diffs,
            IsMessageAssignableResult::DiffType::
                kFieldNameAndTypeChangedValueAbsent,
            "Type and name of field %d in '%s' changed from '%s %s' to '%s %s' "
            "but the field had no value assigned (full message name: '%s')",
            field_a.number(), message_name.name, type_b_string, field_b.name(),
            type_a_string, field_a.name(), message_name.full_name);
      }
    } else {
      AddDiff(diffs, warning_if_assignable_else_error,
              IsMessageAssignableResult::DiffType::kFieldNameAndTypeChanged,
              "Type and name of field %d in '%s' changed from '%s %s' to '%s "
              "%s' (full message name: '%s')",
              field_a.number(), message_name.name, type_b_string,
              field_b.name(), type_a_string, field_a.name(),
              message_name.full_name);
    }
  }

  return absl::OkStatus();
}

// Adds diff for fields having the same type and name but different numbers.
absl::Status AddDiffsForFieldsHavingSameTypeAndName(
    const MessageName& message_name,
    const google::protobuf::FieldDescriptorProto& field_a,
    const google::protobuf::FieldDescriptorProto& field_b,
    const std::optional<absl::flat_hash_set<std::string>>& present_fields_b,
    std::vector<IsMessageAssignableResult::Diff>* diffs) {
  if (present_fields_b.has_value()) {
    bool is_field_present = present_fields_b->contains(
        absl::StrCat(message_name.full_name, ".", field_b.name()));

    if (is_field_present) {
      AddWarningDiff(
          diffs,
          IsMessageAssignableResult::DiffType::kFieldNumberChangedValuePresent,
          "Number of field '%s.%s' changed from %d to %d and the field had a "
          "value assigned, this value will be lost (full field name: '%s.%s')",
          message_name.name, field_a.name(), field_b.number(), field_a.number(),
          message_name.full_name, field_a.name());
    } else {
      AddInfoDiff(
          diffs,
          IsMessageAssignableResult::DiffType::kFieldNumberChangedValueAbsent,
          "Number of field '%s.%s' changed from %d to %d but the field had no "
          "value assigned (full field name: '%s.%s')",
          message_name.name, field_a.name(), field_b.number(), field_a.number(),
          message_name.full_name, field_a.name());
    }
  } else {
    AddWarningDiff(diffs,
                   IsMessageAssignableResult::DiffType::kFieldNumberChanged,
                   "Number of field '%s.%s' changed from %d to %d, any "
                   "assigned values will be lost (full field name: '%s.%s')",
                   message_name.name, field_a.name(), field_b.number(),
                   field_a.number(), message_name.full_name, field_a.name());
  }

  return absl::OkStatus();
}

absl::Status AddDiffsForDeletedField(
    const MessageName& message_name,
    const google::protobuf::FieldDescriptorProto& field_b,
    const std::optional<absl::flat_hash_set<std::string>>& present_fields_b,
    std::vector<IsMessageAssignableResult::Diff>* diffs) {
  if (present_fields_b.has_value()) {
    bool is_field_present = present_fields_b->contains(
        absl::StrCat(message_name.full_name, ".", field_b.name()));

    if (is_field_present) {
      AddWarningDiff(
          diffs, IsMessageAssignableResult::DiffType::kFieldDeletedValuePresent,
          "Field '%s.%s' was deleted and had a value assigned (full field "
          "name: '%s.%s')",
          message_name.name, field_b.name(), message_name.full_name,
          field_b.name());
    } else {
      AddInfoDiff(diffs,
                  IsMessageAssignableResult::DiffType::kFieldDeletedValueAbsent,
                  "Field '%s.%s' was deleted but had no value assigned (full "
                  "field name: '%s.%s')",
                  message_name.name, field_b.name(), message_name.full_name,
                  field_b.name());
    }
  } else {
    AddWarningDiff(diffs, IsMessageAssignableResult::DiffType::kFieldDeleted,
                   "Field '%s.%s' was deleted (full field name: '%s.%s')",
                   message_name.name, field_b.name(), message_name.full_name,
                   field_b.name());
  }

  return absl::OkStatus();
}

absl::StatusOr<std::vector<IsMessageAssignableResult::Diff>>
DiffMessageForAssignability(
    const MessageName& message_name, const DescriptorMapsPair& descriptors,
    const std::optional<absl::flat_hash_set<std::string>>& present_fields_b,
    absl::flat_hash_set<std::string>* visited_message_full_names) {
  visited_message_full_names->insert(message_name.full_name);

  INTR_ASSIGN_OR_RETURN(
      const google::protobuf::DescriptorProto* descriptor_a,
      descriptors.a.FindMessageDescriptor(message_name.full_name));
  INTR_ASSIGN_OR_RETURN(
      const google::protobuf::DescriptorProto* descriptor_b,
      descriptors.b.FindMessageDescriptor(message_name.full_name));

  // Collects diffs for the current message.
  std::vector<IsMessageAssignableResult::Diff> diffs;
  // Collects diffs for message types used in the current message.
  std::vector<IsMessageAssignableResult::Diff> children_diffs;

  // Get all fields for both descriptors. Use sorted maps so that the order of
  // the diffs which we generate below is deterministic.
  absl::btree_map<int, const google::protobuf::FieldDescriptorProto*> fields_a,
      fields_b;
  for (const google::protobuf::FieldDescriptorProto& field :
       descriptor_a->field()) {
    fields_a[field.number()] = &field;
  }
  for (const google::protobuf::FieldDescriptorProto& field :
       descriptor_b->field()) {
    fields_b[field.number()] = &field;
  }

  // Find pairs of fields with matching numbers, generate diffs for any changes
  // to those fields and remove the fields from the maps.
  for (auto it_a = fields_a.begin(); it_a != fields_a.end();) {
    int field_number_a = it_a->first;

    if (auto it_b = fields_b.find(field_number_a); it_b != fields_b.end()) {
      // Match by field number found (name and/or type can be different).
      const google::protobuf::FieldDescriptorProto* field_a = it_a->second;
      const google::protobuf::FieldDescriptorProto* field_b = it_b->second;

      bool field_name_equal = field_a->name() == field_b->name();
      std::optional<MessageName> child_message_name;
      INTR_ASSIGN_OR_RETURN(TypeCompatibility type_compatibility,
                            CompareFieldType(*field_a, *field_b, descriptors,
                                             &child_message_name));

      // Recursively check used message types if the matching field pair
      // generally has the same type.
      if (type_compatibility >= kAssignable && child_message_name.has_value() &&
          !visited_message_full_names->contains(
              child_message_name->full_name)) {
        INTR_ASSIGN_OR_RETURN(
            std::vector<IsMessageAssignableResult::Diff> used_message_diffs,
            DiffMessageForAssignability(*child_message_name, descriptors,
                                        present_fields_b,
                                        visited_message_full_names));
        children_diffs.insert(children_diffs.end(), used_message_diffs.begin(),
                              used_message_diffs.end());
      }

      INTR_RETURN_IF_ERROR(AddDiffsForFieldsHavingSameNumber(
          field_name_equal, type_compatibility, message_name, *field_a,
          *field_b, descriptors, present_fields_b, &diffs));

      it_a = fields_a.erase(it_a);
      fields_b.erase(it_b);
    } else {
      ++it_a;
    }
  }

  // In the remaining fields (which all have different field numbers), match
  // pairs of fields which have the same name and type. Generate diffs for any
  // changes between matched fields and remove the fields from the maps.
  for (auto it_a = fields_a.begin(); it_a != fields_a.end();) {
    bool match_found = false;
    for (auto it_b = fields_b.begin(); it_b != fields_b.end(); ++it_b) {
      const google::protobuf::FieldDescriptorProto* field_a = it_a->second;
      const google::protobuf::FieldDescriptorProto* field_b = it_b->second;

      bool field_name_equal = field_a->name() == field_b->name();
      std::optional<MessageName> child_message_name;
      INTR_ASSIGN_OR_RETURN(TypeCompatibility type_compatibility,
                            CompareFieldType(*field_a, *field_b, descriptors,
                                             &child_message_name));

      if (field_name_equal && type_compatibility >= kAssignable) {
        // Match found (field number is different, name and type are the same).
        match_found = true;

        // Recursively check used message types.
        if (child_message_name.has_value() &&
            !visited_message_full_names->contains(
                child_message_name->full_name)) {
          // This recursive call can add error diffs even though there cannot
          // be any problem with deserialization (the field numbers are
          // different). Change once this causes problems.
          INTR_ASSIGN_OR_RETURN(
              std::vector<IsMessageAssignableResult::Diff> used_message_diffs,
              DiffMessageForAssignability(*child_message_name, descriptors,
                                          present_fields_b,
                                          visited_message_full_names));
          children_diffs.insert(children_diffs.end(),
                                used_message_diffs.begin(),
                                used_message_diffs.end());
        }

        INTR_RETURN_IF_ERROR(AddDiffsForFieldsHavingSameTypeAndName(
            message_name, *field_a, *field_b, present_fields_b, &diffs));

        it_a = fields_a.erase(it_a);
        fields_b.erase(it_b);
        break;
      }
    }

    if (!match_found) {
      ++it_a;
    }
  }

  // Leftover fields in `fields_a` had no match in fields_b and are considered
  // to be "added" fields.
  for (const auto& [_, field_a] : fields_a) {
    AddInfoDiff(&diffs, IsMessageAssignableResult::DiffType::kFieldAdded,
                "Field '%s.%s' was added (full field name: '%s.%s')",
                message_name.name, field_a->name(), message_name.full_name,
                field_a->name());
  }

  // Leftover fields in `fields_b` had no match in fields_a and are considered
  // to be "deleted" fields.
  for (const auto& [field_number_b, field_proto_b] : fields_b) {
    INTR_RETURN_IF_ERROR(AddDiffsForDeletedField(message_name, *field_proto_b,
                                                 present_fields_b, &diffs));
  }

  // Diffs for the current message first, then the diffs for messages used by
  // the current message (i.e., compose diffs in pre-traversal order).
  diffs.insert(diffs.end(), children_diffs.begin(), children_diffs.end());

  return diffs;
}

}  // namespace

absl::StatusOr<IsMessageAssignableResult> IsMessageAssignable(
    std::string_view message_full_name_a,
    const google::protobuf::FileDescriptorSet* absl_nullable fds_a,
    std::string_view message_full_name_b,
    const google::protobuf::FileDescriptorSet* absl_nullable fds_b,
    const google::protobuf::Message* absl_nullable message_b) {
  MessageName message_name_a(message_full_name_a);
  MessageName message_name_b(message_full_name_b);

  std::optional<google::protobuf::FileDescriptorSet> empty_fds_a, empty_fds_b;
  if (!fds_a) {
    empty_fds_a = FileDescriptorSetWithEmptyMessageType(message_name_a);
    fds_a = &empty_fds_a.value();
  }
  if (!fds_b) {
    empty_fds_b = FileDescriptorSetWithEmptyMessageType(message_name_b);
    fds_b = &empty_fds_b.value();
  }

  DescriptorMapsPair descriptors{
      .a = CollectDescriptors(*fds_a, {}),
      .b = CollectDescriptors(*fds_b, {}),
  };

  std::optional<absl::flat_hash_set<std::string>> present_fields_b;
  if (message_b != nullptr) {
    present_fields_b.emplace();
    INTR_RETURN_IF_ERROR(CollectPresentFields(*message_b, *present_fields_b));
  }

  std::vector<IsMessageAssignableResult::Diff> diffs;

  if (message_name_a.full_name != message_name_b.full_name) {
    AddErrorDiff(&diffs,
                 IsMessageAssignableResult::DiffType::kMessageNameChanged,
                 "Top-level message '%s' changed to '%s' (full names: '%s' "
                 "changed to '%s')",
                 message_name_b.name, message_name_a.name,
                 message_name_b.full_name, message_name_a.full_name);
  } else {
    absl::flat_hash_set<std::string> visited_message_full_names;
    INTR_ASSIGN_OR_RETURN(
        diffs, DiffMessageForAssignability(message_name_a, descriptors,
                                           present_fields_b,
                                           &visited_message_full_names));
  }

  bool is_assignable = absl::c_all_of(diffs, [](const auto& diff) {
    return diff.severity !=
           IsMessageAssignableResult::DiffSeverityLevel::kError;
  });

  return IsMessageAssignableResult{.is_assignable = is_assignable,
                                   .diffs = std::move(diffs)};
}

std::ostream& operator<<(
    std::ostream& os,
    const IsMessageDeserializableResult::IssueSeverityLevel& level) {
  using enum IsMessageDeserializableResult::IssueSeverityLevel;

  switch (level) {
    case kWarning:
      os << "kWarning";
      break;
    case kError:
      os << "kError";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const IsMessageDeserializableResult::IssueType& type) {
  using enum IsMessageDeserializableResult::IssueType;

  switch (type) {
    case kInvalidTypeUrl:
      os << "kInvalidTypeUrl";
      break;
    case kInvalidBinaryRepresentation:
      os << "kInvalidBinaryRepresentation";
      break;
    case kFieldMissing:
      os << "kFieldMissing";
      break;
    case kFieldTypeMismatch:
      os << "kFieldTypeMismatch";
      break;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const IsMessageDeserializableResult::Issue& issue) {
  os << "{.severity = " << issue.severity << ", ";
  os << ".type = " << issue.type << ", ";
  os << ".summary = \"" << issue.summary << "\"}";
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const IsMessageDeserializableResult& result) {
  os << "{" << std::boolalpha;
  os << ".is_deserializable = " << result.is_deserializable << ", ";
  os << ".diffs = {";
  os << absl::StrJoin(result.issues, ", ", absl::StreamFormatter());
  os << "}}";
  return os;
}

namespace {

std::string WireTypeToString(google::protobuf::UnknownField::Type wire_type) {
  switch (wire_type) {
    case google::protobuf::UnknownField::TYPE_VARINT:
      return "VARINT";
    case google::protobuf::UnknownField::TYPE_FIXED32:
      return "FIXED32";
    case google::protobuf::UnknownField::TYPE_FIXED64:
      return "FIXED64";
    case google::protobuf::UnknownField::TYPE_GROUP:
      return "GROUP";
    case google::protobuf::UnknownField::TYPE_LENGTH_DELIMITED:
      return "LENGTH_DELIMITED";
    default:
      return "UNKNOWN";
  }
}

// Returns the field types which the given wire-type can encode as a string to
// be included in user messages.
std::string WireTypeToFieldTypesString(
    google::protobuf::UnknownField::Type wire_type) {
  switch (wire_type) {
    case google::protobuf::UnknownField::TYPE_VARINT:
      return "bool, enum, int32, int64, uint32, uint64, sint32 or sint64";
    case google::protobuf::UnknownField::TYPE_FIXED32:
      return "float, fixed32 or sfixed32";
    case google::protobuf::UnknownField::TYPE_FIXED64:
      return "double, fixed64 or sfixed64";
    case google::protobuf::UnknownField::TYPE_GROUP:
      return "group";
    case google::protobuf::UnknownField::TYPE_LENGTH_DELIMITED:
      return "string, message or bytes";
    default:
      return "<no types>";
  }
}

std::string UnknownFieldValueToString(
    const google::protobuf::UnknownField& unknown_field) {
  constexpr int kMaxChars = 100;

  switch (unknown_field.type()) {
    case google::protobuf::UnknownField::TYPE_VARINT:
      return absl::StrCat(unknown_field.varint());
    case google::protobuf::UnknownField::TYPE_FIXED32:
      return absl::StrCat(unknown_field.fixed32());
    case google::protobuf::UnknownField::TYPE_FIXED64:
      return absl::StrCat(unknown_field.fixed64());
    case google::protobuf::UnknownField::TYPE_LENGTH_DELIMITED: {
      std::string escaped_value =
          absl::CEscape(unknown_field.length_delimited());
      // Return a quoted string with a maximum of 100 chars between the
      // quotes.
      return escaped_value.size() <= kMaxChars
                 ? absl::StrCat("\"", escaped_value, "\"")
                 : absl::StrCat("\"", escaped_value.substr(0, kMaxChars - 3),
                                "...\"");
    }
    case google::protobuf::UnknownField::TYPE_GROUP:
      // No support required for deprecated proto2 type.
      ABSL_FALLTHROUGH_INTENDED;
    default:
      return "<unsupported value type>";
  }
}

IsMessageDeserializableResult ResultWithTypeUrlIssue(std::string_view summary) {
  return IsMessageDeserializableResult{
      .is_deserializable = false,
      .issues = {IsMessageDeserializableResult::Issue{
          .severity = IsMessageDeserializableResult::IssueSeverityLevel::kError,
          .type = IsMessageDeserializableResult::IssueType::kInvalidTypeUrl,
          .summary = std::string(summary)}}};
}

IsMessageDeserializableResult::Issue IssueForUnknownField(
    const google::protobuf::UnknownField& unknown_field,
    const google::protobuf::Descriptor& msg_descriptor) {
  const google::protobuf::FieldDescriptor* field =
      msg_descriptor.FindFieldByNumber(unknown_field.number());

  if (field) {
    // Field exists -> type mismatch -> generate error
    std::string summary = absl::StrFormat(
        "value %s of wire-type %s (serialized from %s) for field %s (%d) "
        "cannot be deserialized because the target field has an incompatible "
        "type (%s)",
        UnknownFieldValueToString(unknown_field),
        WireTypeToString(unknown_field.type()),
        WireTypeToFieldTypesString(unknown_field.type()), field->full_name(),
        field->number(),
        google::protobuf::FieldDescriptor::TypeName(field->type()));
    return IsMessageDeserializableResult::Issue{
        .severity = IsMessageDeserializableResult::IssueSeverityLevel::kError,
        .type = IsMessageDeserializableResult::IssueType::kFieldTypeMismatch,
        .summary = std::move(summary)};
  } else {
    // Field does not exist (e.g. was removed) -> generate warning
    std::string summary = absl::StrFormat(
        "value %s of wire-type %s (serialized from %s) for field %d in %s "
        "cannot be deserialized because no field with that number exists",
        UnknownFieldValueToString(unknown_field),
        WireTypeToString(unknown_field.type()),
        WireTypeToFieldTypesString(unknown_field.type()),
        unknown_field.number(), msg_descriptor.full_name());
    return IsMessageDeserializableResult::Issue{
        .severity = IsMessageDeserializableResult::IssueSeverityLevel::kWarning,
        .type = IsMessageDeserializableResult::IssueType::kFieldMissing,
        .summary = std::move(summary)};
  }
}

absl::Status GenerateIssuesForUnknownFields(
    const google::protobuf::Message& msg,
    std::vector<IsMessageDeserializableResult::Issue>& issues) {
  // Generate one issue for every unknown field
  const google::protobuf::UnknownFieldSet& unknown_fields =
      msg.GetReflection()->GetUnknownFields(msg);

  for (int i = 0; i < unknown_fields.field_count(); i++) {
    issues.push_back(
        IssueForUnknownField(unknown_fields.field(i), *msg.GetDescriptor()));
  }

  // Recurse into nested messages
  for (int j = 0; j < msg.GetDescriptor()->field_count(); j++) {
    const google::protobuf::FieldDescriptor* field =
        msg.GetDescriptor()->field(j);

    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      if (field->is_repeated()) {
        // Repeated nested messages (can be map entry messages)
        for (int i = 0; i < msg.GetReflection()->FieldSize(msg, field); i++) {
          const google::protobuf::Message& nested_msg =
              msg.GetReflection()->GetRepeatedMessage(msg, field, i);

          INTR_RETURN_IF_ERROR(
              GenerateIssuesForUnknownFields(nested_msg, issues));
        }
      } else if (msg.GetReflection()->HasField(msg, field)) {
        // Singular nested message
        const google::protobuf::Message& nested_msg =
            msg.GetReflection()->GetMessage(msg, field);

        INTR_RETURN_IF_ERROR(
            GenerateIssuesForUnknownFields(nested_msg, issues));
      }
    }
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<IsMessageDeserializableResult> IsMessageDeserializable(
    const google::protobuf::Any& any,
    const google::protobuf::DescriptorPool& desc_pool,
    google::protobuf::MessageFactory& msg_factory) {
  size_t slash_pos = any.type_url().find_last_of('/');
  if (slash_pos == std::string::npos) {
    return ResultWithTypeUrlIssue(
        absl::StrFormat("No slash in type URL '%s'", any.type_url()));
  }
  std::string message_name = any.type_url().substr(slash_pos + 1);

  if (message_name.empty()) {
    return ResultWithTypeUrlIssue(absl::StrFormat(
        "Type URL '%s' does not contain a message name", any.type_url()));
  }

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>> msg =
      CreateProtoInstanceFromDescriptorPool(message_name, &desc_pool,
                                            &msg_factory);

  if (absl::IsNotFound(msg.status())) {
    return ResultWithTypeUrlIssue(absl::StrFormat(
        "Type '%s' does not exist in descriptor pool", any.type_url()));
  }
  INTR_RETURN_IF_ERROR(msg.status());

  if (!any.UnpackTo(msg->get())) {
    return IsMessageDeserializableResult{
        .is_deserializable = false,
        .issues = {IsMessageDeserializableResult::Issue{
            .severity =
                IsMessageDeserializableResult::IssueSeverityLevel::kError,
            .type = IsMessageDeserializableResult::IssueType::
                kInvalidBinaryRepresentation,
            .summary = "binary representation of message is invalid and cannot "
                       "be deserialized"}}};
  }

  std::vector<IsMessageDeserializableResult::Issue> issues;
  INTR_RETURN_IF_ERROR(GenerateIssuesForUnknownFields(**msg, issues));

  bool has_any_error = absl::c_any_of(
      issues, [](const IsMessageDeserializableResult::Issue& issue) {
        return issue.severity ==
               IsMessageDeserializableResult::IssueSeverityLevel::kError;
      });

  return IsMessageDeserializableResult{.is_deserializable = !has_any_error,
                                       .issues = std::move(issues)};
}

absl::StatusOr<IsMessageDeserializableResult> IsMessageDeserializable(
    const google::protobuf::Any& any,
    const google::protobuf::FileDescriptorSet& fds) {
  google::protobuf::SimpleDescriptorDatabase db;
  INTR_RETURN_IF_ERROR(PopulateDescriptorDatabase(&db, fds));
  google::protobuf::DescriptorPool pool(&db);
  google::protobuf::DynamicMessageFactory factory(&pool);

  return IsMessageDeserializable(any, pool, factory);
}

}  // namespace intrinsic
