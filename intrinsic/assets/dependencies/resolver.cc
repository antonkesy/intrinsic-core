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

#include "intrinsic/assets/dependencies/resolver.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/functional/function_ref.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "google/protobuf/message_lite.h"
#include "intrinsic/assets/id_utils.h"
#include "intrinsic/assets/interface_utils.h"
#include "intrinsic/assets/proto/field_metadata.pb.h"
#include "intrinsic/assets/proto/v1/dependency.pb.h"
#include "intrinsic/assets/proto/v1/grpc_connection.pb.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/executive/engine/skill_action.h"
#include "intrinsic/util/proto/walk_messages.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace assets {

namespace {

absl::Status ProcessField(
    google::protobuf::Message& message,
    const google::protobuf::FieldDescriptor* field,
    const google::protobuf::FieldDescriptor* value_field, /* for map values */
    absl::FunctionRef<absl::Status(google::protobuf::Message&)> process_fn) {
  if (field->is_repeated() && !field->is_map()) {
    for (int i = 0; i < message.GetReflection()->FieldSize(message, field);
         ++i) {
      INTR_RETURN_IF_ERROR(
          process_fn(*message.GetReflection()->MutableRepeatedMessage(
              &message, field, i)));
    }
  } else if (field->is_map()) {
    for (int i = 0; i < message.GetReflection()->FieldSize(message, field);
         ++i) {
      google::protobuf::Message* map_entry =
          message.GetReflection()->MutableRepeatedMessage(&message, field, i);
      INTR_RETURN_IF_ERROR(process_fn(
          *map_entry->GetReflection()->MutableMessage(map_entry, value_field)));
    }
  } else {
    INTR_RETURN_IF_ERROR(
        process_fn(*message.GetReflection()->MutableMessage(&message, field)));
  }
  return absl::OkStatus();
}

}  // namespace

absl::Status Resolver::ResolveAnyField(
    google::protobuf::Message& any_message, executive::SkillAction skill_action,
    const google::protobuf::DescriptorPool* pool,
    google::protobuf::MessageFactory* message_factory) const {
  if (pool == nullptr || message_factory == nullptr) {
    return absl::InvalidArgumentError(
        "Descriptor pool or message factory is null");
  }

  if (any_message.ByteSizeLong() == 0) {
    return absl::OkStatus();
  }

  const google::protobuf::Reflection* reflection = any_message.GetReflection();
  const google::protobuf::Descriptor* any_descriptor =
      any_message.GetDescriptor();
  const google::protobuf::FieldDescriptor* type_url_field =
      any_descriptor->FindFieldByName("type_url");
  const google::protobuf::FieldDescriptor* value_field =
      any_descriptor->FindFieldByName("value");

  if (type_url_field == nullptr ||
      type_url_field->type() !=
          google::protobuf::FieldDescriptor::TYPE_STRING ||
      value_field == nullptr ||
      value_field->type() != google::protobuf::FieldDescriptor::TYPE_BYTES) {
    return absl::InternalError(
        "Message of type google.protobuf.Any does not have type_url or value "
        "fields.");
  }

  std::string type_url = reflection->GetString(any_message, type_url_field);
  if (type_url.empty()) {
    return absl::InvalidArgumentError("Any message has empty type URL.");
  }

  std::string full_type_name;
  if (!google::protobuf::Any::ParseAnyTypeUrl(type_url, &full_type_name)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Could not parse Any type URL: ", type_url));
  }

  const google::protobuf::Descriptor* descriptor =
      pool->FindMessageTypeByName(full_type_name);
  if (descriptor == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Could not find descriptor for type ", full_type_name));
  }

  const google::protobuf::Message* prototype =
      message_factory->GetPrototype(descriptor);
  if (prototype == nullptr) {
    return absl::InternalError(absl::StrCat(
        "Failed to get prototype for descriptor: ", full_type_name));
  }
  std::unique_ptr<google::protobuf::Message> unpacked_msg(prototype->New());
  if (!unpacked_msg->ParseFromString(
          reflection->GetString(any_message, value_field))) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to unpack Any proto with type URL: ", type_url));
  }

  INTR_RETURN_IF_ERROR(RecursivelyWalkMessage(
      *unpacked_msg,
      [this, skill_action, pool, message_factory](
          google::protobuf::Message& msg) -> absl::StatusOr<bool> {
        return this->ResolveDependency(msg, skill_action, pool,
                                       message_factory);
      }));

  std::string new_value;
  if (!unpacked_msg->SerializeToString(&new_value)) {
    return absl::InternalError(
        "Failed to serialize resolved message to string.");
  }
  reflection->SetString(&any_message, value_field, new_value);

  return absl::OkStatus();
}

absl::Status Resolver::ResolveStub(
    ::intrinsic_proto::assets::v1::ResolvedDependency& resolved_dependency,
    const ResolveMessageOptions& options) const {
  const std::string& name = resolved_dependency.name();
  if (name.empty()) {
    return absl::OkStatus();
  }

  for (const auto& interface : options.requires_interfaces) {
    if (interface.starts_with(kGrpcUriPrefix)) {
      ::intrinsic_proto::assets::v1::ResolvedDependency::Interface
          resolved_interface;
      resolved_interface.mutable_grpc()->mutable_connection()->set_address(
          address_);
      auto* metadata = resolved_interface.mutable_grpc()
                           ->mutable_connection()
                           ->mutable_metadata()
                           ->Add();
      metadata->set_key(instance_header_name_);
      metadata->set_value(name);
      (*resolved_dependency.mutable_interfaces())[interface] =
          resolved_interface;
    } else if (interface.starts_with(kDataUriPrefix)) {
      ::intrinsic_proto::assets::v1::ResolvedDependency::Interface
          resolved_interface;
      const auto id_package = assets::PackageFrom(name);
      if (!id_package.ok()) {
        LOG(ERROR) << "Failed to create data asset ID from dependency name: "
                   << id_package.status();
        continue;
      }
      const auto id_name = assets::NameFrom(name);
      if (!id_name.ok()) {
        LOG(ERROR) << "Failed to create data asset ID from dependency name: "
                   << id_name.status();
        continue;
      }

      resolved_interface.mutable_data()->mutable_id()->set_package(*id_package);
      resolved_interface.mutable_data()->mutable_id()->set_name(*id_name);
      (*resolved_dependency.mutable_interfaces())[interface] =
          resolved_interface;
    } else {
      return absl::InvalidArgumentError(
          absl::StrCat("Unsupported dependency type: ", interface));
    }
  }
  if (options.requires_object) {
    resolved_dependency.mutable_object()->set_name(name);
  }

  resolved_dependency.clear_name();
  return absl::OkStatus();
}
absl::Status Resolver::ResolveMessage(
    google::protobuf::Message& message,
    const ResolveMessageOptions& options) const {
  // The message might be a dynamic proto so we serialize and deserialize
  // for modification. Using DynamicCastMessage would be cleaner but does
  // not work (gives nullptr) and static casting produces a segfault.
  std::string serialized = message.SerializeAsString();
  ::intrinsic_proto::assets::v1::ResolvedDependency resolved_dependency;
  if (!resolved_dependency.ParseFromString(serialized)) {
    return absl::InvalidArgumentError(
        "Failed to parse ResolvedDependency from message.");
  }
  INTR_RETURN_IF_ERROR(ResolveStub(resolved_dependency, options));
  // We clear gRPC connections if we are not resolving for execution.
  if (!options.include_connection_info) {
    for (auto& [interface, interface_dependency] :
         *resolved_dependency.mutable_interfaces()) {
      if (interface_dependency.has_grpc()) {
        interface_dependency.mutable_grpc()->clear_connection();
      }
    }
  }
  serialized = resolved_dependency.SerializeAsString();
  if (!message.ParseFromString(serialized)) {
    return absl::InvalidArgumentError(
        "Failed to parse ResolvedDependency back into message.");
  }
  return absl::OkStatus();
}

absl::StatusOr<bool> Resolver::ResolveDependency(
    google::protobuf::Message& message, executive::SkillAction skill_action,
    const google::protobuf::DescriptorPool* pool,
    google::protobuf::MessageFactory* message_factory) const {
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  message.GetReflection()->ListFields(message, &fields);
  for (const google::protobuf::FieldDescriptor* field : fields) {
    // Skip if not a message type.
    if (field->cpp_type() !=
        google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      continue;
    }

    // Handle Any type.
    const google::protobuf::FieldDescriptor* any_check_field = field;
    bool is_map = field->is_map();
    if (is_map) {
      any_check_field = field->message_type()->FindFieldByName("value");
    }

    if (any_check_field != nullptr &&
        any_check_field->cpp_type() ==
            google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE &&
        any_check_field->message_type()->full_name() ==
            google::protobuf::Any::descriptor()->full_name()) {
      const google::protobuf::FieldDescriptor* value_field =
          is_map ? any_check_field : nullptr;
      const auto status = ProcessField(
          message, field, value_field,
          [this, skill_action, pool,
           message_factory](google::protobuf::Message& msg) -> absl::Status {
            return ResolveAnyField(msg, skill_action, pool, message_factory);
          });
      if (!status.ok()) {
        LOG(ERROR) << "Failed to resolve Any field: " << status
                   << " Skipping dependency resolution for this Any proto.";
      }
      continue;
    }

    // Only process fields that are of type ResolvedDependency or a map with
    // value type ResolvedDependency.
    const google::protobuf::FieldDescriptor* value_field = nullptr;
    if (field->is_map()) {
      value_field = field->message_type()->FindFieldByName("value");
      if (value_field == nullptr ||
          value_field->cpp_type() !=
              google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE ||
          value_field->message_type()->full_name() !=
              intrinsic_proto::assets::v1::ResolvedDependency::descriptor()
                  ->full_name()) {
        continue;
      }
    } else if (field->message_type()->full_name() !=
               intrinsic_proto::assets::v1::ResolvedDependency::descriptor()
                   ->full_name()) {
      continue;
    }

    // Pull out the dependency information from the field metadata. If there is
    // no dependency information, we assume here that the user does not want to
    // treat this field as a dependency specification.
    ResolveMessageOptions resolve_message_options = {};
    const google::protobuf::FieldOptions& options = field->options();
    if (options.HasExtension(intrinsic_proto::assets::field_metadata)) {
      const intrinsic_proto::assets::FieldMetadata& field_metadata =
          options.GetExtension(intrinsic_proto::assets::field_metadata);
      resolve_message_options.requires_interfaces.insert(
          resolve_message_options.requires_interfaces.end(),
          field_metadata.dependency().requires_().begin(),
          field_metadata.dependency().requires_().end());
      resolve_message_options.requires_object =
          field_metadata.dependency().has_requires_object();
      if (field_metadata.dependency().has_skill_annotations()) {
        resolve_message_options.include_connection_info =
            field_metadata.dependency()
                .skill_annotations()
                .always_provide_connection_info();
      }
      if (!field_metadata.has_dependency()) {
        // Since there is no dependency annotation, we assume here that the
        // user does not want to treat this field as a dependency
        // specification.
        continue;
      }
    } else {
      // Since there is no field metadata, we assume here that the user does
      // not want to treat this field as a dependency specification.
      continue;
    }

    // We always include connection info during skill execution.
    if (skill_action == executive::SkillAction::Execution) {
      resolve_message_options.include_connection_info = true;
    }

    // Resolve the dependency information for repeated fields, maps fields, and
    // message fields.
    INTR_RETURN_IF_ERROR(
        ProcessField(message, field, value_field,
                     [this, &resolve_message_options](
                         google::protobuf::Message& msg) -> absl::Status {
                       return ResolveMessage(msg, resolve_message_options);
                     }));
  }

  // Recurse into submessages.
  return true;
}

absl::Status Resolver::PopulateFallbackValues(
    google::protobuf::Message& parameter,
    const absl::flat_hash_map<std::string, std::string>&
        fallback_manifest_dependencies) const {
  // Note that we do not support fallback dependencies for nested fields.
  for (int i = 0; i < parameter.GetDescriptor()->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field =
        parameter.GetDescriptor()->field(i);
    // Skip if not a message type.
    if (field->cpp_type() !=
        google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      continue;
    }
    // We do not support fallback values for repeated or map fields.
    if (field->is_map() || field->is_repeated()) {
      continue;
    }
    if (field->message_type()->full_name() !=
        intrinsic_proto::assets::v1::ResolvedDependency::descriptor()
            ->full_name()) {
      continue;
    }

    const google::protobuf::FieldOptions& options = field->options();
    if (options.HasExtension(intrinsic_proto::assets::field_metadata)) {
      const intrinsic_proto::assets::FieldMetadata& field_metadata =
          options.GetExtension(intrinsic_proto::assets::field_metadata);
      if (field_metadata.dependency()
              .skill_annotations()
              .has_fallback_manifest_dependency_key()) {
        const std::string fallback_manifest_dependency_key =
            field_metadata.dependency()
                .skill_annotations()
                .fallback_manifest_dependency_key();
        auto it = fallback_manifest_dependencies.find(
            fallback_manifest_dependency_key);
        if (it != fallback_manifest_dependencies.end()) {
          // The message might be a dynamic proto so we serialize and
          // deserialize for modification. Using DynamicCastMessage would be
          // cleaner but does not work (gives nullptr) and static casting
          // produces a segfault.
          google::protobuf::Message& message =
              *parameter.GetReflection()->MutableMessage(&parameter, field);
          std::string serialized = message.SerializeAsString();
          ::intrinsic_proto::assets::v1::ResolvedDependency resolved_dependency;
          if (!resolved_dependency.ParseFromString(serialized)) {
            return absl::InvalidArgumentError(
                "Failed to parse ResolvedDependency from message.");
          }
          if (resolved_dependency.name().empty()) {
            resolved_dependency.set_name(it->second);
          }
          serialized = resolved_dependency.SerializeAsString();
          if (!message.ParseFromString(serialized)) {
            return absl::InvalidArgumentError(
                "Failed to parse ResolvedDependency back into message.");
          }
        }
      }
    }
  }
  return absl::OkStatus();
}

absl::Status Resolver::ResolveParameterDependencies(
    google::protobuf::Message& parameter, executive::SkillAction skill_action,
    const google::protobuf::DescriptorPool* pool,
    google::protobuf::MessageFactory* message_factory) const {
  return ResolveParameterDependenciesWithFallback(parameter, skill_action, {},
                                                  pool, message_factory);
}

absl::Status Resolver::ResolveParameterDependenciesWithFallback(
    google::protobuf::Message& parameter, executive::SkillAction skill_action,
    const absl::flat_hash_map<std::string, std::string>&
        fallback_manifest_dependencies,
    const google::protobuf::DescriptorPool* pool,
    google::protobuf::MessageFactory* message_factory) const {
  INTR_RETURN_IF_ERROR(
      PopulateFallbackValues(parameter, fallback_manifest_dependencies));

  // Use the recursively walk message  utility to recursively resolve
  // dependencies in the parameter proto.
  return RecursivelyWalkMessage(
      parameter,
      [this, fallback_manifest_dependencies, skill_action, pool,
       message_factory](
          google::protobuf::Message& msg) -> absl::StatusOr<bool> {
        return this->ResolveDependency(msg, skill_action, pool,
                                       message_factory);
      });
}

}  // namespace assets
}  // namespace intrinsic
