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

#ifndef INTRINSIC_ASSETS_DEPENDENCIES_RESOLVER_H_
#define INTRINSIC_ASSETS_DEPENDENCIES_RESOLVER_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/executive/engine/skill_action.h"

namespace intrinsic {
namespace assets {

// Resolver is a utility class for resolving dependencies in an asset
// configuration or parameter proto.
class Resolver {
 public:
  Resolver(absl::string_view address, absl::string_view instance_header_name)
      : address_(address), instance_header_name_(instance_header_name) {}

  // Resolves dependencies in the given parameter proto.
  //
  // For Skills in particular, if the skill is in Execution mode, the parameter
  // proto is checked to see if `always_provide_connection_info` is set to true.
  // Otherwise, the resolved parameter proto will have its connection info
  // cleared.
  absl::Status ResolveParameterDependencies(
      google::protobuf::Message& parameter, executive::SkillAction skill_action,
      const google::protobuf::DescriptorPool* pool,
      google::protobuf::MessageFactory* message_factory) const;

  // Similar to ResolveParameterDependencies, but allows for fallback values
  // to be provided for dependencies that are not present or have an empty name.
  // When a key in the fallback map matches a ResolvedDependency that has been
  // annotated with the fallback_manifest_dependency_key, the corresponding
  // value is used to populate the name field. This fallback only applies to
  // ResolvedDependency messages that are not part of a map, repeated field, or
  // included as a submessage.
  absl::Status ResolveParameterDependenciesWithFallback(
      google::protobuf::Message& parameter, executive::SkillAction skill_action,
      const absl::flat_hash_map<std::string, std::string>&
          fallback_manifest_dependencies,
      const google::protobuf::DescriptorPool* pool,
      google::protobuf::MessageFactory* message_factory) const;

 private:
  struct ResolveMessageOptions {
    std::vector<std::string> requires_interfaces;
    bool requires_object;
    bool include_connection_info;
  };

  absl::Status ResolveStub(
      ::intrinsic_proto::assets::v1::ResolvedDependency& resolved_dependency,
      const ResolveMessageOptions& options) const;

  absl::Status ResolveMessage(google::protobuf::Message& message,
                              const ResolveMessageOptions& options) const;

  absl::StatusOr<bool> ResolveDependency(
      google::protobuf::Message& message, executive::SkillAction skill_action,
      const google::protobuf::DescriptorPool* pool,
      google::protobuf::MessageFactory* message_factory) const;

  absl::Status PopulateFallbackValues(
      google::protobuf::Message& parameter,
      const absl::flat_hash_map<std::string, std::string>&
          fallback_manifest_dependencies) const;

  absl::Status ResolveAnyField(
      google::protobuf::Message& any_message,
      executive::SkillAction skill_action,
      const google::protobuf::DescriptorPool* pool,
      google::protobuf::MessageFactory* message_factory) const;

  std::string address_;
  std::string instance_header_name_;
};

}  // namespace assets
}  // namespace intrinsic

#endif  // INTRINSIC_ASSETS_DEPENDENCIES_RESOLVER_H_
