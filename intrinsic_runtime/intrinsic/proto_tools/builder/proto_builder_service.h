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

#ifndef INTRINSIC_EXECUTIVE_TOOLS_PROTO_BUILDER_SERVICE_H_
#define INTRINSIC_EXECUTIVE_TOOLS_PROTO_BUILDER_SERVICE_H_

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor_database.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic_runtime/intrinsic/proto_tools/util/descriptor_pool_loader.h"
#include "intrinsic/executive/proto/proto_builder.grpc.pb.h"
#include "intrinsic/executive/proto/proto_builder.pb.h"

namespace intrinsic::executive {

class ProtoBuilderService
    : public intrinsic_proto::executive::ProtoBuilder::Service {
 public:
  ProtoBuilderService();

  grpc::Status Compile(
      grpc::ServerContext* context,
      const intrinsic_proto::executive::ProtoCompileRequest* request,
      intrinsic_proto::executive::ProtoCompileResponse* response) override;

  grpc::Status Compose(
      grpc::ServerContext* context,
      const intrinsic_proto::executive::ProtoComposeRequest* request,
      intrinsic_proto::executive::ProtoComposeResponse* response) override;

  grpc::Status GetWellKnownTypes(
      grpc::ServerContext* context,
      const intrinsic_proto::executive::GetWellKnownTypesRequest* request,
      intrinsic_proto::executive::GetWellKnownTypesResponse* response) override;

 private:
  // Registers multiple versions of the same well-known type. The first given
  // version is considered the default version.
  void RegisterWellKnownType(
      std::vector<std::pair<std::string_view, std::string_view>>
          full_names_and_display_versions);

  // Registers a a well-known type which only has a single version.
  void RegisterWellKnownType(std::string_view full_name,
                             std::string_view display_version = "") {
    return RegisterWellKnownType({{full_name, display_version}});
  }

  // Checks if there are any files in the well-known types file descriptor set
  // which are not required/used.
  void CheckWellKnownTypesContainOnlyRequiredFiles(
      const std::vector<std::string>& additional_required_files) const;

  // Ensures that descriptor only contains types from well_known_types_ at its
  // top level fields.
  absl::Status CheckContainsOnlyWellKnownTypes(
      const google::protobuf::DescriptorProto& descriptor);

  std::vector<const google::protobuf::Descriptor*> well_known_types_;
  absl::flat_hash_set<std::string> well_known_types_names_;
  std::vector<intrinsic_proto::executive::TypeWithVersions>
      well_known_types_with_versions_;
  std::unique_ptr<LoadedDescriptorPool> well_known_types_loaded_pool_;
};

}  // namespace intrinsic::executive

#endif  // INTRINSIC_EXECUTIVE_TOOLS_PROTO_BUILDER_SERVICE_H_
