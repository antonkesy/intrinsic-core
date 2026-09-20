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

#ifndef INTRINSIC_PROTO_TOOLS_REGISTRY_PROTO_REGISTRY_SERVICE_H_
#define INTRINSIC_PROTO_TOOLS_REGISTRY_PROTO_REGISTRY_SERVICE_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <string_view>
#include <utility>

#include "absl/status/statusor.h"
#include "google/protobuf/descriptor.pb.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/file_descriptor_set_name.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver.h"
#include "intrinsic/proto_tools/proto/proto_registry.grpc.pb.h"
#include "intrinsic/proto_tools/proto/proto_registry.pb.h"

namespace intrinsic::proto_registry {

class ProtoRegistryService
    : public intrinsic_proto::proto_registry::ProtoRegistry::Service {
 public:
  ProtoRegistryService(const ProtoRegistryService&) = delete;
  ProtoRegistryService& operator=(const ProtoRegistryService&) = delete;
  ~ProtoRegistryService() override = default;

  static absl::StatusOr<std::unique_ptr<ProtoRegistryService>> CreateService(
      std::shared_ptr<ProtoResolverMap> resolver);

  void Shutdown();

  grpc::Status GetNamedFileDescriptorSet(
      grpc::ServerContext* context,
      const intrinsic_proto::proto_registry::GetNamedFileDescriptorSetRequest*
          request,
      intrinsic_proto::proto_registry::NamedFileDescriptorSet* response)
      override;

  grpc::Status DetermineCompatibility(
      grpc::ServerContext* context,
      const intrinsic_proto::proto_registry::DetermineCompatibilityRequest*
          request,
      intrinsic_proto::proto_registry::DetermineCompatibilityResponse* response)
      override;

 private:
  ProtoRegistryService(std::shared_ptr<ProtoResolverMap> resolver);

  absl::StatusOr<
      std::pair<FileDescriptorSetName, google::protobuf::FileDescriptorSet>>
  GetFileDescriptorSet(std::string_view type_url, std::string_view name,
                       std::string_view asset_id);

  std::atomic<bool> quit_ = false;
  std::shared_ptr<ProtoResolverMap> resolver_;
};

}  // namespace intrinsic::proto_registry

#endif  // INTRINSIC_PROTO_TOOLS_REGISTRY_PROTO_REGISTRY_SERVICE_H_
