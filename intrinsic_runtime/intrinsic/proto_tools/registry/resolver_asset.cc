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

#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver_asset.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "google/protobuf/descriptor.pb.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/id_utils.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/assets/proto/installed_assets.pb.h"
#include "intrinsic/assets/proto/view.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/proto/parsed_type_url.h"
#include "intrinsic/util/proto/type_url.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_specs.h"

namespace intrinsic::proto_registry {

namespace {

class AssetResolverImpl : public AssetResolver {
 public:
  absl::StatusOr<google::protobuf::FileDescriptorSet> Resolve(
      const ParsedUrl& parsed_url) override;

  explicit AssetResolverImpl(
      std::shared_ptr<
          intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface>
          installed_assets_stub)
      : installed_assets_stub_(std::move(installed_assets_stub)) {}

 private:
  std::shared_ptr<
      intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface>
      installed_assets_stub_;
};

absl::StatusOr<intrinsic_proto::assets::v1::InstalledAsset>
CallGetInstalledAsset(
    std::string_view asset_id,
    intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface&
        installed_assets_stub,
    std::string_view retrieval_context) {
  INTR_ASSIGN_OR_RETURN(
      std::string asset_name, assets::NameFrom(asset_id),
      std::move(_).With(AttachExtendedStatus(
          12001,
          absl::StrFormat("Failed to get asset name from '%s'", asset_id))));
  INTR_ASSIGN_OR_RETURN(
      std::string asset_package, assets::PackageFrom(asset_id),
      std::move(_).With(AttachExtendedStatus(
          12001,
          absl::StrFormat("Failed to get asset package from '%s'", asset_id))));

  grpc::ClientContext context;
  ConfigureClientContext(&context);
  intrinsic_proto::assets::v1::GetInstalledAssetRequest request;
  request.mutable_id()->set_package(asset_package);
  request.mutable_id()->set_name(asset_name);
  request.set_view(intrinsic_proto::catalog::ASSET_VIEW_TYPE_ALL_METADATA);
  intrinsic_proto::assets::v1::InstalledAsset asset;

  INTR_RETURN_IF_ERROR(ToAbslStatus(installed_assets_stub.GetInstalledAsset(
                           &context, request, &asset)))
      .With(AttachExtendedStatus(
          12100, absl::StrFormat("Failed to get metadata for asset '%s' %s",
                                 asset_id, retrieval_context)));

  return asset;
}

}  // namespace

std::unique_ptr<AssetResolver> AssetResolver::Create(
    std::shared_ptr<
        intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface>
        installed_assets_stub) {
  return std::make_unique<AssetResolverImpl>(std::move(installed_assets_stub));
}

absl::StatusOr<google::protobuf::FileDescriptorSet> AssetResolverImpl::Resolve(
    const ParsedUrl& parsed_url) {
  if (parsed_url.path.empty()) {
    return CreateStatus(12001, "Asset type URL is missing asset ID (path).",
                        absl::StatusCode::kInvalidArgument);
  }

  std::vector<std::string> path_parts =
      absl::StrSplit(parsed_url.path, kTypeUrlSeparator);

  // Ignore version in type URL if present to support legacy type URLs which
  // used to have a version.
  if (path_parts.size() != 1 && path_parts.size() != 2) {
    return CreateStatus(
        12001,
        absl::StrFormat("Asset type URL '%s' has %zu parts, 1 or 2 parts "
                        "expected (<id> or <id>/<versions>)",
                        parsed_url.type_url, path_parts.size()),
        absl::StatusCode::kInvalidArgument);
  }
  const std::string asset_id = std::move(path_parts[0]);

  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::assets::v1::InstalledAsset asset,
      CallGetInstalledAsset(
          asset_id, *installed_assets_stub_,
          absl::StrFormat("for type URL %s", parsed_url.type_url)));

  // TODO(b/530915170): Return only the relevant part of the descriptor set when
  // a specific message or enum was requested.
  return asset.metadata().file_descriptor_set();
}

}  // namespace intrinsic::proto_registry
