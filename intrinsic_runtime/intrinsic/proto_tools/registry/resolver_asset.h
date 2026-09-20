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

#ifndef INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_ASSET_H_
#define INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_ASSET_H_

#include <memory>
#include <string_view>
#include <vector>

#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/util/proto/type_url.h"

namespace intrinsic::proto_registry {

class AssetResolver : public Resolver {
 public:
  static std::unique_ptr<AssetResolver> Create(
      std::shared_ptr<
          intrinsic_proto::assets::v1::InstalledAssetsReader::StubInterface>
          installed_assets_stub);

  std::string_view GetResolverName() const override { return "AssetResolver"; }

  std::vector<std::string_view> GetAreas() const override {
    return {kIntrinsicTypeUrlAreaAssets};
  }
};

}  // namespace intrinsic::proto_registry

#endif  // INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_ASSET_H_
