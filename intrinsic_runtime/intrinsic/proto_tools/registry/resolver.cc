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

#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic/util/proto/parsed_type_url.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_specs.h"

namespace intrinsic::proto_registry {

absl::StatusOr<std::unique_ptr<ProtoResolverMap>> ProtoResolverMap::Create(
    std::vector<std::shared_ptr<Resolver>>&& resolvers) {
  absl::flat_hash_map<std::string, std::shared_ptr<Resolver>> resolver_map;

  for (const std::shared_ptr<Resolver>& resolver : resolvers) {
    std::string resolver_name(resolver->GetResolverName());
    std::vector<std::string_view> areas = resolver->GetAreas();

    for (const std::string_view area : areas) {
      auto [iterator, inserted] =
          resolver_map.insert({std::string(area), resolver});

      if (!inserted) {
        return absl::InvalidArgumentError(
            absl::StrFormat("Resolver for area '%s' already registered, failed "
                            "to register resolver '%s'",
                            area, resolver_name));
      }
    }
  }

  return absl::WrapUnique(new ProtoResolverMap(std::move(resolver_map)));
}

absl::StatusOr<google::protobuf::FileDescriptorSet> ProtoResolverMap::Resolve(
    const ParsedUrl& parsed_url) {
  if (const auto it = resolvers_.find(parsed_url.area);
      it != resolvers_.end()) {
    Resolver& resolver = *it->second;
    return resolver.Resolve(parsed_url);
  } else {
    return CreateStatus(
        12001,
        absl::StrFormat("Type URL '%s' has unsupported area '%s'",
                        parsed_url.type_url, parsed_url.area),
        /*generic_code=*/absl::StatusCode::kNotFound);
  }
}

}  // namespace intrinsic::proto_registry
