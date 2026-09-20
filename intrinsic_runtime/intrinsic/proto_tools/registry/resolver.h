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

#ifndef INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_H_
#define INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_H_

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic/util/proto/parsed_type_url.h"

namespace intrinsic::proto_registry {

// Resolves file descriptor sets belonging to one or more Intrinsic type URL
// areas.
class Resolver {
 public:
  virtual ~Resolver() = default;

  // Returns a display name for this resolver.
  virtual std::string_view GetResolverName() const = 0;

  // Returns the areas for which this resolver can resolve file descriptor sets.
  virtual std::vector<std::string_view> GetAreas() const = 0;

  // Resolves the file descriptor set for the given type URL, assuming that the
  // area of the type URL is supported by this resolver.
  // Note: parsed_url.message_type may be empty if the resolution is performed
  // using a type URL prefix (e.g., when resolving by name/token).
  virtual absl::StatusOr<google::protobuf::FileDescriptorSet> Resolve(
      const ParsedUrl& parsed_url) = 0;
};

// Combines multiple resolvers for different type URL areas.
class ProtoResolverMap {
 public:
  ProtoResolverMap() = delete;
  ProtoResolverMap(const ProtoResolverMap&) = delete;
  static absl::StatusOr<std::unique_ptr<ProtoResolverMap>> Create(
      std::vector<std::shared_ptr<Resolver>>&& resolvers);

  // Resolves the file descriptor set for the given type URL, assuming that the
  // area of the type URL is supported by one of the resolvers in this map.
  // Note: parsed_url.message_type may be empty if the resolution is performed
  // using a type URL prefix.
  absl::StatusOr<google::protobuf::FileDescriptorSet> Resolve(
      const ParsedUrl& parsed_url);

 private:
  explicit ProtoResolverMap(
      absl::flat_hash_map<std::string, std::shared_ptr<Resolver>>&& resolvers)
      : resolvers_(std::move(resolvers)) {}

  absl::flat_hash_map<std::string, std::shared_ptr<Resolver>> resolvers_;
};

}  // namespace intrinsic::proto_registry

#endif  // INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_H_
