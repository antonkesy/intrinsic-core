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

#ifndef INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_SKILL_H_
#define INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_SKILL_H_

#include <memory>
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver.h"
#include "intrinsic/skills/internal/skill_registry_client_interface.h"
#include "intrinsic/util/proto/parsed_type_url.h"
#include "intrinsic/util/proto/type_url.h"

namespace intrinsic::proto_registry {

// Resolver for the 'skills' area which uses the skill registry. While regular
// skills are also provided via the 'assets' area and the installed assets
// service (see AssetResolver), PBTs are only provided via the skill registry.
// As long as this is the case we need to keep this resolver and the 'skills'
// area separate.
class SkillRegistryResolver : public Resolver {
 public:
  static std::unique_ptr<SkillRegistryResolver> Create(
      std::shared_ptr<skills::SkillRegistryClientInterface> client);

  std::string_view GetResolverName() const override {
    return "SkillRegistryResolver";
  }

  std::vector<std::string_view> GetAreas() const override {
    return {kIntrinsicTypeUrlAreaSkills};
  }

  absl::StatusOr<google::protobuf::FileDescriptorSet> Resolve(
      const ParsedUrl& parsed_url) override;

 private:
  explicit SkillRegistryResolver(
      std::shared_ptr<skills::SkillRegistryClientInterface> client)
      : client_(client) {}

  std::shared_ptr<skills::SkillRegistryClientInterface> client_;
};

}  // namespace intrinsic::proto_registry

#endif  // INTRINSIC_PROTO_TOOLS_REGISTRY_RESOLVER_SKILL_H_
