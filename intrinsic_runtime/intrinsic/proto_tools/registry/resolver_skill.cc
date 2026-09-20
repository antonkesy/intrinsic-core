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

#include "intrinsic_runtime/intrinsic/proto_tools/registry/resolver_skill.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "google/protobuf/descriptor.pb.h"
#include "intrinsic/skills/internal/skill_registry_client_interface.h"
#include "intrinsic/skills/proto/skills.pb.h"
#include "intrinsic/util/proto/parsed_type_url.h"
#include "intrinsic/util/proto/type_url.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_specs.h"

namespace intrinsic::proto_registry {

std::unique_ptr<SkillRegistryResolver> SkillRegistryResolver::Create(
    std::shared_ptr<skills::SkillRegistryClientInterface> client) {
  return absl::WrapUnique(new SkillRegistryResolver(client));
}

absl::StatusOr<google::protobuf::FileDescriptorSet>
SkillRegistryResolver::Resolve(const ParsedUrl& parsed_url) {
  if (parsed_url.path.empty()) {
    return CreateStatus(12001, "Skill type URL is missing skill ID (path).",
                        absl::StatusCode::kInvalidArgument);
  }

  std::vector<std::string> path_parts =
      absl::StrSplit(parsed_url.path, kTypeUrlSeparator);

  // Ignore version in type URL if present to support legacy type URLs which
  // used to have a version.
  if (path_parts.size() != 1 && path_parts.size() != 2) {
    return CreateStatus(
        12001,
        absl::StrFormat("Skill type URL '%s' has %zu parts, 1 or 2 parts "
                        "expected (<id> or <id>/<versions>)",
                        parsed_url.type_url, path_parts.size()),
        absl::StatusCode::kInvalidArgument);
  }
  const std::string skill_id = std::move(path_parts[0]);

  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::skills::Skill skill, client_->GetSkillById(skill_id),
      std::move(_).With(AttachExtendedStatus(
          12100,
          absl::StrFormat(
              "Failed to get skill information for skill '%s' for type URL %s",
              skill_id, parsed_url.type_url))));

  return skill.parameter_description().parameter_descriptor_fileset();
}

}  // namespace intrinsic::proto_registry
