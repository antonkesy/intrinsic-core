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

#include "intrinsic/logging/utils/logger_labels.h"

#include <cstdint>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/logging/proto/context.pb.h"

namespace intrinsic {

absl::Status AddSkillLogIdLabel(
    intrinsic_proto::data_logger::Context& context) {
  const uint64_t skill_id = context.skill_id();
  // The skill id in the context is not optional, so we do not add the label if
  // the skill id is zero.
  if (skill_id == 0) {
    return absl::OkStatus();
  }
  auto [it, inserted] = context.mutable_labels()->insert(
      {kSkillIdLabel, std::to_string(skill_id)});
  if (!inserted && it->second != std::to_string(skill_id)) {
    return absl::AlreadyExistsError(
        absl::StrCat("Label ", kSkillIdLabel, " already exists and has value ",
                     it->second, " but skill id is ", skill_id, "."));
  }

  return absl::OkStatus();
}

}  // namespace intrinsic
