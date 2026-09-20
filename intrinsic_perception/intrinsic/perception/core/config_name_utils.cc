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

#include "intrinsic/perception/core/config_name_utils.h"

#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "ortools/base/path.h"
#include "re2/re2.h"

namespace intrinsic::perception {

absl::StatusOr<PoseEstimationConfigName> ParsePoseEstimationConfigName(
    const absl::string_view name) {
  // LINT.IfChange
  constexpr char kNameRegex[] =
      R"(apps\/([\w_-]+)\/workcells\/([\w_-]+)\/poseEstimators\/([\w_-]+))";
  // LINT.ThenChange(//intrinsic/frontend/onprem/perception/common/pose_estimation_util.ts)
  PoseEstimationConfigName parsed_name;
  bool matched = RE2::FullMatch(name, kNameRegex, &parsed_name.app,
                                &parsed_name.workcell, &parsed_name.id);
  if (!matched) {
    return InvalidArgumentErrorBuilder()
           << "Couldn't match the pose estimation config name '" << name
           << "' to pattern: "
              "'apps/{app}/workcells/{workcell}/poseEstimators/{id}'";
  }
  return parsed_name;
}

absl::StatusOr<PatternDetectionConfigName> ParsePatternDetectionConfigName(
    const absl::string_view name) {
  constexpr char kNameRegex[] =
      R"(apps\/([\w_-]+)\/workcells\/([\w_-]+)\/[\w_-]+\/([\w_-]+))";
  PatternDetectionConfigName parsed_name;
  bool matched = RE2::FullMatch(name, kNameRegex, &parsed_name.app,
                                &parsed_name.workcell, &parsed_name.id);
  if (!matched) {
    return InvalidArgumentErrorBuilder()
           << "Couldn't match the pattern detection config name '" << name
           << "' to pattern: "
              "'apps/{app}/workcells/{workcell}/{collection}/{id}'";
  }
  return parsed_name;
}

std::string GcsBaseFolder(const PoseEstimationConfigName& name) {
  return file::JoinPath("apps", name.app, "workcells", name.workcell,
                        "poseEstimators", name.id);
}

std::string InClusterBaseFolder(const PoseEstimationConfigName& name) {
  return file::JoinPath("/poseEstimators", name.id);
}

std::string ExtractPrefixFromConfigName(const absl::string_view name) {
  if (name.empty()) {
    return "";
  }
  if (auto parsed_pose = ParsePoseEstimationConfigName(name);
      parsed_pose.ok()) {
    return absl::StrCat(file::JoinPath(parsed_pose->app, parsed_pose->id), "/");
  }
  if (auto parsed_pattern = ParsePatternDetectionConfigName(name);
      parsed_pattern.ok()) {
    return absl::StrCat(file::JoinPath(parsed_pattern->app, parsed_pattern->id),
                        "/");
  }
  return absl::StrCat(name, "/");
}

}  // namespace intrinsic::perception
