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

#ifndef INTRINSIC_PERCEPTION_CORE_CONFIG_NAME_UTILS_H_
#define INTRINSIC_PERCEPTION_CORE_CONFIG_NAME_UTILS_H_

#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

namespace intrinsic::perception {

struct PoseEstimationConfigName {
  std::string app;
  std::string workcell;
  std::string id;
};

// Parses the name of a pose estimation config into its components.
// Accepted formats:
// apps/{app}/workcell/{cell}/poseEstimators/{id}
// apps/{app}/workcell/{cell}/detectors/{id}    (legacy format).
absl::StatusOr<PoseEstimationConfigName> ParsePoseEstimationConfigName(
    absl::string_view name);

struct PatternDetectionConfigName {
  std::string app;
  std::string workcell;
  std::string id;
};

// Parses the name of a pattern detection config into its components.
// Accepted formats:
// apps/{app}/workcells/{cell}/{collection}/{id}
absl::StatusOr<PatternDetectionConfigName> ParsePatternDetectionConfigName(
    absl::string_view name);

// Returns base folder of pose estimation data within a cluster.
std::string InClusterBaseFolder(const PoseEstimationConfigName& name);

// Extracts the local prefix (app/id/) for any resource config name.
// Returns an empty string if the name is empty.
std::string ExtractPrefixFromConfigName(absl::string_view name);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_CONFIG_NAME_UTILS_H_
