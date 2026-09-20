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

#include "intrinsic/perception/calibration/pattern_detector_factory.h"

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "google/protobuf/descriptor.h"
#include "intrinsic/perception/calibration/pattern_detector_interface.h"
#include "intrinsic/perception/proto/v1/pattern_detection_config.pb.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

namespace {
absl::StatusOr<std::string> GetPatternDetectorName(
    const intrinsic_proto::perception::v1::PatternDetectionConfig::TypeCase
        pattern_detector_type) {
  const google::protobuf::Descriptor* message_descriptor =
      intrinsic_proto::perception::v1::PatternDetectionConfig::descriptor();
  INTR_RET_CHECK_NE(message_descriptor, nullptr)
      << "No pattern descriptor found.";
  const google::protobuf::FieldDescriptor* field_descriptor =
      message_descriptor->FindFieldByNumber(pattern_detector_type);
  INTR_RET_CHECK_NE(field_descriptor, nullptr)
      << "Unknown pattern detector driver.";
  return std::string(field_descriptor->name());
}

absl::StatusOr<std::unique_ptr<PatternDetectorInterface>> CreateByName(
    absl::string_view name,
    const intrinsic_proto::perception::v1::PatternDetectionConfig& config) {
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<PatternDetectorInterface> interface,
      PatternDetectorRegistry::Dispatch(std::string(name), config));
  return interface;
}

}  // namespace

absl::StatusOr<std::unique_ptr<PatternDetectorInterface>> CreatePatternDetector(
    const intrinsic_proto::perception::v1::PatternDetectionConfig& config) {
  INTR_RET_CHECK_NE(
      config.Type_case(),
      intrinsic_proto::perception::v1::PatternDetectionConfig::TYPE_NOT_SET)
          .SetCode(absl::StatusCode::kInvalidArgument)
      << "No pattern detector config specified.";

  INTR_ASSIGN_OR_RETURN(const std::string pattern_detector_alias,
                        GetPatternDetectorName(config.Type_case()));

  INTR_ASSIGN_OR_RETURN(auto detector,
                        CreateByName(pattern_detector_alias, config));
  return detector;
}

}  // namespace intrinsic::perception
