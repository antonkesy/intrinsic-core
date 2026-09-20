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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_PATTERN_DETECTOR_INTERFACE_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_PATTERN_DETECTOR_INTERFACE_H_

#include <memory>
#include <string>

#include "absl/base/no_destructor.h"
#include "absl/status/statusor.h"
#include "cppregpattern/registry.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/proto/v1/pattern_detection_config.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"

namespace intrinsic::perception {

// Interface for pattern detection implementations within perception which is
// exposed to clients of perception.
class PatternDetectorInterface {
 public:
  virtual ~PatternDetectorInterface() = default;

  // Pattern detectors are not copyable but move only.
  PatternDetectorInterface(PatternDetectorInterface&& other) = default;
  PatternDetectorInterface& operator=(PatternDetectorInterface&& other) =
      default;
  PatternDetectorInterface(const PatternDetectorInterface&) = delete;
  PatternDetectorInterface& operator=(const PatternDetectorInterface&) = delete;

  // Returns all detection results for each detected target.
  // Targets are configured in the detector config during the detector creation.
  absl::StatusOr<intrinsic_proto::perception::v1::PatternDetectionResult> Run(
      const CaptureResult& capture_result);

 protected:
  PatternDetectorInterface() = default;

  // Run the actual detector on the provides camera image(s). Detectors need to
  // overwrite this method with the actual implementation.
  virtual absl::StatusOr<
      intrinsic_proto::perception::v1::PatternDetectionResult>
  GetPatternDetectionsImpl(const CaptureResult& capture_result) const = 0;
};

// Registration mechanism for factories that create the pattern detectors.
using PatternDetectorRegistry = registry::Registry<
    std::string,
    absl::StatusOr<std::unique_ptr<PatternDetectorInterface>>(
        const intrinsic_proto::perception::v1::PatternDetectionConfig&),
    registry::MissingKeyPolicy::default_construct>;

#define REGISTER_PATTERN_DETECTOR_INTERFACE(name, alias, FactoryName)       \
  [[maybe_unused]] const bool kUnused##name =                               \
      intrinsic::perception::PatternDetectorRegistry::Register(             \
          alias,                                                            \
          [](const intrinsic_proto::perception::v1::PatternDetectionConfig& \
                 config) { return FactoryName(config); });

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_PATTERN_DETECTOR_INTERFACE_H_
