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

#include "intrinsic/perception/calibration/pattern_detector_interface.h"

#include "absl/status/statusor.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"

namespace intrinsic::perception {

absl::StatusOr<intrinsic_proto::perception::v1::PatternDetectionResult>
PatternDetectorInterface::Run(const CaptureResult& capture_result) {
  return GetPatternDetectionsImpl(capture_result);
}

}  // namespace intrinsic::perception
