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

#ifndef INTRINSIC_PERCEPTION_CALIBRATION_PATTERN_DETECTOR_H_
#define INTRINSIC_PERCEPTION_CALIBRATION_PATTERN_DETECTOR_H_

#include <memory>

#include "absl/status/statusor.h"
#include "grpcpp/server_context.h"
#include "intrinsic/perception/calibration/pattern_detector_interface.h"
#include "intrinsic/perception/cameras/capture_result.h"
#include "intrinsic/perception/core/single_thread_executor.h"
#include "intrinsic/perception/proto/v1/pattern_detection_config.pb.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"

namespace intrinsic::perception {

class PatternDetector final {
 public:
  static absl::StatusOr<PatternDetector> Create(
      const intrinsic_proto::perception::v1::PatternDetectionConfig&
          pattern_detection_config);

  explicit PatternDetector(
      std::unique_ptr<PatternDetectorInterface> pattern_detector_interface,
      const intrinsic_proto::perception::v1::PatternDetectionConfig&
          pattern_detection_config,
      std::unique_ptr<SingleThreadExecutor> executor = nullptr);

  // Detectors can be moved (copying is implicitly disabled).
  PatternDetector(PatternDetector&&) = default;
  PatternDetector& operator=(PatternDetector&&) = default;

  absl::StatusOr<intrinsic_proto::perception::v1::PatternDetectionResult> Run(
      const CaptureResult& capture_result,
      grpc::ServerContext* context = nullptr);

 private:
  std::unique_ptr<PatternDetectorInterface> pattern_detector_interface_;
  intrinsic_proto::perception::v1::PatternDetectionConfig
      pattern_detection_config_;
  std::unique_ptr<SingleThreadExecutor> executor_;
};

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CALIBRATION_PATTERN_DETECTOR_H_
