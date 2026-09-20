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

#ifndef INTRINSIC_PERCEPTION_LOGGING_PATTERN_DETECTION_H_
#define INTRINSIC_PERCEPTION_LOGGING_PATTERN_DETECTION_H_

#include <string_view>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/single_thread_executor.h"
#include "intrinsic/perception/proto/v1/pattern_detection_result.pb.h"

namespace intrinsic::perception {

// Logs pattern detection annotated images asynchronously to the data logger.
absl::Status LogResult(
    const SingleThreadExecutor& executor,
    absl::Span<const SensorImage> sensor_images,
    const intrinsic_proto::perception::v1::PatternDetectionResult& result,
    std::string_view prefix = "");

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_LOGGING_PATTERN_DETECTION_H_
