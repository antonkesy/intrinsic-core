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

#ifndef INTRINSIC_PERCEPTION_LOGGING_SENSOR_IMAGE_H_
#define INTRINSIC_PERCEPTION_LOGGING_SENSOR_IMAGE_H_

#include <string>
#include <string_view>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/single_thread_executor.h"
#include "intrinsic/perception/logging/image.h"

namespace intrinsic::perception {

// Logs a single sensor image asynchronously to the data logger.
absl::StatusOr<std::string> LogSensorImage(
    const SingleThreadExecutor& executor, const SensorImage& sensor_image,
    std::string_view prefix = "",
    const intrinsic_proto::data_logger::Context& context = {},
    std::string_view event_source = kRawFramesEventSource);

// Logs a collection of sensor images asynchronously to the data logger.
absl::StatusOr<std::vector<std::string>> LogSensorImages(
    const SingleThreadExecutor& executor,
    absl::Span<const SensorImage> sensor_images, std::string_view prefix = "",
    const intrinsic_proto::data_logger::Context& context = {},
    std::string_view event_source = kRawFramesEventSource);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_LOGGING_SENSOR_IMAGE_H_
