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

#include "intrinsic/perception/logging/sensor_image.h"

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/perception/cameras/sensor_image.h"
#include "intrinsic/perception/core/single_thread_executor.h"
#include "intrinsic/perception/logging/image.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

absl::StatusOr<std::string> LogSensorImage(
    const SingleThreadExecutor& executor, const SensorImage& sensor_image,
    std::string_view prefix,
    const intrinsic_proto::data_logger::Context& context,
    std::string_view event_source) {
  const std::string filename_prefix = absl::StrCat(
      prefix,
      prefix.empty() || prefix.ends_with("/") || prefix.ends_with("_") ? ""
                                                                       : "_",
      sensor_image.sensor_id(), "_",
      absl::ToUnixMillis(sensor_image.acquisition_time()));

  if (sensor_image.gray8u().has_value()) {
    return LogImage(executor, *sensor_image.gray8u(), filename_prefix, context,
                    event_source);
  } else if (sensor_image.rgb8u().has_value()) {
    return LogImage(executor, *sensor_image.rgb8u(), filename_prefix, context,
                    event_source);
  } else if (sensor_image.gray32f().has_value()) {
    return LogImage(executor, *sensor_image.gray32f(), filename_prefix, context,
                    event_source);
  } else if (sensor_image.depth32f().has_value()) {
    return LogImage(executor, *sensor_image.depth32f(), filename_prefix,
                    context, event_source);
  } else if (sensor_image.point32f().has_value()) {
    return LogImage(executor, *sensor_image.point32f(), filename_prefix,
                    context, event_source);
  } else if (sensor_image.normal32f().has_value()) {
    return LogImage(executor, *sensor_image.normal32f(), filename_prefix,
                    context, event_source);
  } else {
    return absl::InvalidArgumentError("No image found in sensor image.");
  }
}

absl::StatusOr<std::vector<std::string>> LogSensorImages(
    const SingleThreadExecutor& executor,
    absl::Span<const SensorImage> sensor_images, std::string_view prefix,
    const intrinsic_proto::data_logger::Context& context,
    std::string_view event_source) {
  std::vector<std::string> filenames;
  filenames.reserve(sensor_images.size());
  for (const SensorImage& sensor_image : sensor_images) {
    INTR_ASSIGN_OR_RETURN(
        std::string filename,
        LogSensorImage(executor, sensor_image, prefix, context, event_source));
    filenames.push_back(std::move(filename));
  }
  return filenames;
}

}  // namespace intrinsic::perception
