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

#ifndef INTRINSIC_PERCEPTION_LOGGING_IMAGE_H_
#define INTRINSIC_PERCEPTION_LOGGING_IMAGE_H_

#include <string>
#include <string_view>

#include "absl/status/statusor.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/single_thread_executor.h"

namespace intrinsic::perception {

constexpr std::string_view kRawFramesEventSource = "perception.frames.raw";
constexpr std::string_view kAnnotatedFramesEventSource =
    "perception.frames.annotated";

// Logs an image asynchronously using the provided executor and returns the
// logged filename (blob ID).
template <class Traits>
absl::StatusOr<std::string> LogImage(
    const SingleThreadExecutor& executor, const Image<Traits>& image,
    std::string_view filename_prefix,
    const intrinsic_proto::data_logger::Context& context = {},
    std::string_view event_source = kAnnotatedFramesEventSource);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_LOGGING_IMAGE_H_
