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

#include "intrinsic/perception/logging/image.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/logging/data_logger_client.h"
#include "intrinsic/logging/proto/blob.pb.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/perception/core/encoding.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/single_thread_executor.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {
namespace {

template <class Traits>
constexpr std::string_view FilenameSuffix() {
  if constexpr (std::is_same_v<Traits, Gray8u>) return "_gray8u.png";
  if constexpr (std::is_same_v<Traits, Rgb8u>) return "_rgb8u.png";
  if constexpr (std::is_same_v<Traits, Gray32f>) return "_gray32f.png";
  if constexpr (std::is_same_v<Traits, Depth32f>) return "_depth32f.png";
  if constexpr (std::is_same_v<Traits, Point32f>) return "_point32f.png";
  if constexpr (std::is_same_v<Traits, Normal32f>) return "_normal32f.png";
}

}  // namespace

template <class Traits>
absl::StatusOr<std::string> LogImage(
    const SingleThreadExecutor& executor, const Image<Traits>& image,
    std::string_view filename_prefix,
    const intrinsic_proto::data_logger::Context& context,
    std::string_view event_source) {
  const std::string filename =
      absl::StrCat(filename_prefix, FilenameSuffix<Traits>());
  INTR_RETURN_IF_ERROR(
      executor.Schedule([image = image, filename = filename, context = context,
                         event_source = std::string(event_source)]() mutable {
        intrinsic_proto::data_logger::LogItem log_item;
        std::vector<uint8_t> byte_buffer;
        if (EncodeImagePng(image, &byte_buffer)) {
          intrinsic_proto::data_logger::Blob* blob =
              log_item.mutable_blob_payload();
          blob->set_data(reinterpret_cast<char*>(byte_buffer.data()),
                         byte_buffer.size());
          const std::string blob_id = filename;
          blob->set_blob_id(blob_id);
        } else {
          LOG(ERROR) << "Error while encoding image as "
                     << FilenameSuffix<Traits>();
        }
        *log_item.mutable_context() = context;
        log_item.mutable_metadata()->set_event_source(event_source);

        if (const absl::Status status =
                data_logger::LogAndAwaitResponse(std::move(log_item));
            !status.ok()) {
          LOG(ERROR) << "Failed to log image: " << status;
        }
      }));
  return filename;
}

template absl::StatusOr<std::string> LogImage<Gray8u>(
    const SingleThreadExecutor&, const Image<Gray8u>&, std::string_view,
    const intrinsic_proto::data_logger::Context&, std::string_view);
template absl::StatusOr<std::string> LogImage<Rgb8u>(
    const SingleThreadExecutor&, const Image<Rgb8u>&, std::string_view,
    const intrinsic_proto::data_logger::Context&, std::string_view);
template absl::StatusOr<std::string> LogImage<Gray32f>(
    const SingleThreadExecutor&, const Image<Gray32f>&, std::string_view,
    const intrinsic_proto::data_logger::Context&, std::string_view);
template absl::StatusOr<std::string> LogImage<Depth32f>(
    const SingleThreadExecutor&, const Image<Depth32f>&, std::string_view,
    const intrinsic_proto::data_logger::Context&, std::string_view);
template absl::StatusOr<std::string> LogImage<Point32f>(
    const SingleThreadExecutor&, const Image<Point32f>&, std::string_view,
    const intrinsic_proto::data_logger::Context&, std::string_view);
template absl::StatusOr<std::string> LogImage<Normal32f>(
    const SingleThreadExecutor&, const Image<Normal32f>&, std::string_view,
    const intrinsic_proto::data_logger::Context&, std::string_view);

}  // namespace intrinsic::perception
