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

#ifndef INTRINSIC_PERCEPTION_CORE_IMAGE_FILE_REFERENCE_H_
#define INTRINSIC_PERCEPTION_CORE_IMAGE_FILE_REFERENCE_H_

#include <cstdint>
#include <optional>
#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/pixel_type.h"

namespace intrinsic::perception {

struct ImageFileReference {
  std::string path;
  PixelType pixel_type = PixelType::kUnspecified;
  int32_t num_channels = 0;
  std::optional<int64_t> sensor_id;
  auto operator<=>(const ImageFileReference&) const = default;
  template <typename H>
  friend H AbslHashValue(H h, const ImageFileReference& i) {
    return H::combine(std::move(h), i.path, i.pixel_type, i.num_channels,
                      i.sensor_id);
  }
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const ImageFileReference& i) {
    absl::Format(&sink, "file:%s?pixel_type=%s&num_channels=%d&sensor_id=%s",
                 i.path, absl::StrCat(i.pixel_type), i.num_channels,
                 i.sensor_id.has_value() ? absl::StrCat(i.sensor_id.value())
                                         : "nullopt");
  }
};

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_IMAGE_FILE_REFERENCE_H_
