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

#include "intrinsic/perception/cameras/sensor_pixel_size/sensor_pixel_size.h"

#include <optional>
#include <string_view>

#include "intrinsic/perception/cameras/sensor_pixel_size/sensor_pixel_sizes.h"
#include "intrinsic/perception/core/range_tools.h"

namespace intrinsic::perception {

namespace {
const absl::flat_hash_map<std::string_view, double>& SensorPixelSizeByModel(
    std::string_view vendor_name) {
  const auto& all_sizes = SensorPixelSizes();
  const auto it = all_sizes.find(vendor_name);
  if (it != all_sizes.end()) {
    return it->second;
  }
  static const auto& empty =
      *new absl::flat_hash_map<std::string_view, double>{};
  return empty;
}
}  // namespace

std::optional<double> SensorPixelSize(std::string_view vendor_name,
                                      std::string_view model_name) {
  return OptionalCopyAt(SensorPixelSizeByModel(vendor_name), model_name);
}

}  // namespace intrinsic::perception
