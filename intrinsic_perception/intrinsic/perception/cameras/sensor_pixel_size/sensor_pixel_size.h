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


#ifndef INTRINSIC_PERCEPTION_CAMERAS_SENSOR_PIXEL_SIZE_SENSOR_PIXEL_SIZE_H_
#define INTRINSIC_PERCEPTION_CAMERAS_SENSOR_PIXEL_SIZE_SENSOR_PIXEL_SIZE_H_

#include <optional>
#include <string_view>

namespace intrinsic::perception {

std::optional<double> SensorPixelSize(std::string_view vendor_name,
                                      std::string_view model_name);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_SENSOR_PIXEL_SIZE_SENSOR_PIXEL_SIZE_H_
