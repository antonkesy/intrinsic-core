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

#ifndef INTRINSIC_PERCEPTION_CORE_COLOR_GRADIENT_H_
#define INTRINSIC_PERCEPTION_CORE_COLOR_GRADIENT_H_

#include <functional>

#include "boost/container/flat_map.hpp"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

// Returns color value for value 't'. This function linearly interpolates
// through the rainbow colors: blue (t=0.0), cyan, green, yellow, red (t=1.0).
// Values of 't' outside the [0, 1] range are clamped.
Rgba8u::PixelType RainbowColor(float t);

// Function that maps a float value to a color.
template <typename ImageTrait>
using ColorGradientFunction =
    std::function<typename ImageTrait::PixelType(float)>;

// Creates a function to generate a color gradient. Requires value-color pairs
// as input. The resulting function finds the lower/upper colors closest to the
// input value and will linearely interpolate between those colors. If the value
// is out of range, the min respectively max color is returned.
// Example usage:
//
// const auto my_color_gradient = CustomColorGradientFunction<Rgb8u>({
//     {0.0f, {0, 0, 255}},
//     {0.1f, {0, 255, 0}},
//     {1.0f, {255, 0, 0}},
// });
// auto color0 = my_color_gradient(-1);    // {0, 0, 255}
// auto color1 = my_color_gradient(0.05f); // {0, 127, 127}
// auto color2 = my_color_gradient(0.1f);  // {0, 255, 0}
template <typename ImageTrait>
ColorGradientFunction<ImageTrait> CustomColorGradientFunction(
    const boost::container::flat_map<float, typename ImageTrait::PixelType>&
        colors);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_COLOR_GRADIENT_H_
