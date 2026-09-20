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

#include "intrinsic/perception/core/color_gradient.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "absl/log/log.h"
#include "boost/container/flat_map.hpp"
#include "intrinsic/perception/core/eigen_types.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

namespace {

Vector4f Interpolate(const std::vector<Vector4f>& colors, float t) {
  t = std::max(0.0f, std::min(1.0f, t)) * (colors.size() - 1.0);
  float fraction = std::fmod(t, 1.0);
  int i0 = std::min(static_cast<int>(std::floor(t)),
                    static_cast<int>(colors.size()) - 1);
  int i1 = std::min(i0 + 1, static_cast<int>(colors.size()) - 1);
  return colors[i0] * (1.0f - fraction) + colors[i1] * fraction;
}

const std::vector<Vector4f>& RainbowColors() {
  static auto* colors = new std::vector<Vector4f>(
      {Vector4f(0, 0, 1, 1), Vector4f(0, 1, 1, 1), Vector4f(0, 1, 0, 1),
       Vector4f(1, 1, 0, 1), Vector4f(1, 0, 0, 1)});
  return *colors;
}

template <typename ImageTrait, int num_channels>
ColorGradientFunction<ImageTrait> CreateColorGradientFunction(
    const boost::container::flat_map<float, typename ImageTrait::PixelType>&
        colors) {
  if (colors.empty()) {
    LOG(ERROR) << "Trying to create a custom color gradient without any "
                  "colors. The returned gradient will be black.";
    return [](float t) { return ImageTrait::PixelType::Zero(); };
  }

  // Convert colors to float for quicker interpolation.
  boost::container::flat_map<float, Eigen::Vector<float, num_channels>>
      float_colors;
  float_colors.reserve(colors.size());
  for (const auto& color : colors) {
    float_colors.insert({color.first, color.second.template cast<float>()});
  }

  using ScalarType = typename ImageTrait::ScalarType;
  return [float_colors](float t) -> typename ImageTrait::PixelType {
    const auto color1 = float_colors.upper_bound(t);
    if (color1 == float_colors.end()) {
      return (float_colors.rbegin()->second).template cast<ScalarType>();
    }
    if (color1 == float_colors.begin()) {
      return (color1->second).template cast<ScalarType>();
    }
    const auto color0 = color1 - 1;
    const float interp = (t - color0->first) / (color1->first - color0->first);
    return (color0->second * (1.0f - interp) + color1->second * interp)
        .template cast<ScalarType>();
  };
}

}  // namespace

Rgba8u::PixelType RainbowColor(float t) {
  return (Interpolate(RainbowColors(), t) * 255.0f).cast<uint8_t>();
}

template <>
ColorGradientFunction<Rgba8u> CustomColorGradientFunction<Rgba8u>(
    const boost::container::flat_map<float, Rgba8u::PixelType>& colors) {
  return CreateColorGradientFunction<Rgba8u, 4>(colors);
}

template <>
ColorGradientFunction<Rgb8u> CustomColorGradientFunction<Rgb8u>(
    const boost::container::flat_map<float, Rgb8u::PixelType>& colors) {
  return CreateColorGradientFunction<Rgb8u, 3>(colors);
}

template <>
ColorGradientFunction<Rgba32f> CustomColorGradientFunction<Rgba32f>(
    const boost::container::flat_map<float, Rgba32f::PixelType>& colors) {
  return CreateColorGradientFunction<Rgba32f, 4>(colors);
}

template <>
ColorGradientFunction<Rgb32f> CustomColorGradientFunction<Rgb32f>(
    const boost::container::flat_map<float, Rgb32f::PixelType>& colors) {
  return CreateColorGradientFunction<Rgb32f, 3>(colors);
}

}  // namespace perception
}  // namespace intrinsic
