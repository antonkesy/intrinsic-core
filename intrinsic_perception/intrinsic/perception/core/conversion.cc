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

#include "intrinsic/perception/core/conversion.h"

#include <algorithm>
#include <limits>

#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/operators.h"

namespace intrinsic::perception {

Image<Rgb8u> ConvertGray32fToRgb8uImage(const Image<Gray32f>& input) {
  const Image<Gray8u> tmp = ConvertGray32fToGray8uImage(input);
  return ConvertGray8uToRgb8uImage(tmp);
}

Image<Gray8u> ConvertGray32fToGray8uImage(const Image<Gray32f>& input) {
  const auto input_generic32f = Transform<Generic32f>(
      input,
      [](const Gray32f::PixelType& pix) { return Generic32f::PixelType(pix); });
  return ConvertGeneric32fToGray8uImage(input_generic32f);
}

Image<Gray8u> ConvertGeneric32fToGray8uImage(const Image<Generic32f>& input) {
  float min_gray_value = std::numeric_limits<float>::max();
  float max_gray_value = std::numeric_limits<float>::lowest();
  for (const auto& pixel : input) {
    min_gray_value = std::min(pixel, min_gray_value);
    max_gray_value = std::max(pixel, max_gray_value);
  }
  if (max_gray_value <= min_gray_value) {
    return Image<Gray8u>(input.dimensions(), Gray8u::PixelType(0));
  }
  // Keep min and max for normalized input.
  if (min_gray_value >= 0.0f && max_gray_value <= 1.0f) {
    min_gray_value = 0.0f;
    max_gray_value = 1.0f;
  }
  return Transform<Gray8u>(input, [min_gray_value, max_gray_value](
                                      const Generic32f::PixelType& pix) {
    const int grayvalue_in_0_255_range = std::clamp(
        static_cast<int>(0.5f + 255.0f * (pix - min_gray_value) /
                                    (max_gray_value - min_gray_value)),
        0, 255);
    return Gray8u::PixelType(grayvalue_in_0_255_range);
  });
}

Image<Rgb8u> ConvertGray8uToRgb8uImage(const Image<Gray8u>& input) {
  return Transform<Rgb8u>(input, [](const Gray8u::PixelType& pix) {
    return Rgb8u::PixelType(pix, pix, pix);
  });
}

Image<Gray8u> ConvertRgb8uToGray8u(const Image<Rgb8u>& input, int channel) {
  return Transform<Gray8u>(input, [channel](const Rgb8u::PixelType& pix) {
    return Gray8u::PixelType(pix[channel]);
  });
}

Image<Gray8u> ConvertRgb8uToGray8u(const Image<Rgb8u>& input) {
  return Transform<Gray8u>(input, [](const Rgb8u::PixelType& pix) {
    const float r = static_cast<float>(pix[0]);
    const float g = static_cast<float>(pix[1]);
    const float b = static_cast<float>(pix[2]);
    const int grayvalue_in_0_255_range = std::clamp(
        static_cast<int>(0.5f + 0.299f * r + 0.587f * g + 0.114f * b), 0, 255);
    return Gray8u::PixelType(grayvalue_in_0_255_range);
  });
}

Image<Rgb8u> ConvertRgb8uToSingleChannelOnlyImage(const Image<Rgb8u>& input,
                                                  int channel) {
  return Transform<Rgb8u>(input, [channel](const Rgb8u::PixelType& pix) {
    return Rgb8u::PixelType(pix[channel], pix[channel], pix[channel]);
  });
}

}  // namespace intrinsic::perception
