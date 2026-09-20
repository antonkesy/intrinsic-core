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

#ifndef INTRINSIC_PERCEPTION_CORE_CONVERSION_H_
#define INTRINSIC_PERCEPTION_CORE_CONVERSION_H_

#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic::perception {

// Converts a Gray32f image in an Rgb8u image, auto-scaling the range of gray
// values to the rgb range 0-255.
Image<Rgb8u> ConvertGray32fToRgb8uImage(const Image<Gray32f>& input);

// Converts a Gray32f image in a Gray8u image, auto-scaling the range of gray
// values to the range 0-255.
Image<Gray8u> ConvertGray32fToGray8uImage(const Image<Gray32f>& input);

// Converts an image of type Generic32f (which can have negative values or
// values over 1.0) to a Gray8u image, auto-scaling the values to the output
// range 0-255.
Image<Gray8u> ConvertGeneric32fToGray8uImage(const Image<Generic32f>& input);

// Converts a Gray8u image in an Rgb8u image, by using the same values for all
// channels.
Image<Rgb8u> ConvertGray8uToRgb8uImage(const Image<Gray8u>& input);

Image<Gray8u> ConvertRgb8uToGray8u(const Image<Rgb8u>& input, int channel);

// Blends RGB values using a different weight for each channel.
Image<Gray8u> ConvertRgb8uToGray8u(const Image<Rgb8u>& input);

// Converts an Rgb8u image to a single channel Rgb8u image.
Image<Rgb8u> ConvertRgb8uToSingleChannelOnlyImage(const Image<Rgb8u>& input,
                                                  int channel);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CORE_CONVERSION_H_
