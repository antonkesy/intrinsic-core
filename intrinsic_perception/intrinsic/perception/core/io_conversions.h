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

#ifndef INTRINSIC_PERCEPTION_CORE_IO_CONVERSIONS_H_
#define INTRINSIC_PERCEPTION_CORE_IO_CONVERSIONS_H_

#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/core/operators.h"

namespace intrinsic {
namespace perception {

// Converts between different pixel types for image io.
//
// Supportes pixel conversions from various pixel types to rgb (or equivalent).
// Note that this conversion focuses on usage for io, so the conversion asserts
// read/write compatibility for various pixel types, but the resulting
// conversion is likely not optimal for image display.
template <class OutputImageTrait, class InputImageTrait>
typename OutputImageTrait::PixelType PixelConvert(
    const typename InputImageTrait::PixelType& pix);

// Converts between different image types for io.
//
// Supportes image conversions from various image types to rgb (or equivalent).
// Note that this conversion focuses on usage for io, so the conversion asserts
// read/write compatibility for various image types, but the resulting
// conversion is likely not optimal for image display.
template <class OutputImageTrait, class InputImageTrait>
Image<OutputImageTrait> ConvertImage(const Image<InputImageTrait>& input) {
  return Transform<OutputImageTrait>(
      input, PixelConvert<OutputImageTrait, InputImageTrait>);
}

// Converts a channel of a point (e.g. x position for channel 0) into an Rgb8u.
Rgb8u::PixelType PointConvert(const Point32f::PixelType& pix, int channel);

// Converts 3 channels of a point (corresponding to the x, y and z coordinate)
// into a point.
Point32f::PixelType PointConvert(const Rgb8u::PixelType& x,
                                 const Rgb8u::PixelType& y,
                                 const Rgb8u::PixelType& z);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_IO_CONVERSIONS_H_
