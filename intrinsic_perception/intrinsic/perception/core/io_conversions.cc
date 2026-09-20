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

#include "intrinsic/perception/core/io_conversions.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "absl/log/log.h"
#include "intrinsic/perception/core/image_traits.h"

namespace intrinsic {
namespace perception {

template <class OutputImageTrait, class InputImageTrait>
typename OutputImageTrait::PixelType PixelConvert(
    const typename InputImageTrait::PixelType& pix) {
  return static_cast<typename OutputImageTrait::PixelType>(pix);
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Normal32f>(
    const Normal32f::PixelType& pix) {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  if (Normal32f::IsValid(pix)) {
    r = std::max(0.0f, std::min(pix.x() * 128 + 128, 255.1f));
    g = std::max(0.0f, std::min(pix.y() * 128 + 128, 255.1f));
    b = std::max(0.0f, std::min(pix.z() * 128 + 128, 255.1f));
  }
  return {r, g, b};
}

template <>
typename Normal32f::PixelType PixelConvert<Normal32f, Bgr8u>(
    const typename Bgr8u::PixelType& pix) {
  Normal32f::PixelType normal = Normal32f::Invalid();
  if (pix.x() != 0 || pix.y() != 0 || pix.z() != 0) {
    normal.x() = (pix.z() - 128) / 128.0;
    normal.y() = (pix.y() - 128) / 128.0;
    normal.z() = (pix.x() - 128) / 128.0;
    normal.normalized();
  }
  return normal;
}

template <>
typename Normal32f::PixelType PixelConvert<Normal32f, Rgb8u>(
    const typename Rgb8u::PixelType& pix) {
  Normal32f::PixelType normal = Normal32f::Invalid();
  if (pix.x() != 0 || pix.y() != 0 || pix.z() != 0) {
    normal.x() = (pix.x() - 128) / 128.0;
    normal.y() = (pix.y() - 128) / 128.0;
    normal.z() = (pix.z() - 128) / 128.0;
    normal.normalized();
  }
  return normal;
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Depth32f>(
    const typename Depth32f::PixelType& pix) {
  // Encode mm depth into r and g channel.
  // This encoding method is purely for storing and webgl rendering.
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  if (Depth32f::IsValid(pix)) {
    uint64_t microns = std::max(0.0f, std::min(65.535f, pix)) * 1000000.0;
    uint64_t mm = microns / 1000;
    r = mm / 256;
    g = mm % 256;
    b = (microns - mm * 1000) * 256 / 1000;  // 1/256th of a mm
  }
  return {r, g, b};
}

template <>
typename Gray8u::PixelType PixelConvert<Gray8u, Gray32f>(
    const typename Gray32f::PixelType& pix) {
  return std::clamp(pix, 0.0f, 255.0f / 256.0f) * 256.0f;
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Gray32f>(
    const typename Gray32f::PixelType& pix) {
  // Encode gray value in [0,1] into r, g and b channel, by encoding the value
  // as 256-decimal number. This results in a precision of <1e-7. This encoding
  // method is purely for storing and webgl rendering.
  if (pix < 0 || pix > 1) {
    LOG(ERROR) << "Invalid value for Gray32f: " << pix;
    return Rgb8u::Zero();
  }
  if (pix == 1) {
    return {255, 255, 255};
  }
  int gray_value = pix * 256 * 256 * 256;
  uint8_t r = gray_value % 256;
  gray_value = (gray_value - r) / 256;
  uint8_t g = gray_value % 256;
  gray_value = (gray_value - g) / 256;
  uint8_t b = gray_value % 256;
  return {r, g, b};
}

template <>
typename Gray32f::PixelType PixelConvert<Gray32f, Bgr8u>(
    const typename Bgr8u::PixelType& pix) {
  // Encode gray value in [0,1] into b, g and r channel, by encoding the value
  // as 256-decimal number. This results in a precision of <1e-7. This encoding
  // method is purely for storing and webgl rendering.
  return (static_cast<float>(pix.z()) + 256 * static_cast<float>(pix.y()) +
          256 * 256 * static_cast<float>(pix.x())) /
         (256 * 256 * 256);
}

template <>
typename Gray32f::PixelType PixelConvert<Gray32f, Rgb8u>(
    const typename Rgb8u::PixelType& pix) {
  // Encode gray value in [0,1] into r, g and b channel, by encoding the value
  // as 256-decimal number. This results in a precision of <1e-7. This encoding
  // method is purely for storing and webgl rendering.
  return (static_cast<float>(pix.x()) + 256 * static_cast<float>(pix.y()) +
          256 * 256 * static_cast<float>(pix.z())) /
         (256 * 256 * 256);
}

template <>
typename Rgba8u::PixelType PixelConvert<Rgba8u, Label32i>(
    const typename Label32i::PixelType& pix) {
  // Encode label32i value in [-1,num_instances] into r, g, b and a channel, by
  // encoding the value as 256-decimal number. This encoding method is purely
  // for storing.
  if (pix < -1) {
    LOG(ERROR) << "Invalid value for Label32i: " << pix;
    return Rgba8u::Zero();
  }
  int pix_value = pix + 1;
  uint8_t r = pix_value % 256;
  pix_value = (pix_value - r) / 256;
  uint8_t g = pix_value % 256;
  pix_value = (pix_value - g) / 256;
  uint8_t b = pix_value % 256;
  pix_value = (pix_value - b) / 256;
  uint8_t a = pix_value % 256;
  return {r, g, b, a};
}

template <>
typename Label32i::PixelType PixelConvert<Label32i, Bgra8u>(
    const typename Bgra8u::PixelType& pix) {
  int32_t pixel_val =
      (static_cast<int32_t>(pix.z()) + 256 * static_cast<int32_t>(pix.y()) +
       256 * 256 * static_cast<int32_t>(pix.x()) +
       256 * 256 * 256 * static_cast<int32_t>(pix.w()));
  return pixel_val - 1;
}

template <>
typename Label32i::PixelType PixelConvert<Label32i, Rgba8u>(
    const typename Rgba8u::PixelType& pix) {
  int32_t pixel_val =
      (static_cast<int32_t>(pix.x()) + 256 * static_cast<int32_t>(pix.y()) +
       256 * 256 * static_cast<int32_t>(pix.z()) +
       256 * 256 * 256 * static_cast<int32_t>(pix.w()));
  return pixel_val - 1;
}

template <>
typename Depth32f::PixelType PixelConvert<Depth32f, Bgr8u>(
    const typename Bgr8u::PixelType& pix) {
  Depth32f::PixelType depth = Depth32f::Invalid();
  if (pix.z() != 0 || pix.y() != 0 || pix.x() != 0) {
    depth = (static_cast<float>(pix.z()) * 256 + static_cast<float>(pix.y())) *
            .001;
    depth += static_cast<float>(pix.x()) * (1.0 / (256.0 * 1000.0));
  }
  return depth;
}

template <>
typename Depth32f::PixelType PixelConvert<Depth32f, Rgb8u>(
    const typename Rgb8u::PixelType& pix) {
  Depth32f::PixelType depth = Depth32f::Invalid();
  if (pix.x() != 0 || pix.y() != 0 || pix.z() != 0) {
    depth = (static_cast<float>(pix.x()) * 256 + static_cast<float>(pix.y())) *
            .001;
    depth += static_cast<float>(pix.z()) * (1.0 / (256.0 * 1000.0));
  }
  return depth;
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Depth16u>(
    const typename Depth16u::PixelType& pix) {
  // Encode 16 bit depth into r and g channel.
  // This encoding method is purely for storing and webgl rendering.
  uint8_t r = static_cast<int>(pix) / 256;
  uint8_t g = static_cast<int>(pix) % 256;
  return {r, g, 0};
}

template <>
typename Depth16u::PixelType PixelConvert<Depth16u, Bgr8u>(
    const typename Bgr8u::PixelType& pix) {
  uint16_t depth =
      static_cast<uint16_t>(pix.z()) * 256 + static_cast<uint16_t>(pix.y());
  return depth;
}

template <>
typename Depth16u::PixelType PixelConvert<Depth16u, Rgb8u>(
    const typename Rgb8u::PixelType& pix) {
  uint16_t depth =
      static_cast<uint16_t>(pix.x()) * 256 + static_cast<uint16_t>(pix.y());
  return depth;
}

template <>
typename Bgr8u::PixelType PixelConvert<Bgr8u, Rgb8u>(
    const typename Rgb8u::PixelType& pix) {
  return {pix.z(), pix.y(), pix.x()};
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Bgr8u>(
    const typename Bgr8u::PixelType& pix) {
  return {pix.z(), pix.y(), pix.x()};
}

template <>
typename Bgra8u::PixelType PixelConvert<Bgra8u, Rgba8u>(
    const typename Rgba8u::PixelType& pix) {
  return {pix.z(), pix.y(), pix.x(), pix.w()};
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Rgba8u>(
    const typename Rgba8u::PixelType& pix) {
  return {pix.x(), pix.y(), pix.z()};
}

template <>
typename Rgba8u::PixelType PixelConvert<Rgba8u, Bgra8u>(
    const typename Bgra8u::PixelType& pix) {
  return {pix.z(), pix.y(), pix.x(), pix.w()};
}

template <>
typename Rgb8u::PixelType PixelConvert<Rgb8u, Bool8u>(
    const typename Bool8u::PixelType& pix) {
  if (pix > 0) {
    return {255, 255, 255};
  }
  return {0, 0, 0};
}

template <>
typename Bool8u::PixelType PixelConvert<Bool8u, Bgr8u>(
    const typename Bgr8u::PixelType& pix) {
  return pix.z();
}

template <>
typename Bool8u::PixelType PixelConvert<Bool8u, Rgb8u>(
    const typename Rgb8u::PixelType& pix) {
  return pix.x();
}

template <>
typename Point32f::PixelType PixelConvert<Point32f, Generic32f4>(
    const typename Generic32f4::PixelType& pix) {
  return {pix.x(), pix.y(), pix.z()};
}

Rgb8u::PixelType PointConvert(const Point32f::PixelType& pix, int channel) {
  if (Point32f::IsInvalid(pix)) {
    return {255, 255, 255};
  }
  // The 24 bits encoded as r,g,b are used as follows:
  // Bit 0: sign (0 = positive, 1 = negative)
  // Bit 1-3: Represents whole meters allowing the range (-8, 8) excluding the
  // boundaries.
  // Bit 4-23: Fractional part of the fixed point number (micrometer precision).
  constexpr int64_t kFixedPointPerMeter = 0x100000;
  constexpr int64_t kMaxBits = 0x7fffff;
  constexpr float kMaxFloat = static_cast<float>(0xffffff);
  // Taking the min with kMaxFloat ensures to be inside the integer range.
  int64_t value_fixed_point =
      std::min(std::abs(pix(channel)) * kFixedPointPerMeter, kMaxFloat);
  if (value_fixed_point > kMaxBits) {
    return {255, 255, 255};
  }
  uint8_t sign_bit = pix(channel) < 0.0 ? 128 : 0;
  // Edge case negative maximum: In that case we clamp a bit away to make it
  // distinguishable from the invalid pixel.
  if (value_fixed_point == kMaxBits && sign_bit != 0) {
    value_fixed_point -= 1;
  }
  return {static_cast<uint8_t>(sign_bit + (value_fixed_point >> 16)),
          static_cast<uint8_t>((value_fixed_point & 0xffff) >> 8),
          static_cast<uint8_t>(value_fixed_point & 0xff)};
}

Point32f::ScalarType PointConvert(const Rgb8u::PixelType& p) {
  const Point32f::ScalarType sign = p(0) >= 128 ? -1.0f : 1.0;
  return sign * (static_cast<Point32f::ScalarType>(p(0) & 127) / 16.0f +
                 static_cast<Point32f::ScalarType>(p(1)) / 256.0f / 16.0f +
                 static_cast<Point32f::ScalarType>(p(2)) / 65536.0f / 16.0f);
}

Point32f::PixelType PointConvert(const Rgb8u::PixelType& x,
                                 const Rgb8u::PixelType& y,
                                 const Rgb8u::PixelType& z) {
  if (x == Rgb8u::PixelType(255, 255, 255) ||
      y == Rgb8u::PixelType(255, 255, 255) ||
      z == Rgb8u::PixelType(255, 255, 255)) {
    return Point32f::Invalid();
  }
  return {PointConvert(x), PointConvert(y), PointConvert(z)};
}

}  // namespace perception
}  // namespace intrinsic
