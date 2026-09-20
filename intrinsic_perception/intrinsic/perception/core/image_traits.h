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

#ifndef INTRINSIC_PERCEPTION_CORE_IMAGE_TRAITS_H_
#define INTRINSIC_PERCEPTION_CORE_IMAGE_TRAITS_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "intrinsic/perception/core/eigen_types.h"

namespace intrinsic {
namespace perception {

template <class ScalarT, uint8_t num_channels>
struct GenericTrait {
  static constexpr uint16_t kPaddingBytes = 0;
  static constexpr uint8_t kNumChannels = num_channels;
  using PixelType = Vector<ScalarT, kNumChannels>;
  using ScalarType = ScalarT;
  static PixelType Zero() { return Vector<ScalarT, kNumChannels>::Zero(); }
};

// specialization for scalar pixels
template <class ScalarT>
struct GenericTrait<ScalarT, 1> {
  static constexpr uint16_t kPaddingBytes = 0;
  static constexpr uint8_t kNumChannels = 1;
  using PixelType = ScalarT;
  using ScalarType = ScalarT;
  static ScalarT Zero() { return ScalarT(0); }
};

using Generic8u = GenericTrait<uint8_t, 1>;
using Generic16u = GenericTrait<uint16_t, 1>;
using Generic16u3 = GenericTrait<uint16_t, 3>;
using Generic32u = GenericTrait<uint32_t, 1>;
using Generic8u3 = GenericTrait<uint8_t, 3>;
using Generic8i = GenericTrait<int8_t, 1>;
using Generic16i = GenericTrait<int16_t, 1>;
using Generic16i2 = GenericTrait<int16_t, 2>;
using Generic16i3 = GenericTrait<int16_t, 3>;
using Generic32i = GenericTrait<int32_t, 1>;
using Generic8i3 = GenericTrait<int8_t, 3>;
using Generic32f = GenericTrait<float, 1>;
using Generic32f2 = GenericTrait<float, 2>;
using Generic32f3 = GenericTrait<float, 3>;
using Generic32f4 = GenericTrait<float, 4>;

template <class ScalarT, uint64_t denominator>
struct DepthTrait : GenericTrait<ScalarT, 1> {
  static constexpr uint64_t kDenominator = denominator;

  static typename DepthTrait::PixelType Invalid() {
    return typename DepthTrait::PixelType(0u);
  }
  static bool IsValid(const typename DepthTrait::PixelType& pix) {
    return pix != Invalid();
  }
  static bool IsInvalid(const typename DepthTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

// floating point specialization of depth trait with invalid val of NAN.
template <uint64_t denominator>
struct DepthTrait<float, denominator> : GenericTrait<float, 1> {
  static constexpr uint64_t kDenominator = denominator;

  static typename DepthTrait::PixelType Invalid() {
    return
        typename DepthTrait::PixelType(std::numeric_limits<float>::quiet_NaN());
  }
  static bool IsValid(const typename DepthTrait::PixelType& pix) {
    return !std::isnan(pix);
  }
  static bool IsInvalid(const typename DepthTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

// uint16_t depth in millimeters
using Depth16u = DepthTrait<uint16_t, 1000u>;
// float depth in meters
using Depth32f = DepthTrait<float, 1u>;

// Orientation trait
template <class ScalarT, uint64_t modulo_factor>
struct OrientationTrait : GenericTrait<ScalarT, 1> {
  static typename OrientationTrait::PixelType Invalid() {
    return typename OrientationTrait::PixelType(
        std::numeric_limits<ScalarT>::quiet_NaN());
  }
  static bool IsValid(const typename OrientationTrait::PixelType& pix) {
    return !std::isnan(pix);
  }
  static bool IsInvalid(const typename OrientationTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

using Orientation32f = OrientationTrait<float, 360u>;

// Meta trait for intensity types
template <typename ScalarT, uint8_t num_channels, uint64_t max>
struct IntensityTrait : GenericTrait<ScalarT, num_channels> {
  static constexpr uint64_t kIntensityMax = max;
  static_assert(std::is_unsigned<ScalarT>::value ||
                    std::is_floating_point<ScalarT>::value,
                "Scalar must be unsigned integral or floating point type.");
  static bool IsValid(const typename IntensityTrait::PixelType& pix) {
    return true;
  }
  static bool IsInvalid(const typename IntensityTrait::PixelType& pix) {
    return false;
  }
};

// The generic RGB pixel type.
template <typename ScalarT, uint64_t max>
struct RgbTrait : IntensityTrait<ScalarT, 3, max> {};

// uint8_t RGB in the range [0,255]
using Rgb8u = RgbTrait<uint8_t, 255u>;
// float RGB in the range [0,1]
using Rgb32f = RgbTrait<float, 1u>;

// The generic RGBA pixel type.
template <typename ScalarT, uint64_t max>
struct RgbaTrait : IntensityTrait<ScalarT, 4, max> {};

// uint8_t RGBA in the range [0,255]
using Rgba8u = RgbaTrait<uint8_t, 255u>;
// float RGBA in the range [0,1]
using Rgba32f = RgbaTrait<float, 1u>;

template <typename ScalarT, uint64_t max>
struct BgrTrait : IntensityTrait<ScalarT, 3, max> {};

// uint8_t BGR in the range [0,255]
using Bgr8u = BgrTrait<uint8_t, 255u>;
// float BGR in the range [0,1]
using Bgr32f = BgrTrait<float, 1u>;

// The generic BGRA pixel type.
template <typename ScalarT, uint64_t max>
struct BgraTrait : IntensityTrait<ScalarT, 4, max> {};

// uint8_t BGRA in the range [0,255]
using Bgra8u = BgraTrait<uint8_t, 255u>;
// float BGRA in the range [0,1]
using Bgra32f = BgraTrait<float, 1u>;

template <typename ScalarT, uint64_t max>
struct GrayTrait : IntensityTrait<ScalarT, 1, max> {};

// uint8_t gray-scale in the range [0,255]
using Gray8u = GrayTrait<uint8_t, 255u>;
// uint16_t gray-scale in the range [0,65535]
using Gray16u = GrayTrait<uint16_t, 65535u>;
// float gray-scale in the range [0,1]
using Gray32f = GrayTrait<float, 1u>;

// Boolean image.
struct Bool8u : GenericTrait<uint8_t, 1> {};

template <typename ScalarT>
struct NormalTrait : GenericTrait<ScalarT, 3> {
  static typename NormalTrait::PixelType Invalid() {
    return typename NormalTrait::PixelType(
        std::numeric_limits<ScalarT>::quiet_NaN(),
        std::numeric_limits<ScalarT>::quiet_NaN(),
        std::numeric_limits<ScalarT>::quiet_NaN());
  }
  static bool IsValid(const typename NormalTrait::PixelType& pix) {
    return !(std::isnan(pix.x()) || std::isnan(pix.y()) || std::isnan(pix.z()));
  }
  static bool IsInvalid(const typename NormalTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

// float normal image
using Normal32f = NormalTrait<float>;

template <typename ScalarT>
struct GradientTrait : GenericTrait<ScalarT, 2> {
  static typename GradientTrait::PixelType Invalid() {
    return typename GradientTrait::PixelType(
        std::numeric_limits<ScalarT>::quiet_NaN(),
        std::numeric_limits<ScalarT>::quiet_NaN());
  }
  static bool IsValid(const typename GradientTrait::PixelType& pix) {
    return !std::isnan(pix.x()) && !std::isnan(pix.y());
  }
  static bool IsInvalid(const typename GradientTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

// float normal image
using Gradient32f = GradientTrait<float>;

template <typename ScalarT, uint64_t denominator = 1u>
struct PointTrait : GenericTrait<ScalarT, 3> {
  static constexpr uint64_t kDenominator = denominator;
  static typename PointTrait::PixelType Invalid() {
    return typename PointTrait::PixelType(
        std::numeric_limits<ScalarT>::quiet_NaN(),
        std::numeric_limits<ScalarT>::quiet_NaN(),
        std::numeric_limits<ScalarT>::quiet_NaN());
  }
  static bool IsValid(const typename PointTrait::PixelType& pix) {
    return !std::isnan(pix.x()) && !std::isnan(pix.y()) && !std::isnan(pix.z());
  }
  static bool IsInvalid(const typename PointTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

using Point32f = PointTrait<float>;

template <typename ScalarT>
struct PixelTrait : GenericTrait<ScalarT, 2> {
  static typename PixelTrait::PixelType Invalid() {
    return typename PixelTrait::PixelType(
        std::numeric_limits<ScalarT>::quiet_NaN(),
        std::numeric_limits<ScalarT>::quiet_NaN());
  }
  static bool IsValid(const typename PixelTrait::PixelType& pix) {
    return !(std::isnan(pix.x()) || std::isnan(pix.y()));
  }
  static bool IsInvalid(const typename PixelTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

using Pixel32f = PixelTrait<float>;

template <typename ScalarT>
struct LabelTrait : GenericTrait<ScalarT, 1> {
  static typename LabelTrait::PixelType Invalid() {
    return typename LabelTrait::PixelType(-1);
  }
  static bool IsValid(const typename LabelTrait::PixelType& pix) {
    return pix != LabelTrait::PixelType(-1);
  }
  static bool IsInvalid(const typename LabelTrait::PixelType& pix) {
    return !IsValid(pix);
  }
};

using Label32i = LabelTrait<int32_t>;

template <typename MyImageTrait>
class Image;

template <typename>
struct IsImage : std::false_type {};

template <typename ImageTrait>
struct IsImage<Image<ImageTrait>> : std::true_type {};

template <typename T>
inline constexpr bool IsImageValue = IsImage<T>::value;

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CORE_IMAGE_TRAITS_H_
