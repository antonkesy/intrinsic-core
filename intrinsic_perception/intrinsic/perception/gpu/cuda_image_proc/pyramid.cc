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

#include "intrinsic/perception/gpu/cuda_image_proc/pyramid.h"

#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/dimensions.h"
#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"
#include "intrinsic/perception/gpu/cuda_image_proc/pixelwise.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

template <typename OutputImageTraits, typename InputImageTraits>
absl::StatusOr<std::vector<CudaImage<OutputImageTraits>>>
CreateLaplacianPyramid(const CudaImage<InputImageTraits>& image,
                       Dimensions min_dimensions, BorderType border_type) {
  std::vector<CudaImage<OutputImageTraits>> laplacian_pyramid;
  if (min_dimensions.cols < 1 || min_dimensions.rows < 1) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "min_dimensions must be at least 1 x 1 (cols x rows). (was %d x %d)",
        min_dimensions.cols, min_dimensions.rows));
  }
  if (min_dimensions.cols > image.cols() ||
      min_dimensions.rows > image.rows()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "min_dimensions must be at most the same as the dimensions %d x %d "
        "(cols x rows) of the input image. (min_dimensions was %d x %d)",
        image.cols(), image.rows(), min_dimensions.cols, min_dimensions.rows));
  }
  std::optional<CudaImage<InputImageTraits>> image_cur = std::nullopt;
  while (true) {
    const CudaImage<InputImageTraits>& current_image =
        [&image, &image_cur]() -> const CudaImage<InputImageTraits>& {
      if (image_cur.has_value()) return *image_cur;
      return image;
    }();
    Dimensions dimensions_down = DimensionsPyrDown(current_image.dimensions());
    if (dimensions_down.cols < min_dimensions.cols ||
        dimensions_down.rows < min_dimensions.rows ||
        dimensions_down.cols == 1 || dimensions_down.rows == 1) {
      if constexpr (std::is_same_v<OutputImageTraits, InputImageTraits>) {
        laplacian_pyramid.push_back(std::move(current_image));
      } else {
        INTR_ASSIGN_OR_RETURN(auto image_cur_out,
                              Cast<OutputImageTraits>(current_image));
        laplacian_pyramid.push_back(std::move(image_cur_out));
      }
      break;
    }
    INTR_ASSIGN_OR_RETURN(auto image_down, PyrDown(current_image, border_type));
    INTR_RET_CHECK_EQ(dimensions_down, image_down.dimensions())
        << "The predicted dimensions are different from the actual "
           "dimensions of the downsampled image.";
    INTR_ASSIGN_OR_RETURN(
        const auto image_cur_upsampled,
        PyrUp(image_down, current_image.dimensions(), border_type));
    INTR_ASSIGN_OR_RETURN(
        auto pyr_cur,
        Subtract<OutputImageTraits>(current_image, image_cur_upsampled));
    laplacian_pyramid.push_back(std::move(pyr_cur));
    image_cur = std::move(image_down);
  }
  return laplacian_pyramid;
}

template absl::StatusOr<std::vector<CudaImage<Generic16i>>>
CreateLaplacianPyramid(const CudaImage<Gray8u>& image,
                       Dimensions min_dimensions, BorderType border_type);

template absl::StatusOr<std::vector<CudaImage<Generic16i3>>>
CreateLaplacianPyramid(const CudaImage<Rgb8u>& image, Dimensions min_dimensions,
                       BorderType border_type);

template absl::StatusOr<std::vector<CudaImage<Generic32f>>>
CreateLaplacianPyramid(const CudaImage<Generic32f>& image,
                       Dimensions min_dimensions, BorderType border_type);

template absl::StatusOr<std::vector<CudaImage<Generic32f3>>>
CreateLaplacianPyramid(const CudaImage<Generic32f3>& image,
                       Dimensions min_dimensions, BorderType border_type);

template <typename OutputImageTraits, typename InputImageTraits>
absl::StatusOr<CudaImage<OutputImageTraits>>
ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<InputImageTraits>>& laplacian_pyramid,
    BorderType border_type) {
  if (laplacian_pyramid.empty()) {
    return absl::InvalidArgumentError("laplacian_pyramid must not be empty.");
  }
  std::optional<CudaImage<InputImageTraits>> residual_cur = std::nullopt;
  for (int i = static_cast<int>(laplacian_pyramid.size()) - 2; i >= 0; --i) {
    CudaImage<InputImageTraits> residual_upsampled;
    if (residual_cur.has_value()) {
      INTR_ASSIGN_OR_RETURN(
          residual_upsampled,
          PyrUp(*residual_cur, laplacian_pyramid[i].dimensions(), border_type));
    } else {
      INTR_ASSIGN_OR_RETURN(
          residual_upsampled,
          PyrUp(laplacian_pyramid.back(), laplacian_pyramid[i].dimensions(),
                border_type));
    }
    INTR_ASSIGN_OR_RETURN(
        auto residual,
        Add<InputImageTraits>(laplacian_pyramid[i], residual_upsampled));
    residual_cur = std::move(residual);
  }
  if (!residual_cur.has_value()) {
    // Makes a copy.
    residual_cur = laplacian_pyramid.back();
  }
  INTR_RETURN_IF_ERROR(Clamp(*residual_cur, /*min=*/0,
                             static_cast<InputImageTraits::ScalarType>(
                                 OutputImageTraits::kIntensityMax)));
  CudaImage<OutputImageTraits> residual_out;
  if constexpr (std::is_integral<
                    typename OutputImageTraits::ScalarType>::value &&
                std::is_floating_point<
                    typename InputImageTraits::ScalarType>::value) {
    INTR_ASSIGN_OR_RETURN(residual_out,
                          RoundThenCast<OutputImageTraits>(*residual_cur));
  } else {
    INTR_ASSIGN_OR_RETURN(residual_out, Cast<OutputImageTraits>(*residual_cur));
  }
  return residual_out;
}

template absl::StatusOr<CudaImage<Gray8u>> ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<Generic16i>>& laplacian_pyramid,
    BorderType border_type);

template absl::StatusOr<CudaImage<Rgb8u>> ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<Generic16i3>>& laplacian_pyramid,
    BorderType border_type);

template absl::StatusOr<CudaImage<Gray8u>> ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<Generic32f>>& laplacian_pyramid,
    BorderType border_type);

template absl::StatusOr<CudaImage<Rgb8u>> ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<Generic32f3>>& laplacian_pyramid,
    BorderType border_type);

template absl::StatusOr<CudaImage<Gray32f>>
ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<Generic32f>>& laplacian_pyramid,
    BorderType border_type);

template absl::StatusOr<CudaImage<Rgb32f>> ReconstructImageFromLaplacianPyramid(
    const std::vector<CudaImage<Generic32f3>>& laplacian_pyramid,
    BorderType border_type);

}  // namespace perception
}  // namespace intrinsic
