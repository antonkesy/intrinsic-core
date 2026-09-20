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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_IMAGE_UTILS_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_IMAGE_UTILS_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/core/image.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/perception/gpu/cuda_utils/cuda_image.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::perception {

template <typename ImageTrait>
absl::Status FromCudaImage(const CudaImage<ImageTrait>& cuda_image,
                           Image<ImageTrait>& host_image) {
  INTR_RET_CHECK_EQ(host_image.dimensions(), cuda_image.dimensions())
      .SetCode(absl::StatusCode::kInvalidArgument);
  cuda_image.CopyDataToHost(
      reinterpret_cast<typename ImageTrait::ScalarType*>(host_image.data()));
  return absl::OkStatus();
}

template <typename ImageTrait>
absl::StatusOr<Image<ImageTrait>> FromCudaImage(
    const CudaImage<ImageTrait>& image) {
  Image<ImageTrait> host_image(image.dimensions());
  INTR_RETURN_IF_ERROR(FromCudaImage(image, host_image));
  return host_image;
}

template <typename ImageTrait>
absl::Status ToCudaImage(const Image<ImageTrait>& image,
                         CudaImage<ImageTrait>& cuda_image) {
  INTR_RET_CHECK_EQ(cuda_image.dimensions(), image.dimensions())
      .SetCode(absl::StatusCode::kInvalidArgument);
  cuda_image.CopyDataFromHost(
      reinterpret_cast<const typename ImageTrait::ScalarType*>(image.data()));
  return absl::OkStatus();
}

template <typename ImageTrait>
absl::StatusOr<CudaImage<ImageTrait>> ToCudaImage(
    const Image<ImageTrait>& image) {
  INTR_ASSIGN_OR_RETURN(CudaImage<ImageTrait> cuda_image,
                        CudaImage<ImageTrait>::Create(image.dimensions()));
  INTR_RETURN_IF_ERROR(ToCudaImage(image, cuda_image));
  return cuda_image;
}
}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_IMAGE_UTILS_H_
