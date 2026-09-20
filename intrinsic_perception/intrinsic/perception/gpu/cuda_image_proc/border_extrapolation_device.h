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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_BORDER_EXTRAPOLATION_DEVICE_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_BORDER_EXTRAPOLATION_DEVICE_H_

#include <cuda_runtime.h>

#include "intrinsic/perception/gpu/cuda_image_proc/border_extrapolation.h"

namespace intrinsic {
namespace perception {

// `size` is assumed to be > 0 and usually refers to image rows or columns but
// this depends on the use case for this function. `i` refers to the pixel
// index, which can be negative or >= `size`. Returns -1 when border_type is
// kBorderConstantZero. Otherwise returns index in range {0, ..., size - 1}.
__device__ inline int GetExtrapolatedPixelIndex(int i, int size,
                                                BorderType border_type) {
  if (i >= 0 && i < size) {
    return i;
  } else if (border_type == BorderType::kBorderReflect101) {
    if (size == 1) {
      return 0;
    }
    do {
      if (i < 0) {
        i = -i;
      } else {
        i = size - 1 - (i - size) - 1;
      }
    } while (i < 0 || i >= size);
    return i;
  }
  return -1;
}

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_IMAGE_PROC_BORDER_EXTRAPOLATION_DEVICE_H_
