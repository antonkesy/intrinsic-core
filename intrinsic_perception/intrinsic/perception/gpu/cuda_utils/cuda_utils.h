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

#ifndef INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_UTILS_H_
#define INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_UTILS_H_

#include <channel_descriptor.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <driver_types.h>
#include <vector_types.h>

#include <cstdint>

#include "absl/base/optimization.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/image_traits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {

inline absl::Status CudaGetErrorStatus(cudaError_t error_code) {
  if (ABSL_PREDICT_TRUE(error_code == cudaSuccess)) {
    return absl::OkStatus();
  }
  return absl::UnknownError(absl::StrFormat(
      "CUDA error code %d: %s", error_code, cudaGetErrorString(error_code)));
}

#define CUDA_CHECK(error_code)                            \
  CHECK_OK(intrinsic::perception::CudaGetErrorStatus(     \
      cudaPeekAtLastError() == cudaSuccess ? (error_code) \
                                           : cudaGetLastError()))

// Evaluates the expression and exits the current function in case of a Cuda
// error. Note that if there already is a Cuda error from a previous call, the
// expression is not evaluated but only the error is returned.
#define RETURN_IF_CUDA_ERROR(expr)                                        \
  do {                                                                    \
    bool has_previous_error = (cudaPeekAtLastError() != cudaSuccess);     \
    INTR_RETURN_IF_ERROR(intrinsic::perception::CudaGetErrorStatus(       \
        has_previous_error ? cudaGetLastError() : (expr)))                \
        << " in: " << __FILE__ << ":" << __LINE__                         \
        << (has_previous_error                                            \
                ? " (error originated before current expression and the " \
                  "expression wasn't executed)"                           \
                : "");                                                    \
  } while (false)

inline bool GpuIsAvailable() {
  int device_count;
  return CudaGetErrorStatus(cudaGetDeviceCount(&device_count)).ok() &&
         device_count > 0;
}

template <typename ImageTrait>
cudaChannelFormatDesc CudaGetChannelDesc() {
  using ScalarType = typename ImageTrait::ScalarType;
  cudaChannelFormatDesc descriptor;
  descriptor.x = sizeof(ScalarType) * 8;
  descriptor.y = ImageTrait::kNumChannels > 1 ? sizeof(ScalarType) * 8 : 0;
  descriptor.z = ImageTrait::kNumChannels > 2 ? sizeof(ScalarType) * 8 : 0;
  descriptor.w = ImageTrait::kNumChannels > 3 ? sizeof(ScalarType) * 8 : 0;
  if constexpr (std::is_same<ScalarType, uint8_t>::value ||
                std::is_same<ScalarType, uint16_t>::value ||
                std::is_same<ScalarType, uint32_t>::value) {
    descriptor.f = cudaChannelFormatKindUnsigned;
  } else if constexpr (std::is_same<ScalarType, int8_t>::value ||
                       std::is_same<ScalarType, int16_t>::value ||
                       std::is_same<ScalarType, int32_t>::value) {
    descriptor.f = cudaChannelFormatKindSigned;
  } else if constexpr (std::is_same<ScalarType, float>::value) {
    descriptor.f = cudaChannelFormatKindFloat;
  } else {
    static_assert(false, "Unsupported ScalarType");
  }
  return descriptor;
}

template <>
cudaChannelFormatDesc CudaGetChannelDesc<GenericTrait<half2, 1>>();

std::string GetCudaDeviceInfo();

absl::Status VerifyCompiledBinaryAgainstCurrentComputeCapability();

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_GPU_CUDA_UTILS_CUDA_UTILS_H_
