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

#include "intrinsic/perception/gpu/cuda_utils/cuda_utils.h"

#include <exception>
#include <string>

#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "intrinsic/perception/core/image_traits.h"

// TODO(b/418985701): Remove this once the flag propagates correctly.
#ifndef _VSTD
#define _VSTD std
#endif
#include "thrust/device_vector.h"

namespace intrinsic::perception {

template <>
cudaChannelFormatDesc CudaGetChannelDesc<GenericTrait<half2, 1>>() {
  return cudaChannelFormatDesc{
      .x = 16,
      .y = 16,
      .z = 0,
      .w = 0,
      .f = cudaChannelFormatKindFloat,
  };
}

std::string GetCudaDeviceInfo() {
  int driver_version = 0;
  cudaDriverGetVersion(&driver_version);
  int runtime_version = 0;
  cudaRuntimeGetVersion(&runtime_version);
  std::string device_info =
      absl::StrFormat("Driver version: %d Runtime version: %d\n",
                      driver_version, runtime_version);

  int device_count = 0;
  auto status = CudaGetErrorStatus(cudaGetDeviceCount(&device_count));
  if (!status.ok()) {
    LOG(ERROR) << "Failed to get CUDA device count: " << status;
    return absl::StrCat(device_info, "Failed to get CUDA device count");
  }
  device_info =
      absl::StrFormat("%sCUDA device count: %d\n", device_info, device_count);

  for (int i = 0; i < device_count; ++i) {
    cudaDeviceProp prop;
    status = CudaGetErrorStatus(cudaGetDeviceProperties(&prop, i));
    if (!status.ok()) {
      LOG(ERROR) << "Failed to get CUDA device properties[" << i
                 << "]: " << status;
      return absl::StrCat(device_info, "Failed to get CUDA device properties[",
                          i, "]");
    }
    absl::StrAppendFormat(
        &device_info,
        "CUDA device name[%d]: %s memory: %fMB max threads per block: %d\n", i,
        prop.name, prop.totalGlobalMem / 1024 / 1024, prop.maxThreadsPerBlock);
  }
  return device_info;
}

absl::Status VerifyCompiledBinaryAgainstCurrentComputeCapability() {
  // Test whether this binary was compiled with the correct architecture for the
  // current GPU. An easy way to check is to create a device_vector which will
  // throw an exception if the current binary was compiled for the wrong
  // architecture.
  try {
    thrust::device_vector<int> test_vector(100);
    if (test_vector.empty()) {
      LOG(WARNING)
          << "Failed to create device_vector for testing whether this binary "
             "was compiled for the correct architecture for the current GPU. "
             "This is suspicious but does not necessarily mean that the binary "
             "was compiled for the wrong architecture.";
    }
  } catch (const std::exception& e) {
    return absl::InternalError(e.what());
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::perception
