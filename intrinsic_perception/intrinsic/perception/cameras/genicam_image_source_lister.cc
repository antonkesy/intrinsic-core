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

#include "intrinsic/perception/cameras/genicam_image_source_lister.h"

#include <arv.h>

#include <vector>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/image_source_lister.h"

namespace intrinsic {
namespace perception {

absl::StatusOr<std::vector<CameraIdentifier>>
GenICamImageSourceLister::ListAvailableCameras() {
  std::vector<CameraIdentifier> camera_identifiers;
  arv_update_device_list();
  const int num_devices = arv_get_n_devices();
  camera_identifiers.reserve(num_devices);
  LOG(INFO) << "ListAvailableCameras() returned " << num_devices
            << " Genicam cameras";
  for (int device_idx = 0; device_idx < num_devices; ++device_idx) {
    const auto* const device_id = arv_get_device_id(device_idx);
    LOG(INFO) << "Camera " << device_idx << ": " << device_id;
    camera_identifiers.push_back(
        {.driver = CameraIdentifier::GenICam{.device_id = device_id}});
  }
  return camera_identifiers;
}

REGISTER_IMAGE_SOURCE_LISTER(GenICamImageSourceLister, "genicam");

}  // namespace perception
}  // namespace intrinsic
