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

#include "intrinsic/perception/cameras/camera_lister.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/image_source_lister.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

absl::Status AppendCameras(
    std::unique_ptr<ImageSourceLister> image_source_lister,
    std::vector<CameraIdentifier>& camera_identifiers) {
  INTR_ASSIGN_OR_RETURN(std::vector<CameraIdentifier> cameras_from_image_source,
                        image_source_lister->ListAvailableCameras());
  camera_identifiers.insert(camera_identifiers.end(),
                            cameras_from_image_source.begin(),
                            cameras_from_image_source.end());
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<std::vector<CameraIdentifier>> ListAvailableCameras() {
  std::vector<CameraIdentifier> available_cameras;
  for (const auto& kv : GetImageSourceListerAliasRegistry()) {
    INTR_ASSIGN_OR_RETURN(auto lister, ImageSourceListerRegistry::Dispatch(
                                           std::string(kv.first)));
    INTR_RETURN_IF_ERROR(AppendCameras(std::move(lister), available_cameras));
  }
  return available_cameras;
}

}  // namespace perception
}  // namespace intrinsic
