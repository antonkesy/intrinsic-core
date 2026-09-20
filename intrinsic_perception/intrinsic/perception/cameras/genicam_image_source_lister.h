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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_GENICAM_IMAGE_SOURCE_LISTER_H_
#define INTRINSIC_PERCEPTION_CAMERAS_GENICAM_IMAGE_SOURCE_LISTER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/perception/cameras/camera_identifier.h"
#include "intrinsic/perception/cameras/image_source_lister.h"

namespace intrinsic {
namespace perception {

class GenICamImageSourceLister final : public ImageSourceLister {
 public:
  absl::StatusOr<std::vector<CameraIdentifier>> ListAvailableCameras() final;
};

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CAMERAS_GENICAM_IMAGE_SOURCE_LISTER_H_
