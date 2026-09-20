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

#include "intrinsic/perception/cameras/aravis/aravis_ptp_utils.h"

#include <arv.h>

#include "absl/base/nullability.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/perception/cameras/aravis/aravis_software_trigger.h"
#include "intrinsic/perception/cameras/aravis/aravis_utils.h"
#include "intrinsic/perception/cameras/genicam/feature_names.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace perception {
namespace {

constexpr char kBasler[] = "Basler";
constexpr char kBaslerAcePrefix[] = "acA";
constexpr char kBaslerAce2Prefix[] = "a2A";

// Command setting that resets the PTP synchonization timers.
constexpr char kBaslerAceSyncFreeRunTimerUpdate[] = "SyncFreeRunTimerUpdate";
// Enables the PTP based camera synchronisation.
constexpr char kBaslerAceSyncFreeRunTimerEnable[] = "SyncFreeRunTimerEnable";

}  // namespace

absl::Status ConfigurePtpSyncAndStreamingTrigger(
    ArvCamera* absl_nonnull camera) {
  if (camera == nullptr) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "ArvCamera must not be a nullptr.";
  }

  // Depending on the camera different genicam parameters signify the use of PTP
  // synchronisation.
  const bool ptp_sync =
      arv_camera_get_boolean(camera, genicam::kPtpEnable, nullptr) ||
      arv_camera_get_boolean(camera, genicam::kGevIEEE1588, nullptr);

  if (!ptp_sync) {
    return SetFreeRunningTrigger(camera);
  }

  // Specific Vendor/Model combinations support PTP via different settings and
  // the trigger modes which must be set in different orders.
  if (GetVendorName(camera) == kBasler &&
      GetModelName(camera).starts_with(kBaslerAcePrefix)) {
    GError* error = nullptr;
    arv_camera_set_frame_rate_enable(camera, false, &error);
    INTR_RETURN_IF_ERROR(ValidateAndNullArvError(error));
    // The free running trigger must be set before updating and enabling the
    // timer.
    INTR_RETURN_IF_ERROR(SetFreeRunningTrigger(camera));
    arv_camera_execute_command(camera, kBaslerAceSyncFreeRunTimerUpdate,
                               &error);
    INTR_RETURN_IF_ERROR(ValidateAndNullArvError(error));
    arv_camera_set_boolean(camera, kBaslerAceSyncFreeRunTimerEnable, 1, &error);
    INTR_RETURN_IF_ERROR(ValidateAndNullArvError(error));
  } else if (GetVendorName(camera) == kBasler &&
             GetModelName(camera).starts_with(kBaslerAce2Prefix)) {
    GError* error = nullptr;
    arv_camera_set_frame_rate_enable(camera, false, &error);
    INTR_RETURN_IF_ERROR(ValidateAndNullArvError(error));
    INTR_RETURN_IF_ERROR(SetPeriodicTrigger(camera));
  } else {
    LOG(WARNING) << "PTP camera stream synchronisation is enabled but is not "
                    "supported for this camera.";
    INTR_RETURN_IF_ERROR(SetFreeRunningTrigger(camera));
  }

  return absl::OkStatus();
}

}  // namespace perception
}  // namespace intrinsic
