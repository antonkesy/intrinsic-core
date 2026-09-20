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

#include "intrinsic/perception/cameras/software_hdr.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/btree_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/perception/cameras/camera_setting.h"
#include "intrinsic/perception/cameras/camera_setting_access.h"
#include "intrinsic/perception/cameras/camera_setting_properties.h"
#include "intrinsic/perception/cameras/camera_setting_utils.h"
#include "intrinsic/perception/cameras/genicam/feature_names.h"
#include "intrinsic/perception/core/hdr_utils.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"
#include "magic_enum/magic_enum.hpp"
#include "magic_enum/magic_enum_utility.hpp"

namespace intrinsic::perception {

namespace {

constexpr char kEnumPrefix[] = "k";
// LINT.IfChange(hdr_preset_medium)
constexpr std::array<double, 3> kKneePointExposurePercentagesPresetMedium = {
    1.0, 10.0, 200.0};
// LINT.ThenChange(//intrinsic/frontend/onprem/perception/views/camera_settings_dialog.ts:hdr_preset_medium)

inline constexpr auto kParametersDefiningHdrMode =
    std::array<const absl::string_view, 4>(
        {genicam::kMultiSlopeMode, genicam::kMultiSlopeKneePointCount,
         genicam::kMultiSlopeKneePointSelector,
         genicam::kMultiSlopeExposureLimit});

absl::StatusOr<HdrMode> StrToHdrMode(const std::string& hdr_mode_str) {
  std::optional<HdrMode> hdr_mode =
      magic_enum::enum_cast<HdrMode>(kEnumPrefix + hdr_mode_str);
  if (hdr_mode.has_value()) return hdr_mode.value();

  return ::intrinsic::InvalidArgumentErrorBuilder()
         << "Invalid HDR mode: " << hdr_mode_str;
}

std::string HdrModeToStr(const HdrMode hdr_mode) {
  std::string mode_name{magic_enum::enum_name(hdr_mode)};
  // Remove prefix k from HDR mode.
  return mode_name.erase(0, 1);
}

}  // namespace

SoftwareHdr::SoftwareHdr(absl::Duration min_exposure_time,
                         absl::Duration max_exposure_time)
    : min_exposure_time_(std::min(min_exposure_time, max_exposure_time)),
      max_exposure_time_(std::max(min_exposure_time, max_exposure_time)),
      knee_point_exposure_percentages_(
          kKneePointExposurePercentagesPresetMedium.begin(),
          kKneePointExposurePercentagesPresetMedium.end()) {}

SoftwareHdr::SoftwareHdr(
    const CameraSettingProperties::Float::Range& exposure_time_range)
    : SoftwareHdr(absl::Microseconds(exposure_time_range.minimum),
                  absl::Microseconds(exposure_time_range.maximum)) {}

bool SoftwareHdr::CanHandleCameraSetting(std::string_view name) {
  return absl::c_any_of(
      kParametersDefiningHdrMode,
      [name](absl::string_view setting_name) { return setting_name == name; });
}

absl::StatusOr<CameraSettingAccess> SoftwareHdr::ReadCameraSettingAccess(
    std::string_view name) const {
  if (!CanHandleCameraSetting(name)) {
    return ::intrinsic::NotFoundErrorBuilder()
           << "Requested HDR setting '" << name << "' is not supported.";
  }
  return CreateCameraSettingAccess(name, CameraSettingAccess::Mode::kReadWrite);
}

absl::StatusOr<CameraSetting> SoftwareHdr::ReadCameraSetting(
    std::string_view name) const {
  if (name == genicam::kMultiSlopeMode) {
    return CreateEnumCameraSetting(name, HdrModeToStr(multi_slope_mode_));
  } else if (name == genicam::kMultiSlopeKneePointCount) {
    return CreateCameraSetting(name, knee_point_exposure_percentages_.size());
  } else if (name == genicam::kMultiSlopeKneePointSelector) {
    return CreateCameraSetting(name, active_knee_point_ + 1);
  } else if (name == genicam::kMultiSlopeExposureLimit) {
    return CreateCameraSetting(
        name, knee_point_exposure_percentages_.at(active_knee_point_));
  } else {
    return ::intrinsic::NotFoundErrorBuilder()
           << "Requested HDR setting '" << name << "' is not supported.";
  }
}

absl::StatusOr<CameraSettingProperties>
SoftwareHdr::ReadCameraSettingProperties(std::string_view name) const {
  if (name == genicam::kMultiSlopeMode) {
    std::vector<std::string> values;
    magic_enum::enum_for_each<HdrMode>([&values](HdrMode hdr_mode) {
      values.push_back(HdrModeToStr(hdr_mode));
    });
    return CameraSettingProperties{
        .name = std::string(name),
        .properties =
            CameraSettingProperties::Enumeration{.values = std::move(values)}};
  } else if (name == genicam::kMultiSlopeKneePointCount) {
    return CameraSettingProperties{
        .name = std::string(name),
        .properties = CameraSettingProperties::Integer{
            .range =
                CameraSettingProperties::Integer::Range{
                    .minimum = 1,
                    .maximum = std::numeric_limits<int64_t>::max()},
            .increment = 1}};
  } else if (name == genicam::kMultiSlopeKneePointSelector) {
    return CameraSettingProperties{
        .name = std::string(name),
        .properties = CameraSettingProperties::Integer{
            .range =
                CameraSettingProperties::Integer::Range{
                    .minimum = 1,
                    .maximum = static_cast<int64_t>(
                        knee_point_exposure_percentages_.size())},
            .increment = 1}};
  } else if (name == genicam::kMultiSlopeExposureLimit) {
    return CameraSettingProperties{
        .name = std::string(name),
        .properties = CameraSettingProperties::Float{
            .range =
                CameraSettingProperties::Float::Range{
                    .minimum = 0.0,
                    .maximum = std::numeric_limits<double>::max()},
            .increment = std::numeric_limits<double>::epsilon(),
            .unit = "%"}};
  } else {
    return ::intrinsic::NotFoundErrorBuilder()
           << "Requested HDR setting '" << name << "' is not supported.";
  }
}

absl::Status SoftwareHdr::UpdateCameraSetting(
    const CameraSetting& camera_setting) {
  if (camera_setting.name == genicam::kMultiSlopeMode) {
    INTR_ASSIGN_OR_RETURN(const std::string enum_value,
                          GetEnumerationValue(camera_setting));
    INTR_ASSIGN_OR_RETURN(multi_slope_mode_, StrToHdrMode(enum_value));
  } else if (camera_setting.name == genicam::kMultiSlopeKneePointCount) {
    INTR_ASSIGN_OR_RETURN(const int64_t knee_point_count,
                          GetValue<int64_t>(camera_setting));
    if (knee_point_count < 1) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "Number of knee-points '" << knee_point_count
             << "' is not >= 1 for HDR.";
    }
    knee_point_exposure_percentages_.resize(knee_point_count, 0.0);
    if (active_knee_point_ >= knee_point_exposure_percentages_.size()) {
      active_knee_point_ = 0;
    }
  } else if (camera_setting.name == genicam::kMultiSlopeKneePointSelector) {
    INTR_ASSIGN_OR_RETURN(const int64_t knee_point_selector,
                          GetValue<int64_t>(camera_setting));
    if (knee_point_selector <= 0) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "MultiSlopeKneePointSelector cannot be '" << knee_point_selector
             << "' as it should be > 0.";
    }
    if (knee_point_selector > knee_point_exposure_percentages_.size()) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "MultiSlopeKneePointSelector cannot be '" << knee_point_selector
             << "' as it is greater than number of knee-points '"
             << knee_point_exposure_percentages_.size() << "'.";
    }
    active_knee_point_ = knee_point_selector - 1;
  } else if (camera_setting.name == genicam::kMultiSlopeExposureLimit) {
    INTR_ASSIGN_OR_RETURN(const double exposure_limit,
                          GetValue<double>(camera_setting));
    if (exposure_limit < 0.0) {
      return ::intrinsic::InvalidArgumentErrorBuilder()
             << "MultiSlopeKneePointSelector '" << active_knee_point_ + 1
             << "' cannot have exposure time percentage of " << exposure_limit
             << ". It should be greater or equal to 0.";
    }
    knee_point_exposure_percentages_[active_knee_point_] = exposure_limit;
  } else {
    return ::intrinsic::NotFoundErrorBuilder()
           << "Requested HDR setting '" << camera_setting.name
           << "' is not supported.";
  }
  return absl::OkStatus();
}

absl::StatusOr<std::vector<CameraSetting>> SoftwareHdr::HdrExposureTimeSettings(
    const CameraSetting& init_exposure_time) const {
  if (init_exposure_time.name != genicam::kExposureTime) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "The passed camera setting must be named `"
           << genicam::kExposureTime << "`.";
  }
  INTR_ASSIGN_OR_RETURN(const double init_exposure_time_us,
                        GetValue<double>(init_exposure_time));

  const absl::Duration base_exposure_time =
      absl::Microseconds(init_exposure_time_us);

  if (base_exposure_time !=
      std::clamp(base_exposure_time, min_exposure_time_, max_exposure_time_)) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "The initial exposure time must be between " << min_exposure_time_
           << " and " << max_exposure_time_ << ".";
  }

  if (multi_slope_mode_ == HdrMode::kOff) {
    return std::vector<CameraSetting>{};
  }

  absl::btree_set<absl::Duration> sorted_unique_exposure_times;
  for (const double exposure_percentage : knee_point_exposure_percentages_) {
    absl::Duration exposure_time =
        std::clamp(exposure_percentage / 100.0 * base_exposure_time,
                   min_exposure_time_, max_exposure_time_);
    if (exposure_time > absl::ZeroDuration() &&
        absl::AbsDuration(exposure_time - base_exposure_time) >=
            absl::Nanoseconds(1)) {
      sorted_unique_exposure_times.insert(exposure_time);
    }
  }
  std::vector<CameraSetting> exposure_times;
  exposure_times.reserve(sorted_unique_exposure_times.size());
  for (const absl::Duration t : sorted_unique_exposure_times) {
    // Update exposure time.
    exposure_times.push_back({.name = genicam::kExposureTime,
                              .value = CameraSetting::Float{
                                  .value = absl::ToDoubleMicroseconds(t)}});
  }
  return exposure_times;
}

absl::StatusOr<HdrOperator> SoftwareHdr::HdrOperator() const {
  if (multi_slope_mode_ == HdrMode::kSoftwareMertens) {
    return HdrOperator::kMertens;
  } else {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Invalid HDR mode for software based HDR: "
           << HdrModeToStr(multi_slope_mode_);
  }
}

bool SoftwareHdr::IsEnabled() const {
  return multi_slope_mode_ != HdrMode::kOff;
}

}  // namespace intrinsic::perception
