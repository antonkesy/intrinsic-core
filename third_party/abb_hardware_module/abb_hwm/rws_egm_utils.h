#ifndef ABB_HARDWARE_MODULE_ABB_HWM_RWS_EGM_UTILS_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_RWS_EGM_UTILS_H_

#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "nlohmann/json.hpp"
#include "third_party/abb_hardware_module/abb_hwm/abb_config.pb.h"

namespace abb_hardware_module {
namespace rapid {
using json = nlohmann::json;
using EgmConfig = ::intrinsic_proto::icon::EgmConfig;

constexpr size_t kMaxDecimalPoints = 5;

constexpr std::pair<size_t, size_t> EgmSettingsFieldPositionInJson(
    const int field) INTRINSIC_CHECK_REALTIME_SAFE {
  switch (field) {
    case EgmConfig::kSetupUcUseFilteringFieldNumber:
      return {2, 0};
    case EgmConfig::kSetupUcCommTimeoutFieldNumber:
      return {2, 1};
    case EgmConfig::kActivateCondMinMaxFieldNumber:
      return {3, 4};
    case EgmConfig::kActivateLpFilterFieldNumber:
      return {3, 5};
    case EgmConfig::kActivateSampleTimeFieldNumber:
      return {3, 6};
    case EgmConfig::kActivateMaxSpeedDeviationFieldNumber:
      return {3, 7};
    case EgmConfig::kRunCondTimeFieldNumber:
      return {4, 0};
    case EgmConfig::kRunRampInTimeFieldNumber:
      return {4, 1};
    case EgmConfig::kRunPosCorrGainFieldNumber:
      return {4, 3};
    case EgmConfig::kStopRampOutTimeFieldNumber:
      return {5, 0};
    default:
      // field not implemented!
      return {-1, -1};
  }
}

template <typename T>
absl::Status SetEgmSettingsJsonField(const int field_number, const T& value,
                                     json* settings)
    INTRINSIC_NON_REALTIME_ONLY {
  auto [section, index] = EgmSettingsFieldPositionInJson(field_number);
  if (section == -1) {
    return absl::InvalidArgumentError(
        absl::StrCat("SetEgmSettingsJsonField: Field number ", field_number,
                     " not implemented"));
  }
  try {
    (*settings)[section][index] = value;
    return absl::OkStatus();
  } catch (std::exception& e) {
    return absl::InternalError(e.what());
  }
}

absl::Status UpdateEgmSettingsJson(
    const ::intrinsic_proto::icon::EgmConfig& egm_config,
    json* settings) INTRINSIC_NON_REALTIME_ONLY;
absl::StatusOr<std::string> ApplyEgmConfigToEgmSettingsRapidString(
    const std::string& current_value,
    const ::intrinsic_proto::icon::EgmConfig& egm_config)
    INTRINSIC_NON_REALTIME_ONLY;
absl::StatusOr<json> ParseEgmSettingsJsonFromRapidString(const std::string& inp)
    INTRINSIC_NON_REALTIME_ONLY;

enum class StateMachineState : uint8_t {
  kIdle = 0,
  kInitialize = 1,
  kRunRapidRoutine = 2,
  kRunEgmRoutine = 3,
};
}  // namespace rapid

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_RWS_EGM_UTILS_H_
