
#include "third_party/abb_hardware_module/abb_hwm/rws_egm_utils.h"

#include <cstddef>
#include <iomanip>
#include <ios>
#include <regex>
#include <sstream>
#include <string>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/util/status/status_macros.h"

namespace {

std::string MakeJsonLikeString(const std::string& inp) {
  std::string content = inp;
  content = std::regex_replace(content, std::regex("TRUE"), "true");
  content = std::regex_replace(content, std::regex("FALSE"), "false");
  return content;
}

// Function to count decimal places in a string representation of a number
int CountDecimalPlaces(const std::string& number) {
  size_t decimalPos = number.find('.');
  if (decimalPos == std::string::npos) {
    return 0;  // No decimal point found
  }
  // Count digits after the decimal point
  return number.length() - decimalPos - 1;
}

std::string FormatFloatsInString(const std::string& input, int num_decimals) {
  std::regex float_regex(R"([-+]?\d*\.\d+)");  // Matches floating-point numbers
                                               // with a decimal point
  std::ostringstream output;
  std::smatch match;
  std::string::const_iterator search_start(input.cbegin());

  while (std::regex_search(search_start, input.cend(), match, float_regex)) {
    // Append the text leading up to the number
    output << match.prefix();

    std::string matched_number = match.str();
    int actual_decimals = CountDecimalPlaces(matched_number);

    // Only format if the number of decimals is greater than specified
    if (actual_decimals > num_decimals) {
      double num = std::stod(matched_number);
      output << std::fixed << std::setprecision(num_decimals) << num;
    } else {
      // Append the original number if it doesn't need formatting
      output << matched_number;
    }

    // Move the start position past the current match
    search_start = match.suffix().first;
  }

  // Append any remaining text after the last match
  output << std::string(search_start, input.cend());

  return output.str();
}

absl::StatusOr<std::string> MakeRapidString(const std::string& inp) {
  std::string content = inp;
  content = std::regex_replace(content, std::regex("true"), "TRUE");
  content = std::regex_replace(content, std::regex("false"), "FALSE");
  content = FormatFloatsInString(content,
                                 abb_hardware_module::rapid::kMaxDecimalPoints);
  return content;
}

}  // namespace

namespace abb_hardware_module {
namespace rapid {

absl::StatusOr<json> ParseEgmSettingsJsonFromRapidString(
    const std::string& inp) {
  std::string content = MakeJsonLikeString(inp);
  try {
    auto content_json = json::parse(content);
    return content_json;
  } catch (...) {
    return absl::InvalidArgumentError("Failed to parse JSON from input: " +
                                      content);
  }
}

absl::Status UpdateEgmSettingsJson(
    const ::intrinsic_proto::icon::EgmConfig& egm_config, json* settings) {
  if (egm_config.has_setup_uc_use_filtering()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kSetupUcUseFilteringFieldNumber,
        egm_config.setup_uc_use_filtering(), settings));
  }

  if (egm_config.has_setup_uc_comm_timeout()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kSetupUcCommTimeoutFieldNumber,
        egm_config.setup_uc_comm_timeout(), settings));
  }

  if (egm_config.has_activate_cond_min_max()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kActivateCondMinMaxFieldNumber,
        egm_config.activate_cond_min_max(), settings));
  }

  if (egm_config.has_activate_lp_filter()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kActivateLpFilterFieldNumber,
        egm_config.activate_lp_filter(), settings));
  }

  if (egm_config.has_activate_sample_time()) {
    // Check if the value is exactly 4, corresponding to 250 Hz
    // In the future when supporting slower control rates, modify
    // this to check that the sample time is a multiple of 4.
    if (egm_config.activate_sample_time() != 4) {
      return absl::InvalidArgumentError(
          "activate_sample_time must be set to exactly 4.");
    }
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kActivateSampleTimeFieldNumber,
        egm_config.activate_sample_time(), settings));
  }

  if (egm_config.has_activate_max_speed_deviation()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::
            kActivateMaxSpeedDeviationFieldNumber,
        egm_config.activate_max_speed_deviation(), settings));
  }

  if (egm_config.has_run_cond_time()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kRunCondTimeFieldNumber,
        egm_config.run_cond_time(), settings));
  }

  if (egm_config.has_run_ramp_in_time()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kRunRampInTimeFieldNumber,
        egm_config.run_ramp_in_time(), settings));
  }

  if (egm_config.has_run_pos_corr_gain()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kRunPosCorrGainFieldNumber,
        egm_config.run_pos_corr_gain(), settings));
  }

  if (egm_config.has_stop_ramp_out_time()) {
    INTR_RETURN_IF_ERROR(SetEgmSettingsJsonField(
        ::intrinsic_proto::icon::EgmConfig::kStopRampOutTimeFieldNumber,
        egm_config.stop_ramp_out_time(), settings));
  }

  return absl::OkStatus();
}

absl::StatusOr<std::string> ApplyEgmConfigToEgmSettingsRapidString(
    const std::string& current_string,
    const ::intrinsic_proto::icon::EgmConfig& egm_config) {
  INTR_ASSIGN_OR_RETURN(json settings,
                        ParseEgmSettingsJsonFromRapidString(current_string));
  INTR_RETURN_IF_ERROR(UpdateEgmSettingsJson(egm_config, &settings));
  return MakeRapidString(settings.dump());
}

}  // namespace rapid

}  // namespace abb_hardware_module
