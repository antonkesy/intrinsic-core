#include "third_party/abb_hardware_module/utils/rate_monitor.h"

#include "absl/status/status.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/duration.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace {

double kMinAllowedDtNs = 1e3;
}

namespace abb_hardware_module {

absl::Status RateMonitor::Init(double min_rate_hz, double max_rate_hz) {
  if (min_rate_hz <= 0 || max_rate_hz <= 0) {
    return absl::InvalidArgumentError(
        "min_rate_hz and max_rate_hz must be greater than 0.");
  }

  if (min_rate_hz > max_rate_hz) {
    return absl::InvalidArgumentError(
        "min_rate_hz must be less than or equal to max_rate_hz.");
  }

  min_rate_hz_ = min_rate_hz;
  max_rate_hz_ = max_rate_hz;
  has_ticked_ = false;
  has_initialized_ = true;
  return absl::OkStatus();
}

intrinsic::icon::RealtimeStatus RateMonitor::Tick() {
  if (!has_initialized_) {
    return intrinsic::icon::FailedPreconditionError(
        "RateMonitor::Init(min_freq, max_freq) must be called before "
        "RateMonitor::Tick().");
  }

  if (!has_ticked_) {
    last_tick_ = intrinsic::Clock::Now();
    has_ticked_ = true;
    return intrinsic::icon::OkStatus();
  }

  double ns_since_last_tick =
      intrinsic::ToDoubleNanoseconds(intrinsic::Clock::Now() - last_tick_);
  last_tick_ = intrinsic::Clock::Now();

  if (ns_since_last_tick <= kMinAllowedDtNs) {
    return intrinsic::icon::InternalError(
        intrinsic::icon::FixedStrCat<
            intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
            "RateMonitor::Tick() called too quickly. The time since the last "
            "tick is",
            ns_since_last_tick / 1e3,
            " μs, which is less than minimum allowed value ",
            kMinAllowedDtNs / 1e3, " μs."));
  }
  double observed_frequency_hz = 1.e9 / ns_since_last_tick;

  if (observed_frequency_hz > max_rate_hz_) {
    return intrinsic::icon::InternalError(
        intrinsic::icon::FixedStrCat<
            intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
            "Too high rate observed. Observed rate was ", observed_frequency_hz,
            " Hz, which is higher than the accepted ", max_rate_hz_, " Hz"));
  }
  if (observed_frequency_hz < min_rate_hz_) {
    return intrinsic::icon::DeadlineExceededError(
        intrinsic::icon::FixedStrCat<
            intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
            "Too low rate observed. Observed rate was ", observed_frequency_hz,
            " Hz, which is lower than the accepted ", min_rate_hz_, " Hz"));
  }
  return intrinsic::icon::OkStatus();
}

}  // namespace abb_hardware_module
