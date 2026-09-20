#ifndef ABB_HARDWARE_MODULE_UTILS_RATE_MONITOR_H_
#define ABB_HARDWARE_MODULE_UTILS_RATE_MONITOR_H_

#include <cstdint>

#include "absl/status/status.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace abb_hardware_module {

// RateMonitor is utilty for monitoring the rate of loops.
// The Init method should be called outside of the monitored loop. It resets
// timing and sets the minimum and maximum tolerated rate. The Tick method
// should be called inside the loop. It returns a RealtimeStatus indicating a
// rate above the maximum with kInternal, and a rate below the minimum with
// kDeadlineExceeded, and a rate within the limits with kOk.
//
// Example usage:
// in init-routine outside of loop:
// RateMonitor rate_monitor;
// rate_monitor.Init(min_rate_hz, max_rate_hz);
//
// in loop:
// RealtimeStatus status = rate_monitor.Tick();
// if (intrinsic::icon::IsInternal(status)) {
//     // handle too high rate
// } else if (intrinsic::icon::IsDeadlineExceeded(status)) {
//     // handle too low rate
// }
class RateMonitor {
 public:
  RateMonitor() : has_ticked_(false), has_initialized_(false) {}

  absl::Status Init(double min_rate_hz,
                    double max_rate_hz) INTRINSIC_NON_REALTIME_ONLY;
  intrinsic::icon::RealtimeStatus Tick() INTRINSIC_CHECK_REALTIME_SAFE;

 private:
  double max_rate_hz_;
  double min_rate_hz_;
  intrinsic::Clock::time_point last_tick_;
  bool has_ticked_;
  bool has_initialized_;
};

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_UTILS_RATE_MONITOR_H_
