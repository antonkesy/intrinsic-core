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

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/algorithms/wrench_stability_monitor_factory.h"
#include "intrinsic/icon/equipment/force_control_settings.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

namespace {

// Idle implementation of `WrenchStabilityMonitor` used in open-source / IOC
// builds. This monitor performs no analysis and always reports a stability
// index of 0.0., which implies a stable wrench.
class IdleWrenchStabilityMonitor : public WrenchStabilityMonitor {
 public:
  IdleWrenchStabilityMonitor() = default;

  RealtimeStatus Update(const Wrench& /*wrench*/) override {
    return OkStatus();
  }

  void Reset() override {}

  double GetStabilityIndex() const override { return 0.0; }
};

}  // namespace

absl::StatusOr<std::unique_ptr<WrenchStabilityMonitor>>
CreateWrenchStabilityMonitor(
    const intrinsic_proto::icon::ForceControlSettings& /*settings*/,
    const double /*control_frequency_hz*/) {
  return std::make_unique<IdleWrenchStabilityMonitor>();
}

}  // namespace intrinsic::icon
