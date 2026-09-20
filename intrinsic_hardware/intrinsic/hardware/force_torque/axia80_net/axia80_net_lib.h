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

#ifndef INCODE_INTRINSIC_HARDWARE_INTRINSIC_HARDWARE_FORCE_TORQUE_AXIA80_NET_AXIA80_NET_LIB_H_
#define INCODE_INTRINSIC_HARDWARE_INTRINSIC_HARDWARE_FORCE_TORQUE_AXIA80_NET_AXIA80_NET_LIB_H_

#include <time.h>

#include <cstdint>

#include "absl/status/status.h"
#include "intrinsic/hardware/force_torque/axia80_net/axia80_net.pb.h"
#include "intrinsic/hardware/gripper/wsg32/async_buffer.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/sensor_utils.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::ati {

namespace internal {

struct Response {
  uint32_t rdt_sequence;
  uint32_t ft_sequence;
  uint32_t status;
  int32_t ft_data[6];
  timespec last_receive_time_monotonic;
};

}  // namespace internal

class Axia80NetHwm final : public intrinsic::icon::HardwareModuleInterface {
 public:
  Axia80NetHwm() = default;

  static constexpr char kHwModuleTypeName[] = "Axia80NetHwm";

  absl::Status Init(
      intrinsic::icon::HardwareModuleInitContext& init_context) override;

  absl::Status InitInternal(
      const ::intrinsic_proto::icon::Axia80NetConfig& config,
      const ThreadOptions& thread_options);

  absl::Status Prepare() override;

  icon::RealtimeStatus Activate() override;

  icon::RealtimeStatus Deactivate() override;

  absl::Status EnableMotion() override;

  absl::Status DisableMotion() override;

  absl::Status ClearFaults() override;

  absl::Status Shutdown() override;

  icon::RealtimeStatus ReadStatus() override;

  icon::RealtimeStatus ApplyCommand() override;

 private:
  icon::MutableHardwareInterfaceHandle<intrinsic_fbs::ForceTorqueStatus>
      status_handle_;
  icon::HardwareInterfaceHandle<intrinsic_fbs::ForceTorqueCommand>
      command_handle_;

  // Thread management.
  ThreadOptions thread_options_;
  ::intrinsic::Thread thread_;
  wsg32::AsyncBuffer<internal::Response> status_buffer_;

  ::intrinsic::icon::TaringData taring_data_;
  bool enabled_ = false;

  ::intrinsic_proto::icon::Axia80NetConfig config_;
};

}  // namespace intrinsic::ati

#endif  // INCODE_INTRINSIC_HARDWARE_INTRINSIC_HARDWARE_FORCE_TORQUE_AXIA80_NET_AXIA80_NET_LIB_H_
