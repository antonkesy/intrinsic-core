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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_TRAJECTORY_TRACKING_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_TRAJECTORY_TRACKING_ACTION_H_

#include <limits>
#include <memory>
#include <optional>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/trajectory_tracking_action.pb.h"
#include "intrinsic/icon/actions/trajectory_tracking_action_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/algorithms/trajectory_player.h"
#include "intrinsic/icon/control/algorithms/trajectory_residual_controller.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"

namespace intrinsic::icon {

// Follows a given intrinsic::JointTrajectoryPVA and uses quintic splining to
// interpolate between the discretized states. The previously commanded
// robot position/velocity setpoint must be within
// `kMaxInitialJointPositionDeviation`/`kMaxInitialJointVelocityDeviation` of
// the first joint state of the command. A residual controller is applied to
// compensate for initial state deviations smaller than above thresholds.
class TrajectoryTrackingAction final : public RtclActionInterface {
 public:
  struct Params {
    // The joint position-velocity-acceleration trajectory to be played back on
    // the robot. Time stamps must start at zero. Time stamps are not required
    // to increase uniformly, however the trajectory is expected to be
    // dynamically/kinematically consistent. The action does not perform
    // additional consistency checks beyond limit checking, the system may be at
    // risk if the input trajectory is inconsistent.
    JointTrajectoryPVA trajectory;

    static absl::StatusOr<Params> FromProto(
        const intrinsic_proto::icon::actions::
            TrajectoryTrackingActionFixedParams& proto_params);
  };

  // Constructs a TrajectoryTrackingAction from `slot_id`, `trajectory_player`
  // and a `trajectory_residual_controller`.
  TrajectoryTrackingAction(
      RealtimeSlotId slot_id,
      std::unique_ptr<TrajectoryPlayer> trajectory_player,
      std::unique_ptr<TrajectoryResidualController>
          trajectory_residual_controller,
      std::unique_ptr<IsSettledCriterion> is_settled_criterion,
      RealtimeSignalId path_accurate_stop_signal_id,
      const JointLimits& planning_limits, const JointLimits& system_limits);

  static absl::StatusOr<std::unique_ptr<TrajectoryTrackingAction>> Create(
      const TrajectoryTrackingActionInfo::FixedParams& params_proto,
      ActionFactoryContext& context);

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

 private:
  // IsDone reports `true` if the trajectory has been fully played back
  // (trajectory phase is 1.0) or if a path-accurate stop has been triggered and
  // the speed override state internal to the TrajectoryPlayer has reached zero.
  bool IsDone() const;

  const RealtimeSlotId slot_id_;

  std::unique_ptr<TrajectoryPlayer> trajectory_player_;
  std::unique_ptr<TrajectoryResidualController> residual_controller_;
  JointStatePVA initial_state_residual_;
  std::optional<JointStatePVA> final_state_residual_;
  JointLimits planning_limits_;
  JointLimits system_limits_;

  double distance_to_final_setpoint_ = std::numeric_limits<double>::max();
  double trajectory_done_for_seconds_ = 0.0;
  // The wall time passed since the trajectory was started. This is does not
  // correspond to the time parametrization of the trajectory.
  // wall_time_passed_since_trajectory_start_ increases by the time_step every
  // Sense cycle and is not affected by speed override which mean the
  // trajectory's time parametrization is advancing more slowly than the wall
  // time.
  double wall_time_passed_since_trajectory_start_ = 0.0;

  // Internal variables corresponding to the evaluation of the settled state.
  bool is_settled_ = false;
  std::unique_ptr<IsSettledCriterion> is_settled_criterion_;

  RealtimeSignalId path_accurate_stop_signal_id_;
  SignalValue path_accurate_stop_signal_value_;
  bool path_accurate_stop_requested_ = false;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_TRAJECTORY_TRACKING_ACTION_H_
