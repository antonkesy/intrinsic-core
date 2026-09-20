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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_STOP_TRAJECTORY_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_STOP_TRAJECTORY_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace control {

// Compute stopping trajectory.
// Uses acceleration-limited trajectory.
// Use reflexxes-based JointVelocityReflexxes if you need jerk limits or
// synchronization of joints.
class JointStopTrajectory {
 public:
  JointStopTrajectory() = default;
  // Initialize trajectory generator; ndof = number of degrees of freedom.
  // Return false on error.
  bool Init(int ndof);

  // Return number of degrees of freedom / joints.
  int NumDoFs() const { return ndof_; }

  // Set maximum acceleration. Return false on error.
  bool SetAcceleration(const eigenmath::VectorNd& max_acceleration);

  // Set initial state at given time.
  // Return false on error.
  bool SetInitialConditions(const eigenmath::VectorNd& position,
                            const eigenmath::VectorNd& velocity, double time);

  // Get position and times at stopping locations (one stopping time per dof).
  // nullptr output arguments are ignored.
  // Return false on error.
  bool GetStopPositionAndTime(eigenmath::VectorNd* position,
                              eigenmath::VectorNd* time = nullptr);

  // Get position at specified time.
  // Assumes SetInitialConditions has been called.
  // Return false on error.
  bool GetPositionAtTime(double time, eigenmath::VectorNd* position);

  // Get velocity at specified time.
  // Assumes SetInitialConditions has been called.
  // Return false on error.
  bool GetVelocityAtTime(double time, eigenmath::VectorNd* velocity);

  // Get acceleration at specified time.
  // Assumes SetInitialConditions has been called.
  // Return false on error.
  bool GetAccelerationAtTime(double time, eigenmath::VectorNd* acceleration);

  // Get time when last joint is at rest.
  // Return false on error.
  bool GetFinalStopTime(double* time);

 private:
  eigenmath::VectorNd StopTimes() const;
  eigenmath::VectorNd StopPositions() const;
  int ndof_ = 0;
  eigenmath::VectorNd max_acceleration_;
  eigenmath::VectorNd initial_position_;
  eigenmath::VectorNd initial_velocity_;
  eigenmath::VectorNd stop_times_;
  eigenmath::VectorNd stop_positions_;
  double initial_time_;
  double final_stop_time_;
  bool initialized_ = false;
};

}  // namespace control
}  // namespace intrinsic
#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_STOP_TRAJECTORY_H_
