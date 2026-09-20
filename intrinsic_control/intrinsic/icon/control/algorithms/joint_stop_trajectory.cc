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

#include "intrinsic/icon/control/algorithms/joint_stop_trajectory.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/log.h"

namespace intrinsic {
namespace control {

bool JointStopTrajectory::Init(const int ndof) {
  if (ndof <= 0) {
    INTRINSIC_RT_LOG(ERROR) << "ndof needs to be > 0, got " << ndof;
    return false;
  }
  ndof_ = ndof;
  max_acceleration_.resize(ndof_);
  initial_position_.resize(ndof_);
  initial_velocity_.resize(ndof_);
  stop_times_.resize(ndof_);
  stop_positions_.resize(ndof_);

  max_acceleration_.setZero();
  initial_position_.setZero();
  initial_velocity_.setZero();
  stop_times_.setZero();
  stop_positions_.setZero();

  initialized_ = false;
  return true;
}

bool JointStopTrajectory::SetAcceleration(
    const eigenmath::VectorNd& max_acceleration) {
  if (max_acceleration.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Dimension error: max_acceleration size " << max_acceleration.size()
        << ", expected " << ndof_;
    return false;
  }
  if (max_acceleration.minCoeff() <= 0) {
    INTRINSIC_RT_LOG(ERROR)
        << "All acceleration coefficients must be > 0, smallest one is "
        << max_acceleration.minCoeff();
    return false;
  }
  max_acceleration_ = max_acceleration;
  return true;
}

bool JointStopTrajectory::SetInitialConditions(
    const eigenmath::VectorNd& position, const eigenmath::VectorNd& velocity,
    const double time) {
  if (position.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR) << "Dimension error: position size "
                            << position.size() << ", expected " << ndof_;
    return false;
  }
  if (velocity.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR) << "Dimension error: velocity size "
                            << velocity.size() << ", expected " << ndof_;
    return false;
  }

  initial_position_ = position;
  initial_velocity_ = velocity;
  initial_time_ = time;

  stop_times_ = StopTimes();
  final_stop_time_ = stop_times_.maxCoeff();
  stop_positions_ = StopPositions();

  initialized_ = true;
  return true;
}

eigenmath::VectorNd JointStopTrajectory::StopTimes() const {
  return initial_velocity_.cwiseAbs().cwiseQuotient(max_acceleration_) +
         eigenmath::VectorNd::Ones(ndof_) * initial_time_;
}

eigenmath::VectorNd JointStopTrajectory::StopPositions() const {
  eigenmath::VectorNd acc(ndof_);
  for (int dof = 0; dof < ndof_; dof++) {
    if (initial_velocity_[dof] <= 0) {
      acc[dof] = max_acceleration_[dof];
    } else {
      acc[dof] = -max_acceleration_[dof];
    }
  }
  eigenmath::VectorNd delta(ndof_);
  const eigenmath::VectorNd& vel = initial_velocity_;
  delta = -0.5 * vel.cwiseProduct(vel).cwiseQuotient(acc);
  eigenmath::VectorNd result(initial_position_ + delta);

  return result;
}

bool JointStopTrajectory::GetStopPositionAndTime(eigenmath::VectorNd* position,
                                                 eigenmath::VectorNd* time) {
  if (!initialized_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Call SetInitialConditions before GetStopPositionAndTime";
    return false;
  }

  if (position == nullptr && time == nullptr) {
    return true;
  }

  if (position != nullptr) {
    if (position->size() != ndof_) {
      INTRINSIC_RT_LOG(ERROR) << "Dimension error: position size "
                              << position->size() << ", expected " << ndof_;
      return false;
    }
    *position = stop_positions_;
  }
  if (time != nullptr) {
    if (time->size() != ndof_) {
      INTRINSIC_RT_LOG(ERROR) << "Dimension error: time size " << time->size()
                              << ", expected " << ndof_;
      return false;
    }
    *time = stop_times_;
  }

  return true;
}

bool JointStopTrajectory::GetPositionAtTime(double time,
                                            eigenmath::VectorNd* position) {
  if (!initialized_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Call SetInitialConditions before GetPositionAtTime";
    return false;
  }
  if (position == nullptr) {
    INTRINSIC_RT_LOG(ERROR) << "position == nullptr";
    return false;
  }
  if (time < initial_time_) {
    INTRINSIC_RT_LOG(ERROR) << "time must be > initial_time, but " << time
                            << " < " << initial_time_;
    return false;
  }

  eigenmath::VectorNd acc(ndof_);
  for (int dof = 0; dof < ndof_; dof++) {
    if (initial_velocity_[dof] <= 0) {
      acc[dof] = max_acceleration_[dof];
    } else {
      acc[dof] = -max_acceleration_[dof];
    }
  }
  const double trel = time - initial_time_;
  const double trel2 = trel * trel;

  for (int dof = 0; dof < ndof_; dof++) {
    if (time >= stop_times_[dof]) {
      (*position)[dof] = stop_positions_[dof];
    } else {
      (*position)[dof] = 0.5 * acc[dof] * trel2 +
                         initial_velocity_[dof] * trel + initial_position_[dof];
    }
  }
  return true;
}

bool JointStopTrajectory::GetVelocityAtTime(double time,
                                            eigenmath::VectorNd* velocity) {
  if (!initialized_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Call SetInitialConditions before GetVelocityAtTime";
    return false;
  }
  if (velocity == nullptr) {
    INTRINSIC_RT_LOG(ERROR) << "velocity == nullptr";
    return false;
  }
  if (time < initial_time_) {
    INTRINSIC_RT_LOG(ERROR) << "time must be > initial_time, but " << time
                            << " < " << initial_time_;
    return false;
  }

  eigenmath::VectorNd acc(ndof_);

  for (int dof = 0; dof < ndof_; dof++) {
    if (initial_velocity_[dof] <= 0) {
      acc[dof] = max_acceleration_[dof];
    } else {
      acc[dof] = -max_acceleration_[dof];
    }
  }
  const double trel = time - initial_time_;

  for (int dof = 0; dof < ndof_; dof++) {
    if (time >= stop_times_[dof]) {
      (*velocity)[dof] = 0.0;
    } else {
      (*velocity)[dof] = acc[dof] * trel + initial_velocity_[dof];
    }
  }
  return true;
}

bool JointStopTrajectory::GetAccelerationAtTime(
    double time, eigenmath::VectorNd* acceleration) {
  if (!initialized_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Call SetInitialConditions before GetAccelerationAtTime";
    return false;
  }
  if (acceleration == nullptr) {
    INTRINSIC_RT_LOG(ERROR) << "acceleration == nullptr";
    return false;
  }
  if (time < initial_time_) {
    INTRINSIC_RT_LOG(ERROR) << "time must be > initial_time, but " << time
                            << " < " << initial_time_;
    return false;
  }

  for (int dof = 0; dof < ndof_; dof++) {
    if (time >= stop_times_[dof]) {
      (*acceleration)[dof] = 0.0;
    } else {
      if (initial_velocity_[dof] <= 0) {
        (*acceleration)[dof] = max_acceleration_[dof];
      } else {
        (*acceleration)[dof] = -max_acceleration_[dof];
      }
    }
  }
  return true;
}

bool JointStopTrajectory::GetFinalStopTime(double* time) {
  if (!initialized_) {
    INTRINSIC_RT_LOG(ERROR)
        << "Call SetInitialConditions before GetFinalStopTime";
    return false;
  }
  if (time == nullptr) {
    INTRINSIC_RT_LOG(ERROR) << "time == nullptr";
    return false;
  }

  *time = final_stop_time_;
  return true;
}

}  // namespace control
}  // namespace intrinsic
