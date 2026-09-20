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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_FORCE_TORQUE_SENSOR_PART_FORCE_SENSOR_CONTROLLER_MIGRATION_UTILS_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_FORCE_TORQUE_SENSOR_PART_FORCE_SENSOR_CONTROLLER_MIGRATION_UTILS_H_

#include <cstdlib>
#include <ostream>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hal/interfaces/force_sensor.fbs.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

const double kPayloadChangePrecision = 1e-5;

// Post-sensor payload/inertia, i.e. only the payload that the FT-sensor is
// supporting.
struct PostSensorPayload {
  // Vector from the FT sensor to the center of gravity.
  eigenmath::Vector3d ft_t_cog = eigenmath::Vector3d::Zero();
  // Mass in kg that the force/torque sensor is supporting.
  double mass_kg = 0.0;

  // Combine two payloads correctly to a combined payload by shifting the
  // center of gravity according to each individual mass.
  PostSensorPayload operator+(const PostSensorPayload& other) const;
};

// Can be used to count the number of (nearly) constant sensor readings in a
// row.
template <typename VectorType>
class ConstSensorReadingsCounter {
 public:
  explicit ConstSensorReadingsCounter(double const_readings_threshold = 1e-9)
      : const_readings_threshold_(const_readings_threshold),
        num_constant_readings_(0) {}

  void AddReading(const VectorType& v_prev, const VectorType& v_new) {
    // Check proximity of two of the same values in a row per dof.
    bool small_values = (v_prev - v_new).template lpNorm<Eigen::Infinity>() <
                        const_readings_threshold_;
    if (small_values) {
      num_constant_readings_++;
    } else {
      num_constant_readings_ = 0;
    }
  }

  int num_constant_readings() const { return num_constant_readings_; }

 private:
  double const_readings_threshold_;
  int num_constant_readings_;
};

// Computes the quasi-static wrench on a F/T sensor due to the
// `support_mass_kg` that the FT sensor is supporting. Captures only gravity
// effects. `robot_T_ft` is the pose of the FT sensor relative to the world
// frame. `ft_p_cog` is the vector from the FT sensor to the center of gravity
// of the support mass. Returns wrench on the F/T sensor due to supported mass.
Wrench ComputeWrenchFtSupportMass(double support_mass_kg,
                                  const Pose3d& robot_T_ft,
                                  const eigenmath::Vector3d& ft_p_cog);

// Computes the dynamic load on the F/T sensor due to the mass that the FT
// sensor is supporting. Captures only dynamic effects such as centrifugal and
// Euler forces. Coriolis effects are neglected.
// `ft_twist` is the twist of the FT-sensor frame expressed in local
// coordinates, the FT-sensor frame.
// `ft_acceleration` is the acceleration of the FT-sensor frame expressed in
// the FT-sensor frame.
// `support_mass` is the mass in kg.
// `robot_T_ft` is the  pose of the FT sensor relative to the world frame.
// `ft_p_cog` is the pose of the CoM w.r.t. the FT-sensor frame.
// Returns wrench on the F/T sensor due to mass and dynamic effects,
// expressed in FT-sensor frame.
// TODO(b/174645393): extract and isolate libraries to googlex directory.
// TODO(b/182244422): add mini-benchmark.
Wrench ComputePostSensorDynamicLoadAtFTSensor(
    const Twist& ft_twist, const eigenmath::Vector6d& ft_acceleration,
    double support_mass_kg, const intrinsic::Pose3d& robot_T_ft,
    const eigenmath::Vector3d& ft_p_cog);

struct ComputePostSensorDynamicsResult {
  Wrench dynamic_load_at_ft;
  Wrench dynamic_load_at_target;
};

RealtimeStatusOr<ComputePostSensorDynamicsResult> ComputePostSensorDynamicLoad(
    const eigenmath::VectorNd& current_joint_velocity,
    const eigenmath::VectorNd& current_joint_acceleration,
    kinematics::ElementId target_link_element_id, const Pose3d& base_t_target,
    const Pose3d& target_t_ft_sensor,
    const PostSensorPayload& total_post_sensor_payload,
    kinematics::State* kinematics_state);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_FORCE_TORQUE_SENSOR_PART_FORCE_SENSOR_CONTROLLER_MIGRATION_UTILS_H_
