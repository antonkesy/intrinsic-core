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

#ifndef INTRINSIC_WORLD_UTIL_ROS_JOINT_STATE_UTIL_H_
#define INTRINSIC_WORLD_UTIL_ROS_JOINT_STATE_UTIL_H_

#include "absl/status/status.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "third_party/ros2/ros_interfaces/jazzy/sensor_msgs/msg/joint_state.pb.h"

namespace intrinsic {
namespace object_world {

// Sets the joint positions of the kinematic object based on the given ROS
// JointState message. The joints are matched by name.
absl::Status SetJointPositionsFromRos(
    KinematicObject& object,
    const ::sensor_msgs::msg::pb::jazzy::JointState& joint_state);

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_UTIL_ROS_JOINT_STATE_UTIL_H_
