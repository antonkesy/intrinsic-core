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

#ifndef INTRINSIC_WORLD_OBJECTS_PRINT_OBJECT_WORLD_H_
#define INTRINSIC_WORLD_OBJECTS_PRINT_OBJECT_WORLD_H_

#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/world/objects/object_world.h"

namespace intrinsic::object_world {

// Returns a printed summary of the object and frames in the given object-view
// of a world. Example result:
//
// """
// => root (RootObject)
//   => robot (KinematicObject)
//     => gripper (KinematicObject)
//     -> robot.flange (Frame)
//   => workpiece (PhysicalObject)
//     -> workpiece.grasp (Frame)
// """
absl::StatusOr<std::string> PrintObjectWorldSummary(const ObjectWorld& world,
                                                    bool enable_colors = false);

// Returns a printed mapping from attachment entities of a world to objects in
// the given object-view of this world. Includes a short description of the
// output format. Example result:
//
// """
// Mapping from attachment entities to objects:
//
// Format: [<object/frame name>*] <entity id> <further entity details>
//   where <object/frame name> == <object name> or <object name>.<frame name>
//
// [root] 1, local_name=Root, labels={world_origin}
//  [robot] 24, local_name=base_link, labels={my_robot::arm, robot_base}
//   [robot] 31, local_name=shoulder_pan_joint, labels={my_robot::arm}
//    [robot] 25, local_name=shoulder_link, labels={my_robot::arm}
//     [robot] 32, local_name=shoulder_lift_joint, labels={my_robot::arm}
//      [robot] 26, local_name=upper_arm_link, labels={my_robot::arm}
//       [robot] 33, local_name=elbow_joint, labels={my_robot::arm}
//        [robot] 27, local_name=forearm_link, labels={my_robot::arm}
//         [robot] 34, local_name=wrist_1_joint, labels={my_robot::arm}
//          [robot] 28, local_name=wrist_1_link, labels={my_robot::arm}
//           [robot] 35, local_name=wrist_2_joint, labels={my_robot::arm}
//            [robot] 29, local_name=wrist_2_link, labels={my_robot::arm}
//             [robot] 36, local_name=wrist_3_joint, labels={my_robot::arm}
//              [robot] 30, local_name=wrist_3_link, labels={my_robot::arm, tip}
//               [robot.flange] 37, local_name=flange
//               [gripper] 47, alias=gripper, local_name=gripper_body,
//                             labels={my_robot::my_gripper}
//                [gripper] 51, local_name=finger1,
//                              labels={my_robot::my_gripper}
//  [workpiece] 20, alias=workpiece, local_name=whole, labels={workpiece}
//   [workpiece.grasp] 64, alias=grasp, local_name=grasp
// """
absl::StatusOr<std::string> PrintEntityToObjectMapping(
    const ObjectWorld& world, bool enable_colors = false);

}  // namespace intrinsic::object_world

#endif  // INTRINSIC_WORLD_OBJECTS_PRINT_OBJECT_WORLD_H_
