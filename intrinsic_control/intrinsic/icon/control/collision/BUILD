# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("//bazel:cc_macros.bzl", "cc_library")

package(default_visibility = ["//intrinsic_control/intrinsic/icon/control:__subpackages__"])

cc_library(
    name = "collision_error",
    hdrs = ["collision_error.h"],
)

cc_library(
    name = "collision_data_types",
    hdrs = ["collision_data_types.h"],
    deps = ["@com_gitlab_libeigen_eigen//:eigen"],
)

cc_library(
    name = "collision_utility_functions",
    srcs = ["collision_utility_functions.cc"],
    hdrs = ["collision_utility_functions.h"],
    deps = [
        ":collision_data_types",
    ],
)

cc_library(
    name = "collision_object",
    hdrs = ["collision_object.h"],
    deps = [
        ":collision_data_types",
        ":collision_error",
        "//intrinsic_control/intrinsic/icon/utils:realtime_guard",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_or",
    ],
)

cc_library(
    name = "collision",
    srcs = ["collision_world.cc"],
    hdrs = ["collision_world.h"],
    deps = [
        ":collision_data_types",
        ":collision_error",
        ":collision_object",
        "//intrinsic_control/intrinsic/icon/utils:realtime_guard",
        "@abseil-cpp//absl/log:check",
        "@com_gitlab_libeigen_eigen//:eigen",
    ],
)

cc_library(
    name = "collisionloader",
    srcs = ["collision_world_loader.cc"],
    hdrs = ["collision_world_loader.h"],
    deps = [
        ":collision",
        ":collision_data_types",
        ":collision_error",
        ":collision_object",
        ":collision_utility_functions",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/container:flat_hash_set",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/log:check",
    ],
)

cc_library(
    name = "robotcollision",
    srcs = ["robot_collision_checker.cc"],
    hdrs = ["robot_collision_checker.h"],
    deps = [
        ":collision",
        ":collision_object",
        ":collisionloader",
        "//intrinsic_control/intrinsic/icon/testing:realtime_annotations",
        "//intrinsic_control/intrinsic/icon/utils:log",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_macro",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status_or",
        "//intrinsic_kinematics/intrinsic/kinematics",
        "//intrinsic_kinematics/intrinsic/kinematics:skeleton",
        "//intrinsic_kinematics/intrinsic/kinematics:state",
        "//intrinsic_sdk/intrinsic/eigenmath",
        "//intrinsic_sdk/intrinsic/math:pose3",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/log:check",
    ],
)
