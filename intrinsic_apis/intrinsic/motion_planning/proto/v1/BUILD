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

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "compute_ik_proto",
    srcs = ["compute_ik.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_proto",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_proto",
    ],
)

cc_proto_library(
    name = "compute_ik_cc_proto",
    deps = [":compute_ik_proto"],
)

go_proto_library(
    name = "compute_ik_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/compute_ik_go_proto",
    proto = ":compute_ik_proto",
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_go_proto",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_go_proto",
    ],
)

py_proto_library(
    name = "compute_ik_py_pb2",
    deps = [":compute_ik_proto"],
)

proto_library(
    name = "geometric_constraints_proto",
    srcs = ["geometric_constraints.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_proto",
        "@intrinsic_apis//intrinsic/math/proto:point_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_proto",
        "@intrinsic_apis//intrinsic/math/proto:vector3_proto",
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_proto",
    ],
)

cc_proto_library(
    name = "geometric_constraints_cc_proto",
    deps = [":geometric_constraints_proto"],
)

go_proto_library(
    name = "geometric_constraints_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/geometric_constraints_go_proto",
    proto = ":geometric_constraints_proto",
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:point_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:vector3_go_proto",
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_go_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_go_proto",
    ],
)

py_proto_library(
    name = "geometric_constraints_py_pb2",
    deps = [":geometric_constraints_proto"],
)

proto_library(
    name = "motion_blending_parameter_proto",
    srcs = ["motion_blending_parameter.proto"],
    deps = ["@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_proto"],
)

cc_proto_library(
    name = "motion_blending_parameter_cc_proto",
    deps = [":motion_blending_parameter_proto"],
)

go_proto_library(
    name = "motion_blending_parameter_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/motion_blending_parameter_go_proto",
    proto = ":motion_blending_parameter_proto",
    deps = [
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_go_proto",
    ],
)

py_proto_library(
    name = "motion_blending_parameter_py_pb2",
    deps = [":motion_blending_parameter_proto"],
)

proto_library(
    name = "motion_planner_config_proto",
    srcs = ["motion_planner_config.proto"],
    deps = [
        "@com_google_protobuf//:duration_proto",
        "@intrinsic_apis//intrinsic/world/proto:collision_checker_config_proto",
    ],
)

cc_proto_library(
    name = "motion_planner_config_cc_proto",
    deps = [":motion_planner_config_proto"],
)

go_proto_library(
    name = "motion_planner_config_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/motion_planner_config_go_proto",
    proto = ":motion_planner_config_proto",
    deps = [
        "@intrinsic_apis//intrinsic/world/proto:collision_checker_config_go_proto",
    ],
)

py_proto_library(
    name = "motion_planner_config_py_pb2",
    deps = [":motion_planner_config_proto"],
)

proto_library(
    name = "motion_planning_proto",
    srcs = ["motion_planning.proto"],
    deps = ["@intrinsic_apis//intrinsic/icon/proto:joint_space_proto"],
)

cc_proto_library(
    name = "motion_planning_cc_proto",
    deps = [":motion_planning_proto"],
)

go_proto_library(
    name = "motion_planning_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/motion_planning_go_proto",
    proto = ":motion_planning_proto",
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_go_proto",
    ],
)

py_proto_library(
    name = "motion_planning_py_pb2",
    deps = [":motion_planning_proto"],
)

proto_library(
    name = "motion_planning_error_proto",
    srcs = ["motion_planning_error.proto"],
    deps = [
        ":geometric_constraints_proto",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_proto",
    ],
)

cc_proto_library(
    name = "motion_planning_error_cc_proto",
    deps = [":motion_planning_error_proto"],
)

go_proto_library(
    name = "motion_planning_error_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/motion_planning_error_go_proto",
    proto = ":motion_planning_error_proto",
    deps = [
        ":geometric_constraints_go_proto",
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_go_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_go_proto",
        "@intrinsic_apis//intrinsic/world/proto:collision_settings_go_proto",
    ],
)

py_proto_library(
    name = "motion_planning_error_py_pb2",
    deps = [":motion_planning_error_proto"],
)

proto_library(
    name = "motion_planning_limits_proto",
    srcs = ["motion_planning_limits.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_proto",
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_proto",
    ],
)

cc_proto_library(
    name = "motion_planning_limits_cc_proto",
    deps = [":motion_planning_limits_proto"],
)

go_proto_library(
    name = "motion_planning_limits_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/motion_planning_limits_go_proto",
    proto = ":motion_planning_limits_proto",
    deps = [
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_go_proto",
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_go_proto",
    ],
)

py_proto_library(
    name = "motion_planning_limits_py_pb2",
    deps = [":motion_planning_limits_proto"],
)

proto_library(
    name = "robot_specification_proto",
    srcs = ["robot_specification.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_proto",
    ],
)

cc_proto_library(
    name = "robot_specification_cc_proto",
    deps = [":robot_specification_proto"],
)

go_proto_library(
    name = "robot_specification_go_proto",
    importpath = "intrinsic/motion_planning/proto/v1/robot_specification_go_proto",
    proto = ":robot_specification_proto",
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_go_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_go_proto",
    ],
)

py_proto_library(
    name = "robot_specification_py_pb2",
    deps = [":robot_specification_proto"],
)
