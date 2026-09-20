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
    name = "adio_proto",
    srcs = ["adio.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "adio_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":adio_proto"],
)

go_proto_library(
    name = "adio_go_proto",
    importpath = "intrinsic/icon/actions/adio_go_proto",
    protos = [":adio_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "adio_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":adio_proto"],
)

proto_library(
    name = "cartesian_jogging_proto",
    srcs = ["cartesian_jogging.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:cart_space_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_proto",
    ],
)

cc_proto_library(
    name = "cartesian_jogging_cc_proto",
    deps = [":cartesian_jogging_proto"],
)

go_proto_library(
    name = "cartesian_jogging_go_proto",
    importpath = "intrinsic/icon/actions/cartesian_jogging_go_proto",
    protos = [":cartesian_jogging_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:cart_space_go_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_go_proto",
    ],
)

py_proto_library(
    name = "cartesian_jogging_py_pb2",
    deps = [":cartesian_jogging_proto"],
)

proto_library(
    name = "point_to_point_move_proto",
    srcs = ["point_to_point_move.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_proto",
    ],
)

cc_proto_library(
    name = "point_to_point_move_cc_proto",
    deps = [":point_to_point_move_proto"],
)

go_proto_library(
    name = "point_to_point_move_go_proto",
    importpath = "intrinsic/icon/actions/point_to_point_move_go_proto",
    protos = [":point_to_point_move_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:joint_space_go_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_go_proto",
    ],
)

py_proto_library(
    name = "point_to_point_move_py_pb2",
    deps = [":point_to_point_move_proto"],
)

proto_library(
    name = "simple_gripper_proto",
    srcs = ["simple_gripper.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "simple_gripper_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":simple_gripper_proto"],
)

py_proto_library(
    name = "simple_gripper_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":simple_gripper_proto"],
)

proto_library(
    name = "tare_force_torque_sensor_proto",
    srcs = ["tare_force_torque_sensor.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "tare_force_torque_sensor_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":tare_force_torque_sensor_proto"],
)

py_proto_library(
    name = "tare_force_torque_sensor_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":tare_force_torque_sensor_proto"],
)

proto_library(
    name = "trajectory_tracking_action_proto",
    srcs = ["trajectory_tracking_action.proto"],
    deps = ["@intrinsic_apis//intrinsic/icon/proto:joint_space_proto"],
)

cc_proto_library(
    name = "trajectory_tracking_action_cc_proto",
    deps = [":trajectory_tracking_action_proto"],
)

py_proto_library(
    name = "trajectory_tracking_action_py_pb2",
    deps = [":trajectory_tracking_action_proto"],
)

proto_library(
    name = "wait_for_settling_action_proto",
    srcs = ["wait_for_settling_action.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "wait_for_settling_action_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":wait_for_settling_action_proto"],
)

py_proto_library(
    name = "wait_for_settling_action_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":wait_for_settling_action_proto"],
)
