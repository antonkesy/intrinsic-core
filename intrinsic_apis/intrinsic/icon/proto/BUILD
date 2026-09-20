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

load("@com_github_grpc_grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "cart_space_proto",
    srcs = ["cart_space.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "cart_space_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":cart_space_proto"],
)

go_proto_library(
    name = "cart_space_go_proto",
    importpath = "intrinsic/icon/proto/cart_space_go_proto",
    protos = [":cart_space_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "cart_space_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":cart_space_proto"],
)

proto_library(
    name = "generic_part_config_proto",
    srcs = ["generic_part_config.proto"],
    deps = [
        ":cart_space_proto",
        "@intrinsic_apis//intrinsic/kinematics/proto:skeleton_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
    ],
)

cc_proto_library(
    name = "generic_part_config_cc_proto",
    deps = [":generic_part_config_proto"],
)

go_proto_library(
    name = "generic_part_config_go_proto",
    importpath = "intrinsic/icon/proto/generic_part_config_go_proto",
    protos = [":generic_part_config_proto"],
    deps = [
        ":cart_space_go_proto",
        "@intrinsic_apis//intrinsic/kinematics/proto:skeleton_go_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_go_proto",
    ],
)

py_proto_library(
    name = "generic_part_config_py_pb2",
    deps = [":generic_part_config_proto"],
)

proto_library(
    name = "ik_options_proto",
    srcs = ["ik_options.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "ik_options_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":ik_options_proto"],
)

py_proto_library(
    name = "ik_options_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":ik_options_proto"],
)

proto_library(
    name = "io_block_proto",
    srcs = ["io_block.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "io_block_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":io_block_proto"],
)

go_proto_library(
    name = "io_block_go_proto",
    importpath = "intrinsic/icon/proto/io_block_go_proto",
    protos = [":io_block_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "io_block_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":io_block_proto"],
)

proto_library(
    name = "joint_position_pid_torque_controller_config_proto",
    srcs = ["joint_position_pid_torque_controller_config.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "joint_position_pid_torque_controller_config_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":joint_position_pid_torque_controller_config_proto"],
)

go_proto_library(
    name = "joint_position_pid_torque_controller_config_go_proto",
    importpath = "intrinsic/icon/proto/joint_position_pid_torque_controller_config_go_proto",
    protos = [":joint_position_pid_torque_controller_config_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "joint_position_pid_velocity_controller_config_proto",
    srcs = ["joint_position_pid_velocity_controller_config.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "joint_position_pid_velocity_controller_config_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":joint_position_pid_velocity_controller_config_proto"],
)

proto_library(
    name = "joint_space_proto",
    srcs = ["joint_space.proto"],
    deps = [
        "@com_google_protobuf//:duration_proto",
        "@intrinsic_apis//intrinsic/kinematics/types:dynamic_limits_check_mode_proto",
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_proto",
    ],
)

cc_proto_library(
    name = "joint_space_cc_proto",
    deps = [":joint_space_proto"],
)

go_proto_library(
    name = "joint_space_go_proto",
    importpath = "intrinsic/icon/proto/joint_space_go_proto",
    protos = [":joint_space_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/kinematics/types:dynamic_limits_check_mode_go_proto",
        "@intrinsic_apis//intrinsic/skills/proto:skill_parameter_metadata_go_proto",
    ],
)

py_proto_library(
    name = "joint_space_py_pb2",
    deps = [":joint_space_proto"],
)

proto_library(
    name = "limit_provider_service_proto",
    srcs = ["limit_provider_service.proto"],
    deps = ["@intrinsic_apis//intrinsic/kinematics/types:joint_limits_proto"],
)

cc_proto_library(
    name = "limit_provider_service_cc_proto",
    deps = [":limit_provider_service_proto"],
)

cc_grpc_library(
    name = "limit_provider_service_cc_grpc_proto",
    srcs = [":limit_provider_service_proto"],
    generate_mocks = True,
    grpc_only = True,
    deps = [":limit_provider_service_cc_proto"],
)

go_proto_library(
    name = "limit_provider_service_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
    ],
    importpath = "intrinsic/icon/proto/limit_provider_service_go_proto",
    protos = [":limit_provider_service_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/kinematics/types:joint_limits_go_proto",
    ],
)

py_proto_library(
    name = "limit_provider_service_py_pb2",
    deps = [":limit_provider_service_proto"],
)

proto_library(
    name = "linear_joint_acceleration_filter_config_proto",
    srcs = ["linear_joint_acceleration_filter_config.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "linear_joint_acceleration_filter_config_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":linear_joint_acceleration_filter_config_proto"],
)

go_proto_library(
    name = "linear_joint_acceleration_filter_config_go_proto",
    importpath = "intrinsic/icon/proto/linear_joint_acceleration_filter_config_go_proto",
    protos = [":linear_joint_acceleration_filter_config_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "logging_mode_proto",
    srcs = ["logging_mode.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "logging_mode_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":logging_mode_proto"],
)

go_proto_library(
    name = "logging_mode_go_proto",
    importpath = "intrinsic/icon/proto/logging_mode_go_proto",
    protos = [":logging_mode_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "logging_mode_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":logging_mode_proto"],
)

proto_library(
    name = "matrix_proto",
    srcs = ["matrix.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "matrix_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":matrix_proto"],
)

go_proto_library(
    name = "matrix_go_proto",
    importpath = "intrinsic/icon/proto/matrix_go_proto",
    protos = [":matrix_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "matrix_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":matrix_proto"],
)

proto_library(
    name = "safety_status_proto",
    srcs = ["safety_status.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "safety_status_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":safety_status_proto"],
)

go_proto_library(
    name = "safety_status_go_proto",
    importpath = "intrinsic/icon/proto/safety_status_go_proto",
    protos = [":safety_status_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "safety_status_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":safety_status_proto"],
)

proto_library(
    name = "part_status_proto",
    srcs = ["part_status.proto"],
    deps = [
        ":cart_space_proto",
        ":io_block_proto",
        ":safety_status_proto",
        "@intrinsic_apis//intrinsic/icon/proto/v1:types_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_proto",
        "@intrinsic_apis//intrinsic/math/proto:vector3_proto",
    ],
)

cc_proto_library(
    name = "part_status_cc_proto",
    deps = [":part_status_proto"],
)

go_proto_library(
    name = "part_status_go_proto",
    importpath = "intrinsic/icon/proto/part_status_go_proto",
    protos = [":part_status_proto"],
    deps = [
        ":cart_space_go_proto",
        ":io_block_go_proto",
        ":safety_status_go_proto",
        "@intrinsic_apis//intrinsic/icon/proto/v1:types_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:quaternion_go_proto",
        "@intrinsic_apis//intrinsic/math/proto:vector3_go_proto",
    ],
)

py_proto_library(
    name = "part_status_py_pb2",
    deps = [":part_status_proto"],
)

proto_library(
    name = "streaming_output_proto",
    srcs = ["streaming_output.proto"],
    visibility = ["//visibility:public"],
    deps = ["@com_google_protobuf//:any_proto"],
)

cc_proto_library(
    name = "streaming_output_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":streaming_output_proto"],
)

go_proto_library(
    name = "streaming_output_go_proto",
    importpath = "intrinsic/icon/proto/streaming_output_go_proto",
    protos = [":streaming_output_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "streaming_output_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":streaming_output_proto"],
)
