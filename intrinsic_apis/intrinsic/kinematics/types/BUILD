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
    name = "dynamic_limits_check_mode_proto",
    srcs = ["dynamic_limits_check_mode.proto"],
)

proto_library(
    name = "joint_limits_proto",
    srcs = ["joint_limits.proto"],
)

proto_library(
    name = "rotational_limits_proto",
    srcs = ["rotational_limits.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/math/proto:quaternion_proto",
        "@intrinsic_apis//intrinsic/math/proto:vector3_proto",
    ],
)

cc_proto_library(
    name = "dynamic_limits_check_mode_cc_proto",
    deps = [":dynamic_limits_check_mode_proto"],
)

go_proto_library(
    name = "dynamic_limits_check_mode_go_proto",
    importpath = "intrinsic/kinematics/types/dynamic_limits_check_mode_go_proto",
    protos = [":dynamic_limits_check_mode_proto"],
)

py_proto_library(
    name = "dynamic_limits_check_mode_py_pb2",
    deps = [":dynamic_limits_check_mode_proto"],
)

cc_proto_library(
    name = "joint_limits_cc_proto",
    deps = [":joint_limits_proto"],
)

go_proto_library(
    name = "joint_limits_go_proto",
    importpath = "intrinsic/kinematics/types/joint_limits_go_proto",
    protos = [":joint_limits_proto"],
)

py_proto_library(
    name = "joint_limits_py_pb2",
    deps = [":joint_limits_proto"],
)

cc_proto_library(
    name = "rotational_limits_cc_proto",
    deps = [":rotational_limits_proto"],
)

py_proto_library(
    name = "rotational_limits_py_pb2",
    deps = [":rotational_limits_proto"],
)
