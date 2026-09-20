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
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_grpc_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "gripper_config_proto",
    srcs = ["gripper_config.proto"],
    deps = ["//intrinsic/hardware/gpio/v1:signal_proto"],
)

cc_proto_library(
    name = "gripper_config_cc_proto",
    deps = [":gripper_config_proto"],
)

go_proto_library(
    name = "gripper_config_go_proto",
    importpath = "intrinsic/hardware/gripper/eoat/gripper_config_go_proto",
    protos = [":gripper_config_proto"],
    deps = [
        "//intrinsic/hardware/gpio/v1:signal_go_proto",
    ],
)

proto_library(
    name = "eoat_service_proto",
    srcs = ["eoat_service.proto"],
    deps = [
        ":gripper_config_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

cc_proto_library(
    name = "eoat_service_cc_proto",
    deps = [":eoat_service_proto"],
)

cc_grpc_library(
    name = "eoat_service_cc_grpc_proto",
    srcs = [":eoat_service_proto"],
    generate_mocks = True,
    grpc_only = True,
    deps = [":eoat_service_cc_proto"],
)

go_proto_library(
    name = "eoat_service_go_proto",
    importpath = "intrinsic/hardware/gripper/eoat/eoat_service_go_proto",
    protos = [":eoat_service_proto"],
    deps = [
        ":gripper_config_go_proto",
        "//intrinsic/hardware/gpio/v1:gpio_service_go_proto",
    ],
)

py_proto_library(
    name = "eoat_service_py_pb2",
    deps = [":eoat_service_proto"],
)

py_grpc_library(
    name = "eoat_service_py_pb2_grpc",
    srcs = [":eoat_service_proto"],
    deps = [":eoat_service_py_pb2"],
)
