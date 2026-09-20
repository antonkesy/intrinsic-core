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
    name = "config_proto",
    srcs = ["config.proto"],
)

cc_proto_library(
    name = "config_cc_proto",
    deps = [":config_proto"],
)

go_proto_library(
    name = "config_go_proto",
    importpath = "intrinsic/config/proto/config_go_proto",
    protos = [":config_proto"],
)

py_proto_library(
    name = "config_py_pb2",
    deps = [":config_proto"],
)

proto_library(
    name = "data_files_proto",
    srcs = ["data_files.proto"],
    deps = ["@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:transfer_proto"],
)

cc_proto_library(
    name = "data_files_cc_proto",
    deps = [":data_files_proto"],
)

go_proto_library(
    name = "data_files_go_proto",
    importpath = "intrinsic/config/proto/data_files_go_proto",
    protos = [":data_files_proto"],
    deps = ["@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:transfer_go_proto"],
)

py_proto_library(
    name = "data_files_py_pb2",
    deps = [":data_files_proto"],
)

proto_library(
    name = "operation_mode_proto",
    srcs = ["operation_mode.proto"],
)

cc_proto_library(
    name = "operation_mode_cc_proto",
    deps = [":operation_mode_proto"],
)

go_proto_library(
    name = "operation_mode_go_proto",
    importpath = "intrinsic/config/proto/operation_mode_go_proto",
    protos = [":operation_mode_proto"],
)

py_proto_library(
    name = "operation_mode_py_pb2",
    deps = [":operation_mode_proto"],
)
