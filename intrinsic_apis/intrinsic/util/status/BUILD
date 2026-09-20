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
    name = "extended_status_proto",
    srcs = ["extended_status.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_protobuf//:timestamp_proto",
        "@intrinsic_apis//intrinsic/logging/proto:context_proto",
    ],
)

cc_proto_library(
    name = "extended_status_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":extended_status_proto"],
)

go_proto_library(
    name = "extended_status_go_proto",
    importpath = "intrinsic/util/status/extended_status_go_proto",
    protos = [":extended_status_proto"],
    visibility = ["//visibility:public"],
    deps = [
        "@intrinsic_apis//intrinsic/logging/proto:context_go_proto",
    ],
)

py_proto_library(
    name = "extended_status_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":extended_status_proto"],
)

proto_library(
    name = "status_proto",
    srcs = ["status.proto"],
)

cc_proto_library(
    name = "status_cc_proto",
    deps = [":status_proto"],
)

go_proto_library(
    name = "status_go_proto",
    importpath = "intrinsic/util/status/status_go_proto",
    protos = [":status_proto"],
)
