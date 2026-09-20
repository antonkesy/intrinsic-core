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
    name = "clusterdiscovery_api_proto",
    srcs = ["clusterdiscovery_api.proto"],
    deps = [
        "//intrinsic/kubernetes/generator/proto:auth_options_proto",
    ],
)

go_proto_library(
    name = "clusterdiscovery_api_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
    ],
    importpath = "intrinsic/frontend/cloud/api/v1/clusterdiscovery_api_go_proto",
    protos = [":clusterdiscovery_api_proto"],
    deps = ["//intrinsic/kubernetes/generator/proto:auth_options_go_proto"],
)

proto_library(
    name = "solutiondiscovery_api_proto",
    srcs = ["solutiondiscovery_api.proto"],
    deps = [
        ":clusterdiscovery_api_proto",
        "//intrinsic/kubernetes/generator/proto:auth_options_proto",
    ],
)

cc_proto_library(
    name = "solutiondiscovery_api_cc_proto",
    deps = [":solutiondiscovery_api_proto"],
)

cc_grpc_library(
    name = "solutiondiscovery_api_cc_grpc_proto",
    srcs = [":solutiondiscovery_api_proto"],
    grpc_only = True,
    deps = [":solutiondiscovery_api_cc_proto"],
)

py_proto_library(
    name = "solutiondiscovery_api_py_pb2",
    deps = [":solutiondiscovery_api_proto"],
)

py_grpc_library(
    name = "solutiondiscovery_api_py_pb2_grpc",
    srcs = [":solutiondiscovery_api_proto"],
    deps = [":solutiondiscovery_api_py_pb2"],
)

go_proto_library(
    name = "solutiondiscovery_api_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
    ],
    importpath = "intrinsic/frontend/cloud/api/v1/solutiondiscovery_api_go_proto",
    protos = [":solutiondiscovery_api_proto"],
    deps = [
        ":clusterdiscovery_api_go_proto",
        "//intrinsic/kubernetes/generator/proto:auth_options_go_proto",
    ],
)
