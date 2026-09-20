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
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("//bazel:go_macros.bzl", "go_proto_library")
load("//bazel:protos.bzl", "proto_library")

package(
    default_visibility = [
        "//visibility:public",
    ],
)

proto_library(
    name = "spacemouse_proto",
    srcs = ["spacemouse.proto"],
    visibility = ["//visibility:public"],
    deps = ["@intrinsic_apis//intrinsic/math/proto:twist_proto"],
)

py_proto_library(
    name = "spacemouse_py_pb2",
    deps = [":spacemouse_proto"],
)

go_proto_library(
    name = "spacemouse_go_proto",
    importpath = "intrinsic/ml/rl/proto/v1/spacemouse_go_proto",
    proto = ":spacemouse_proto",
    visibility = ["//visibility:public"],
    deps = ["@intrinsic_apis//intrinsic/math/proto:twist_go_proto"],
)

cc_proto_library(
    name = "spacemouse_cc_proto",
    deps = [":spacemouse_proto"],
)
