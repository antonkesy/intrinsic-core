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
load("//bazel:protos.bzl", "proto_library")

package(
    default_visibility = [
        "//intrinsic/manipulation/skills/force:__subpackages__",
        "//intrinsic/skills/apps/icon2:__subpackages__",
        "//intrinsic_control/intrinsic/icon:__subpackages__",
    ],
)

proto_library(
    name = "controller_params_proto",
    srcs = ["controller_params.proto"],
)

py_proto_library(
    name = "controller_params_py_pb2",
    deps = [":controller_params_proto"],
)

cc_proto_library(
    name = "controller_params_cc_proto",
    deps = [":controller_params_proto"],
)

proto_library(
    name = "force_primitives_proto",
    srcs = ["force_primitives.proto"],
    deps = [
        ":controller_params_proto",
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
        "@intrinsic_apis//intrinsic/math/proto:vector3_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_proto",
    ],
)

py_proto_library(
    name = "force_primitives_py_pb2",
    deps = [":force_primitives_proto"],
)

cc_proto_library(
    name = "force_primitives_cc_proto",
    deps = [":force_primitives_proto"],
)
