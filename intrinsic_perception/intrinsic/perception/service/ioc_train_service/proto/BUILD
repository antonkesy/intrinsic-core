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

load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "ioc_pose_estimator_params_proto",
    srcs = ["ioc_pose_estimator_params.proto"],
)

py_proto_library(
    name = "ioc_pose_estimator_params_py_pb2",
    deps = [":ioc_pose_estimator_params_proto"],
)

go_proto_library(
    name = "ioc_pose_estimator_params_go_proto",
    importpath = "intrinsic_perception/intrinsic/perception/service/ioc_train_service/proto/ioc_pose_estimator_params_go_proto",
    proto = ":ioc_pose_estimator_params_proto",
)
