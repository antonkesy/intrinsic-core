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

# gazelle:proto disable
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

exports_files([
    "default_inference_service_config_values.textproto",
])

proto_library(
    name = "inference_service_config_proto",
    srcs = ["inference_service_config.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/assets/proto:field_metadata_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:resolved_dependency_proto",
    ],
)

proto_library(
    name = "ml_model_asset_proto",
    srcs = ["ml_model_asset.proto"],
    deps = [
        "//intrinsic_inference/core/v1:ml_model_proto",
        "@intrinsic_apis//intrinsic/assets/data/proto/v1:referenced_data_proto",
    ],
)

cc_proto_library(
    name = "inference_service_config_cc_proto",
    deps = [":inference_service_config_proto"],
)

cc_proto_library(
    name = "ml_model_asset_cc_proto",
    deps = [":ml_model_asset_proto"],
)

py_proto_library(
    name = "inference_service_config_py_pb2",
    deps = [":inference_service_config_proto"],
)

py_proto_library(
    name = "ml_model_asset_py_pb2",
    deps = [":ml_model_asset_proto"],
)
