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

load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load(":intrinsic_mlmodel.bzl", "intrinsic_mlmodel")

package(
    default_visibility = ["//intrinsic_inference/assets/inference_service:__subpackages__"],
)

py_library(
    name = "backend_parsers",
    srcs = ["backend_parsers.py"],
    deps = [
        "//intrinsic_inference/core/v1:ml_model_py_pb2",
        "@com_google_protobuf//:protobuf_python",
        "@triton-inference-server-common//protobuf:common_py_pb2",
    ],
)

py_binary(
    name = "mlmodel_manifest_generator",
    srcs = ["mlmodel_manifest_generator.py"],
    visibility = ["//visibility:public"],
    deps = [
        ":backend_parsers",
        "//intrinsic_inference/assets/inference_service/v1:ml_model_asset_py_pb2",
        "//intrinsic_inference/core/v1:ml_model_py_pb2",
        "@com_google_absl_py//absl:app",
        "@com_google_absl_py//absl/flags",
        "@com_google_protobuf//:protobuf_python",
        "@intrinsic_apis//intrinsic/assets/data/proto/v1:data_manifest_py_pb2",
        "@intrinsic_apis//intrinsic/assets/data/proto/v1:referenced_data_py_pb2",
    ],
)

intrinsic_mlmodel(
    name = "minimal_model",
    config = "minimal_triton_config.pbtxt",
    description = "A minimal test model for intrinsic_mlmodel rule.",
    display_name = "Minimal Test Model",
    id = "ai.intrinsic.minimal_model",
    model_files = {
        "minimal_triton_model.py": "1/model.py",
    },
)
