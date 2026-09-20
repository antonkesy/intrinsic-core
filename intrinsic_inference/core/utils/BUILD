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

load("@rules_python//python:defs.bzl", "py_library")

py_library(
    name = "oip_mappings",
    srcs = ["oip_mappings.py"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "@ai_intrinsic_sdks_pip_deps//numpy",
    ],
)

py_library(
    name = "oip_utils",
    srcs = ["oip_utils.py"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        ":oip_mappings",
        "@ai_intrinsic_sdks_pip_deps//numpy",
        "@open_inference_protocol//:open_inference_grpc_py_pb2",
    ],
)
