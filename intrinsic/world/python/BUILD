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

load("//bazel:cc_macros.bzl", "pybind_extension")
load("//bazel:python.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

pybind_extension(
    name = "geometry_types",
    srcs = ["geometry_types.cc"],
    deps = [
        "//intrinsic_sdk/intrinsic/world:geometry_types",
    ],
)

py_library(
    name = "geometry_component_utils",
    srcs = ["geometry_component_utils.py"],
    deps = [
        ":geometry_types",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:exact_geometry_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:geometry_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:geometry_storage_refs_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:inline_geometry_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:primitive_shape_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:primitives_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:transformed_geometry_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:transformed_primitive_shape_py_pb2",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:transformed_primitive_shape_set_py_pb2",
        "@intrinsic_apis//intrinsic/world/proto:geometry_component_py_pb2",
    ],
)
