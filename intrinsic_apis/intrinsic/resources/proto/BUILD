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
    name = "resource_handle_proto",
    srcs = ["resource_handle.proto"],
    deps = ["@com_google_protobuf//:any_proto"],
)

cc_proto_library(
    name = "resource_handle_cc_proto",
    deps = [":resource_handle_proto"],
)

go_proto_library(
    name = "resource_handle_go_proto",
    importpath = "intrinsic/resources/proto/resource_handle_go_proto",
    protos = [":resource_handle_proto"],
)

py_proto_library(
    name = "resource_handle_py_pb2",
    deps = [":resource_handle_proto"],
)

proto_library(
    name = "resource_permissions_proto",
    srcs = ["resource_permissions.proto"],
)

go_proto_library(
    name = "resource_permissions_go_proto",
    importpath = "intrinsic/resources/proto/resource_permissions_go_proto",
    protos = [":resource_permissions_proto"],
)

proto_library(
    name = "resource_registry_proto",
    srcs = ["resource_registry.proto"],
    deps = [
        ":resource_handle_proto",
        "@com_google_protobuf//:any_proto",
        "@intrinsic_apis//intrinsic/config/proto:data_files_proto",
        "@intrinsic_apis//intrinsic/kubernetes/generator/proto:auth_options_proto",
        "@intrinsic_apis//intrinsic/scene/proto/v1:scene_object_config_proto",
    ],
)

cc_proto_library(
    name = "resource_registry_cc_proto",
    deps = [":resource_registry_proto"],
)

cc_grpc_library(
    name = "resource_registry_cc_grpc",
    srcs = [":resource_registry_proto"],
    generate_mocks = True,
    grpc_only = True,
    deps = [":resource_registry_cc_proto"],
)

go_proto_library(
    name = "resource_registry_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
    ],
    importpath = "intrinsic/resources/proto/resource_registry_go_proto",
    protos = [":resource_registry_proto"],
    deps = [
        ":resource_handle_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:metadata_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:view_go_proto",
        "@intrinsic_apis//intrinsic/config/proto:data_files_go_proto",
        "@intrinsic_apis//intrinsic/kubernetes/generator/proto:auth_options_go_proto",
        "@intrinsic_apis//intrinsic/scene/proto/v1:scene_object_config_go_proto",
        "@org_golang_google_protobuf//types/known/anypb",
    ],
)

py_proto_library(
    name = "resource_registry_py_pb2",
    deps = [":resource_registry_proto"],
)

py_grpc_library(
    name = "resource_registry_py_pb2_grpc",
    srcs = [":resource_registry_proto"],
    deps = [":resource_registry_py_pb2"],
)

proto_library(
    name = "resource_volume_proto",
    srcs = ["resource_volume.proto"],
)

go_proto_library(
    name = "resource_volume_go_proto",
    importpath = "intrinsic/resources/proto/resource_volume_go_proto",
    protos = [":resource_volume_proto"],
)

proto_library(
    name = "resource_service_definition_proto",
    srcs = ["resource_service_definition.proto"],
    deps = [
        ":resource_permissions_proto",
        ":resource_volume_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:recommended_configuration_config_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto:service_http_config_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto:service_permissions_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:dynamic_reconfiguration_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_inspection_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_state_proto",
        "@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:image_proto",
    ],
)

go_proto_library(
    name = "resource_service_definition_go_proto",
    importpath = "intrinsic/resources/proto/resource_service_definition_go_proto",
    protos = [":resource_service_definition_proto"],
    deps = [
        ":resource_permissions_go_proto",
        ":resource_volume_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:recommended_configuration_config_go_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto:service_http_config_go_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto:service_permissions_go_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:dynamic_reconfiguration_go_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_inspection_go_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_state_go_proto",
        "@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:image_go_proto",
    ],
)

proto_library(
    name = "runtime_context_proto",
    srcs = ["runtime_context.proto"],
    deps = [
        "@com_google_protobuf//:any_proto",
        "@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:transfer_proto",
    ],
)

cc_proto_library(
    name = "runtime_context_cc_proto",
    deps = [":runtime_context_proto"],
)

go_proto_library(
    name = "runtime_context_go_proto",
    importpath = "intrinsic/resources/proto/runtime_context_go_proto",
    protos = [":runtime_context_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:transfer_go_proto",
    ],
)

py_proto_library(
    name = "runtime_context_py_pb2",
    deps = [":runtime_context_proto"],
)
