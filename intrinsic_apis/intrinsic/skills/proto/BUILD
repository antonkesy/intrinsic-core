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
    name = "footprint_proto",
    srcs = ["footprint.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/geometry/proto:transformed_geometry_storage_refs_proto",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:transformed_geometry_proto",
        "@intrinsic_apis//intrinsic/world/proto:entity_search_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_proto",
    ],
)

cc_proto_library(
    name = "footprint_cc_proto",
    deps = [":footprint_proto"],
)

go_proto_library(
    name = "footprint_go_proto",
    importpath = "intrinsic/skills/proto/footprint_go_proto",
    protos = [":footprint_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/geometry/proto:transformed_geometry_storage_refs_go_proto",
        "@intrinsic_apis//intrinsic/geometry/proto/v1:transformed_geometry_go_proto",
        "@intrinsic_apis//intrinsic/world/proto:entity_search_go_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_go_proto",
    ],
)

py_proto_library(
    name = "footprint_py_pb2",
    deps = [":footprint_proto"],
)

proto_library(
    name = "equipment_proto",
    srcs = ["equipment.proto"],
    deps = [":footprint_proto"],
)

cc_proto_library(
    name = "equipment_cc_proto",
    deps = [":equipment_proto"],
)

go_proto_library(
    name = "equipment_go_proto",
    importpath = "intrinsic/skills/proto/equipment_go_proto",
    protos = [":equipment_proto"],
    deps = [":footprint_go_proto"],
)

py_proto_library(
    name = "equipment_py_pb2",
    deps = [":equipment_proto"],
)

proto_library(
    name = "error_proto",
    srcs = ["error.proto"],
)

cc_proto_library(
    name = "error_cc_proto",
    deps = [":error_proto"],
)

go_proto_library(
    name = "error_go_proto",
    importpath = "intrinsic/skills/proto/error_go_proto",
    protos = [":error_proto"],
)

py_proto_library(
    name = "error_py_pb2",
    deps = [":error_proto"],
)

proto_library(
    name = "motion_targets_proto",
    srcs = ["motion_targets.proto"],
    deps = [
        "@intrinsic_apis//intrinsic/math/proto:pose_proto",
        "@intrinsic_apis//intrinsic/world/proto:entity_search_proto",
    ],
)

cc_proto_library(
    name = "motion_targets_cc_proto",
    deps = [":motion_targets_proto"],
)

py_proto_library(
    name = "motion_targets_py_pb2",
    deps = [":motion_targets_proto"],
)

proto_library(
    name = "skill_manifest_proto",
    srcs = ["skill_manifest.proto"],
    deps = [
        ":equipment_proto",
        "@com_google_protobuf//:any_proto",
        "@com_google_protobuf//:duration_proto",
        "@intrinsic_apis//intrinsic/assets/proto:documentation_proto",
        "@intrinsic_apis//intrinsic/assets/proto:id_proto",
        "@intrinsic_apis//intrinsic/assets/proto:status_spec_proto",
        "@intrinsic_apis//intrinsic/assets/proto:vendor_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:recommended_configuration_config_proto",
    ],
)

cc_proto_library(
    name = "skill_manifest_cc_proto",
    deps = [":skill_manifest_proto"],
)

go_proto_library(
    name = "skill_manifest_go_proto",
    importpath = "intrinsic/skills/proto/skill_manifest_go_proto",
    protos = [":skill_manifest_proto"],
    deps = [
        ":equipment_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:documentation_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:id_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:status_spec_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:vendor_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:recommended_configuration_config_go_proto",
    ],
)

py_proto_library(
    name = "skill_manifest_py_pb2",
    deps = [":skill_manifest_proto"],
)

proto_library(
    name = "processed_skill_manifest_proto",
    srcs = ["processed_skill_manifest.proto"],
    deps = [
        ":skill_manifest_proto",
        "@com_google_protobuf//:descriptor_proto",
        "@intrinsic_apis//intrinsic/assets/proto:documentation_proto",
        "@intrinsic_apis//intrinsic/assets/proto:id_proto",
        "@intrinsic_apis//intrinsic/assets/proto:status_spec_proto",
        "@intrinsic_apis//intrinsic/assets/proto:vendor_proto",
        "@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:image_proto",
    ],
)

go_proto_library(
    name = "processed_skill_manifest_go_proto",
    importpath = "intrinsic/skills/proto/processed_skill_manifest_go_proto",
    protos = [":processed_skill_manifest_proto"],
    deps = [
        ":skill_manifest_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:documentation_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:id_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:status_spec_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:vendor_go_proto",
        "@intrinsic_apis//intrinsic/kubernetes/workcell_spec/proto:image_go_proto",
    ],
)

py_proto_library(
    name = "processed_skill_manifest_py_pb2",
    deps = [":processed_skill_manifest_proto"],
)

proto_library(
    name = "skill_parameter_metadata_proto",
    srcs = ["skill_parameter_metadata.proto"],
    deps = ["@com_google_protobuf//:descriptor_proto"],
)

cc_proto_library(
    name = "skill_parameter_metadata_cc_proto",
    deps = [":skill_parameter_metadata_proto"],
)

go_proto_library(
    name = "skill_parameter_metadata_go_proto",
    importpath = "intrinsic/skills/proto/skill_parameter_metadata_go_proto",
    protos = [":skill_parameter_metadata_proto"],
)

py_proto_library(
    name = "skill_parameter_metadata_py_pb2",
    deps = [":skill_parameter_metadata_proto"],
)

proto_library(
    name = "skills_proto",
    srcs = ["skills.proto"],
    deps = [
        ":equipment_proto",
        "@com_google_protobuf//:any_proto",
        "@com_google_protobuf//:descriptor_proto",
        "@intrinsic_apis//intrinsic/resources/proto:resource_handle_proto",
    ],
)

cc_proto_library(
    name = "skills_cc_proto",
    deps = [":skills_proto"],
)

go_proto_library(
    name = "skills_go_proto",
    importpath = "intrinsic/skills/proto/skills_go_proto",
    protos = [":skills_proto"],
    deps = [
        ":equipment_go_proto",
        "@intrinsic_apis//intrinsic/resources/proto:resource_handle_go_proto",
    ],
)

py_proto_library(
    name = "skills_py_pb2",
    deps = [":skills_proto"],
)

proto_library(
    name = "skill_registry_proto",
    srcs = ["skill_registry.proto"],
    deps = [
        ":skills_proto",
        "@com_google_protobuf//:empty_proto",
        "@intrinsic_apis//intrinsic/kubernetes/generator/proto:auth_options_proto",
    ],
)

cc_proto_library(
    name = "skill_registry_cc_proto",
    deps = [":skill_registry_proto"],
)

cc_grpc_library(
    name = "skill_registry_cc_grpc_proto",
    srcs = [":skill_registry_proto"],
    generate_mocks = True,
    grpc_only = True,
    deps = [":skill_registry_cc_proto"],
)

go_proto_library(
    name = "skill_registry_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
    ],
    importpath = "intrinsic/skills/proto/skill_registry_go_proto",
    protos = [":skill_registry_proto"],
    deps = [
        ":skills_go_proto",
        "@intrinsic_apis//intrinsic/kubernetes/generator/proto:auth_options_go_proto",
    ],
)

py_proto_library(
    name = "skill_registry_py_pb2",
    deps = [":skill_registry_proto"],
)

py_grpc_library(
    name = "skill_registry_py_pb2_grpc",
    srcs = [":skill_registry_proto"],
    deps = [":skill_registry_py_pb2"],
)

proto_library(
    name = "skill_service_config_proto",
    srcs = ["skill_service_config.proto"],
    deps = [
        ":skills_proto",
        "@com_google_protobuf//:duration_proto",
        "@intrinsic_apis//intrinsic/assets/proto:status_spec_proto",
    ],
)

cc_proto_library(
    name = "skill_service_config_cc_proto",
    deps = [":skill_service_config_proto"],
)

go_proto_library(
    name = "skill_service_config_go_proto",
    importpath = "intrinsic/skills/proto/skill_service_config_go_proto",
    protos = [":skill_service_config_proto"],
    deps = [
        ":skills_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:status_spec_go_proto",
    ],
)

py_proto_library(
    name = "skill_service_config_py_pb2",
    deps = [":skill_service_config_proto"],
)
