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

load("//bazel:go_macros.bzl", "go_library")

go_library(
    name = "graph",
    srcs = ["graph.go"],
    importpath = "intrinsic/assets/hardware_devices/graph",
    visibility = ["//intrinsic:intrinsic_internal"],
    deps = [
        "//intrinsic/assets/catalog/proto/v1:asset_catalog_internal_go_proto",
        "//intrinsic/kubernetes/acl:clientcontext",
        "//intrinsic/util/proto:names",
        "@intrinsic_apis//intrinsic/assets/data/proto/v1:data_asset_go_proto",
        "@intrinsic_apis//intrinsic/assets/hardware_devices/proto/v1:hardware_device_manifest_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:asset_type_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:id_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:view_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:asset_graph_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:reference_go_proto",
        "@intrinsic_apis//intrinsic/assets/scene_objects/proto:scene_object_manifest_go_proto",
        "@intrinsic_apis//intrinsic/assets/services/proto:service_manifest_go_proto",
        "@org_golang_google_grpc//:go_default_library",
        "@org_golang_google_protobuf//proto",
    ],
)

go_library(
    name = "hardwaredevicebundle",
    srcs = ["hardwaredevicebundle.go"],
    importpath = "intrinsic/assets/hardware_devices/hardwaredevicebundle",
    visibility = ["//intrinsic:internal_api_users"],
    deps = [
        ":hardwaredevicevalidate",
        "//intrinsic/assets:idutils",
        "//intrinsic/assets:imageutils",
        "//intrinsic/assets:ioutils",
        "//intrinsic/assets:referenceddata",
        "//intrinsic/assets/data:databundle",
        "//intrinsic/assets/scene_objects:gzfprocessor",
        "//intrinsic/assets/scene_objects:sceneobjectbundle",
        "//intrinsic/assets/services:servicebundle",
        "//intrinsic/util/archive:tartooling",
        "@com_github_google_safearchive//tar",
        "@intrinsic_apis//intrinsic/assets/hardware_devices/proto/v1:hardware_device_manifest_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:asset_type_go_proto",
    ],
)

go_library(
    name = "hardwaredevicefix",
    srcs = ["hardwaredevicefix.go"],
    importpath = "intrinsic/assets/hardware_devices/hardwaredevicefix",
    visibility = ["//intrinsic:internal_api_users"],
    deps = [
        "//intrinsic/assets/data:datafix",
        "//intrinsic/assets/scene_objects:sceneobjectfix",
        "//intrinsic/assets/services:servicefix",
        "@intrinsic_apis//intrinsic/assets/hardware_devices/proto/v1:hardware_device_manifest_go_proto",
    ],
)

go_library(
    name = "hardwaredevicevalidate",
    srcs = ["hardwaredevicevalidate.go"],
    importpath = "intrinsic/assets/hardware_devices/hardwaredevicevalidate",
    visibility = ["//intrinsic:public_api_users"],
    deps = [
        "//intrinsic/assets:idutils",
        "//intrinsic/assets:metadatautils",
        "//intrinsic/assets/data:datavalidate",
        "//intrinsic/assets/errors:report",
        "//intrinsic/assets/scene_objects:sceneobjectvalidate",
        "//intrinsic/assets/services:servicevalidate",
        "@intrinsic_apis//intrinsic/assets/hardware_devices/proto/v1:hardware_device_manifest_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:asset_type_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto:id_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:asset_graph_go_proto",
        "@intrinsic_apis//intrinsic/assets/proto/v1:reference_go_proto",
    ],
)
