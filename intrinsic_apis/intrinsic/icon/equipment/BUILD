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
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_google_protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@io_bazel_rules_go//proto:def.bzl", "go_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "force_control_settings_proto",
    srcs = ["force_control_settings.proto"],
    deps = ["@intrinsic_apis//intrinsic/icon/proto:cart_space_proto"],
)

cc_proto_library(
    name = "force_control_settings_cc_proto",
    deps = [":force_control_settings_proto"],
)

go_proto_library(
    name = "force_control_settings_go_proto",
    importpath = "intrinsic/icon/equipment/force_control_settings_go_proto",
    protos = [":force_control_settings_proto"],
    deps = [
        "@intrinsic_apis//intrinsic/icon/proto:cart_space_go_proto",
    ],
)

py_proto_library(
    name = "force_control_settings_py_pb2",
    deps = [":force_control_settings_proto"],
)

proto_library(
    name = "icon_equipment_proto",
    srcs = ["icon_equipment.proto"],
    deps = [
        ":force_control_settings_proto",
        "@intrinsic_apis//intrinsic/hardware/gpio:gpio_service_equipment_proto",
    ],
)

cc_proto_library(
    name = "icon_equipment_cc_proto",
    deps = [":icon_equipment_proto"],
)

go_proto_library(
    name = "icon_equipment_go_proto",
    importpath = "intrinsic/icon/equipment/icon_equipment_go_proto",
    protos = [":icon_equipment_proto"],
    deps = [
        ":force_control_settings_go_proto",
        "@intrinsic_apis//intrinsic/hardware/gpio:gpio_service_equipment_go_proto",
    ],
)

py_proto_library(
    name = "icon_equipment_py_pb2",
    deps = [":icon_equipment_proto"],
)
