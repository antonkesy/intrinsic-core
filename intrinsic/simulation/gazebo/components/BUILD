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

load("//bazel:cc_macros.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_component_registers",
    deps = [
        ":articulation_component_register",
        ":interaction_component_register",
        ":joint_commanded_position_register",
        ":outfeed_component_register",
        ":spawner_component_register",
    ],
)

cc_library(
    name = "spawner_component",
    hdrs = ["spawner_component.h"],
    deps = [
        "//intrinsic_sdk/intrinsic/math:pose3",
        "//intrinsic_sdk/intrinsic/math:proto_conversion",
        "@abseil-cpp//absl/status",
        "@gz-sim",
        "@intrinsic_apis//intrinsic/scene/proto/v1:scene_object_cc_proto",
        "@intrinsic_apis//intrinsic/world/proto:object_world_refs_cc_proto",
    ],
)

cc_library(
    name = "spawner_component_register",
    srcs = ["spawner_component_register.cc"],
    deps = [
        ":spawner_component",
        "@gz-sim",
    ],
    alwayslink = 1,
)

cc_library(
    name = "outfeed_component",
    hdrs = ["outfeed_component.h"],
    deps = [
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/status",
        "@gz-math",
        "@gz-sim",
    ],
)

cc_library(
    name = "outfeed_component_register",
    srcs = ["outfeed_component_register.cc"],
    deps = [
        ":outfeed_component",
        "@gz-sim",
    ],
    alwayslink = 1,
)

cc_library(
    name = "world_component",
    hdrs = ["world_component.h"],
    deps = [
        "//intrinsic/world",
        "@gz-sim",
    ],
)

cc_library(
    name = "world_component_register",
    srcs = ["world_component_register.cc"],
    deps = [
        ":world_component",
        "@gz-sim",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ppr_component",
    hdrs = ["ppr_component.h"],
    deps = ["@gz-sim"],
)

cc_library(
    name = "ppr_component_register",
    srcs = ["ppr_component_register.cc"],
    deps = [
        ":ppr_component",
        "@gz-sim",
    ],
    alwayslink = 1,
)

cc_library(
    name = "interaction_component",
    hdrs = ["interaction_component.h"],
    deps = ["@gz-sim"],
)

cc_library(
    name = "interaction_component_register",
    srcs = ["interaction_component_register.cc"],
    deps = [
        ":interaction_component",
        "@gz-sim",
    ],
    alwayslink = 1,
)

cc_library(
    name = "digital_io_components",
    hdrs = ["digital_io_components.h"],
    deps = ["@gz-sim"],
)

cc_library(
    name = "digital_io_components_register",
    srcs = ["digital_io_components_register.cc"],
    deps = [
        ":digital_io_components",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_library(
    name = "joint_commanded_position",
    hdrs = ["joint_commanded_position.h"],
    deps = ["@gz-sim"],
)

cc_library(
    name = "joint_commanded_position_register",
    srcs = ["joint_commanded_position_register.cc"],
    deps = [
        ":joint_commanded_position",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_library(
    name = "ft_sensor_tare_components",
    hdrs = ["ft_sensor_tare_components.h"],
    deps = [
        "//intrinsic_control/intrinsic/icon/utils:sensor_utils",
        "@gz-sim",
    ],
)

cc_library(
    name = "ft_sensor_tare_components_register",
    srcs = ["ft_sensor_tare_components_register.cc"],
    deps = [
        ":ft_sensor_tare_components",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_library(
    name = "kinematic_chain_payload_component",
    hdrs = ["kinematic_chain_payload_component.h"],
    deps = [
        "//intrinsic_sdk/intrinsic/world/robot_payload:robot_payload_base",
        "@gz-sim",
    ],
)

cc_library(
    name = "kinematic_chain_payload_component_register",
    srcs = ["kinematic_chain_payload_component_register.cc"],
    deps = [
        ":kinematic_chain_payload_component",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_library(
    name = "service_state_component",
    hdrs = ["service_state_component.h"],
    deps = [
        "@abseil-cpp//absl/status:statusor",
        "@gz-sim",
        "@intrinsic_apis//intrinsic/assets/services/proto/v1:service_state_cc_proto",
    ],
)

cc_library(
    name = "service_state_component_register",
    srcs = ["service_state_component_register.cc"],
    deps = [
        ":service_state_component",
        "@gz-sim",
    ],
    alwayslink = True,
)

cc_library(
    name = "articulation_component",
    hdrs = ["articulation_component.h"],
    deps = ["@gz-sim"],
)

cc_library(
    name = "articulation_component_register",
    srcs = ["articulation_component_register.cc"],
    deps = [
        ":articulation_component",
        "@gz-sim",
    ],
    alwayslink = 1,
)
