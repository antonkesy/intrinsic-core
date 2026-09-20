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
    name = "bag_metadata_proto",
    srcs = ["bag_metadata.proto"],
    deps = ["@com_google_protobuf//:timestamp_proto"],
)

cc_proto_library(
    name = "bag_metadata_cc_proto",
    deps = [":bag_metadata_proto"],
)

go_proto_library(
    name = "bag_metadata_go_proto",
    importpath = "intrinsic/logging/proto/bag_metadata_go_proto",
    protos = [":bag_metadata_proto"],
)

py_proto_library(
    name = "bag_metadata_py_pb2",
    deps = [":bag_metadata_proto"],
)

proto_library(
    name = "bag_packager_service",
    srcs = ["bag_packager_service.proto"],
    deps = [
        ":bag_metadata_proto",
        "@com_google_protobuf//:empty_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "bag_packager_service_cc_proto",
    deps = [":bag_packager_service"],
)

go_proto_library(
    name = "bag_packager_service_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
    ],
    importpath = "intrinsic/logging/proto/bag_packager_service_go_proto",
    protos = [":bag_packager_service"],
    deps = [":bag_metadata_go_proto"],
)

py_proto_library(
    name = "bag_packager_service_py_pb2",
    deps = [":bag_packager_service"],
)

py_grpc_library(
    name = "bag_packager_service_py_pb2_grpc",
    srcs = [":bag_packager_service"],
    deps = [":bag_packager_service_py_pb2"],
)

cc_grpc_library(
    name = "bag_packager_service_cc_grpc",
    srcs = [":bag_packager_service"],
    generate_mocks = True,
    grpc_only = True,
    deps = [":bag_packager_service_cc_proto"],
)

proto_library(
    name = "blob_proto",
    srcs = ["blob.proto"],
)

cc_proto_library(
    name = "blob_cc_proto",
    deps = [":blob_proto"],
)

go_proto_library(
    name = "blob_go_proto",
    importpath = "intrinsic/logging/proto/blob_go_proto",
    protos = [":blob_proto"],
)

py_proto_library(
    name = "blob_py_pb2",
    deps = [":blob_proto"],
)

proto_library(
    name = "context_proto",
    srcs = ["context.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "context_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":context_proto"],
)

go_proto_library(
    name = "context_go_proto",
    importpath = "intrinsic/logging/proto/context_go_proto",
    protos = [":context_proto"],
    visibility = ["//visibility:public"],
)

py_proto_library(
    name = "context_py_pb2",
    visibility = ["//visibility:public"],
    deps = [":context_proto"],
)

proto_library(
    name = "critical_event_log",
    srcs = ["critical_event_log.proto"],
    deps = ["@com_google_protobuf//:timestamp_proto"],
)

cc_proto_library(
    name = "critical_event_log_cc_proto",
    deps = [":critical_event_log"],
)

go_proto_library(
    name = "critical_event_log_go_proto",
    importpath = "intrinsic/logging/proto/critical_event_log_go_proto",
    protos = [":critical_event_log"],
)

py_proto_library(
    name = "critical_event_log_py_pb2",
    deps = [":critical_event_log"],
)

proto_library(
    name = "downsampler_proto",
    srcs = ["downsampler.proto"],
    deps = [
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "downsampler_cc_proto",
    deps = [":downsampler_proto"],
)

go_proto_library(
    name = "downsampler_go_proto",
    importpath = "intrinsic/logging/proto/downsampler_go_proto",
    protos = [":downsampler_proto"],
)

py_proto_library(
    name = "downsampler_py_pb2",
    deps = [":downsampler_proto"],
)

proto_library(
    name = "flowstate_event",
    srcs = ["flowstate_event.proto"],
)

cc_proto_library(
    name = "flowstate_event_cc_proto",
    deps = [":flowstate_event"],
)

go_proto_library(
    name = "flowstate_event_go_proto",
    importpath = "intrinsic/logging/proto/flowstate_event_go_proto",
    protos = [":flowstate_event"],
)

py_proto_library(
    name = "flowstate_event_py_pb2",
    deps = [":flowstate_event"],
)

proto_library(
    name = "pubsub_listener_service",
    srcs = ["pubsub_listener_service.proto"],
)

cc_proto_library(
    name = "pubsub_listener_service_cc_proto",
    deps = [":pubsub_listener_service"],
)

py_proto_library(
    name = "pubsub_listener_service_py_pb2",
    deps = [":pubsub_listener_service"],
)

py_grpc_library(
    name = "pubsub_listener_service_py_pb2_grpc",
    srcs = [":pubsub_listener_service"],
    deps = [":pubsub_listener_service_py_pb2"],
)

cc_grpc_library(
    name = "pubsub_listener_service_cc_grpc",
    srcs = [":pubsub_listener_service"],
    grpc_only = True,
    deps = [":pubsub_listener_service_cc_proto"],
)

proto_library(
    name = "recording_data_categories_proto",
    srcs = ["recording_data_categories.proto"],
)

cc_proto_library(
    name = "recording_data_categories_cc_proto",
    deps = [":recording_data_categories_proto"],
)

go_proto_library(
    name = "recording_data_categories_go_proto",
    importpath = "intrinsic/logging/proto/recording_data_categories_go_proto",
    protos = [":recording_data_categories_proto"],
)

py_proto_library(
    name = "recording_data_categories_py_pb2",
    deps = [":recording_data_categories_proto"],
)

proto_library(
    name = "replay_service",
    srcs = ["replay_service.proto"],
    deps = [
        ":replay_service_options",
        "@com_google_googleapis//google/api:annotations_proto",
        "@intrinsic_apis//intrinsic/kubernetes/generator/proto:auth_options_proto",
    ],
)

cc_proto_library(
    name = "replay_service_cc_proto",
    deps = [":replay_service"],
)

go_proto_library(
    name = "replay_service_go_proto",
    compilers = [
        "@io_bazel_rules_go//proto:go_grpc_v2",
        "@io_bazel_rules_go//proto:go_proto",
        "@com_github_grpc_ecosystem_grpc_gateway_v2//protoc-gen-grpc-gateway:go_gen_grpc_gateway",
    ],
    importpath = "intrinsic/logging/proto/replay_service_go_proto",
    protos = [":replay_service"],
    deps = [
        ":replay_service_options_go_proto",
        "@intrinsic_apis//intrinsic/kubernetes/generator/proto:auth_options_go_proto",
        "@org_golang_google_genproto_googleapis_api//annotations",
    ],
)

py_proto_library(
    name = "replay_service_py_pb2",
    deps = [":replay_service"],
)

py_grpc_library(
    name = "replay_service_py_pb2_grpc",
    srcs = [":replay_service"],
    deps = [":replay_service_py_pb2"],
)

cc_grpc_library(
    name = "replay_service_cc_grpc",
    srcs = [":replay_service"],
    generate_mocks = True,
    grpc_only = True,
    deps = [":replay_service_cc_proto"],
)

proto_library(
    name = "replay_service_options",
    srcs = ["replay_service_options.proto"],
    deps = [
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "replay_service_options_cc_proto",
    deps = [":replay_service_options"],
)

go_proto_library(
    name = "replay_service_options_go_proto",
    importpath = "intrinsic/logging/proto/replay_service_options_go_proto",
    protos = [":replay_service_options"],
)

py_proto_library(
    name = "replay_service_options_py_pb2",
    deps = [":replay_service_options"],
)
