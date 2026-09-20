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
    name = "opencensus",
    srcs = ["opencensus.cc"],
    hdrs = [
        "opencensus.h",
        "opencensus-internal.h",
    ],
    deps = [
        ":internal_constants",
        ":metrics_utils",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/strings:str_format",
        "@abseil-cpp//absl/time",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_grpc_grpc//:grpc_opencensus_plugin",
        "@com_github_grpc_grpc//:grpcpp_otel_plugin",
        "@com_github_jupp0r_prometheus_cpp//core",  # buildcleaner: keep
        "@com_github_jupp0r_prometheus_cpp//pull",  # buildcleaner: keep
        "@com_google_googleapis//google/rpc:code_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@io_opencensus_cpp//opencensus/exporters/stats/prometheus:prometheus_exporter",
        "@io_opencensus_cpp//opencensus/stats",
        "@io_opencensus_cpp//opencensus/tags",
        "@io_opentelemetry_cpp//api",
        "@io_opentelemetry_cpp//exporters/otlp:otlp_grpc_exporter",
        "@io_opentelemetry_cpp//exporters/prometheus:prometheus_exporter",
        "@io_opentelemetry_cpp//sdk/src/metrics",
        "@io_opentelemetry_cpp//sdk/src/resource",
        "@io_opentelemetry_cpp//sdk/src/trace",
    ],
)

cc_library(
    name = "internal_constants",
    hdrs = ["internal_constants.h"],
    visibility = ["//intrinsic/stats:__pkg__"],
)

cc_library(
    name = "scoped_span",
    srcs = ["scoped_span.cc"],
    hdrs = ["scoped_span.h"],
    deps = [
        ":tracing_utils",
        "@abseil-cpp//absl/strings",
        "@com_github_grpc_grpc//:grpc++",
        "@io_opentelemetry_cpp//api",
    ],
)

cc_library(
    name = "tracing_utils",
    srcs = ["tracing_utils.cc"],
    hdrs = ["tracing_utils.h"],
    visibility = ["//intrinsic:intrinsic_internal"],
    deps = [
        ":internal_constants",
        "@com_github_grpc_grpc//:grpc++",
        "@io_opentelemetry_cpp//api",
    ],
)

cc_library(
    name = "metrics_utils",
    srcs = ["metrics_utils.cc"],
    hdrs = ["metrics_utils.h"],
    visibility = ["//intrinsic:intrinsic_internal"],
    deps = [
        "@abseil-cpp//absl/strings:string_view",
        "@abseil-cpp//absl/types:span",
        "@io_opentelemetry_cpp//api",
        "@io_opentelemetry_cpp//sdk/src/metrics",
    ],
)
