load("//bazel:cc_macros.bzl", "cc_library")

package(default_applicable_licenses = ["//third_party/abb_hardware_module:license"])

cc_library(
    name = "rate_monitor",
    srcs = ["rate_monitor.cc"],
    hdrs = ["rate_monitor.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//intrinsic_control/intrinsic/icon/testing:realtime_annotations",
        "//intrinsic_control/intrinsic/icon/utils:core_time",
        "//intrinsic_control/intrinsic/icon/utils:fixed_str_cat",
        "//intrinsic_control/intrinsic/icon/utils:realtime_status",
        "//intrinsic_control/intrinsic/icon/utils:time",
        "@abseil-cpp//absl/status",
    ],
)
