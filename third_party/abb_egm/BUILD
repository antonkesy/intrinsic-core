load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_license//rules:license.bzl", "license")
load("//bazel:protos.bzl", "proto_library")

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:private"],
)

license(
    name = "license",
    package_name = "abb_egm",
    license_kinds = [
        "@rules_license//licenses/spdx:BSD-3-Clause",
    ],
    license_text = "LICENSE",
)

exports_files(["LICENSE"])

proto_library(
    name = "egm_proto",
    srcs = ["egm.proto"],
)

cc_proto_library(
    name = "egm_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":egm_proto"],
)
