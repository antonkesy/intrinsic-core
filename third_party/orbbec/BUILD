load("@rules_license//rules:license.bzl", "license")

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:public"],
)

license(
    name = "license",
    package_name = "orbbec",
    license_kinds = [
        "@rules_license//licenses/spdx:Apache-2.0",
    ],
    license_text = "LICENSE",
    package_url = "https://github.com/orbbec/OrbbecSDK_ROS2",
)

exports_files([
    "LICENSE",
    "NOTICE",
])

filegroup(
    name = "assets",
    srcs = [
        "BUILD",
        "LICENSE",
        "NOTICE",
        "README.md",
        "gemini_335le/visual.glb",
    ],
)
