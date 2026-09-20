load("@rules_license//rules:license.bzl", "license")

package(
    default_applicable_licenses = ["//third_party/abb_hardware_module:license"],
    default_visibility = ["//visibility:private"],
)

license(
    name = "license",
    package_name = "abb_hardware_module",
    license_kinds = [
        "@rules_license//licenses/generic:notice",
    ],
)

exports_files(["LICENSE"])
