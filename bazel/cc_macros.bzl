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

"""This file contains starlark macros for building cc targets."""

load(
    "@pybind11_bazel//:build_defs.bzl",
    "PYBIND_COPTS",
    "PYBIND_DEPS",
    "PYBIND_FEATURES",
    _native_pybind_library = "pybind_library",
)
load("@rules_cc//cc:cc_import.bzl", _native_cc_import = "cc_import")
load("@rules_cc//cc:cc_shared_library.bzl", "cc_shared_library")
load(
    "@rules_cc//cc:defs.bzl",
    _native_cc_binary = "cc_binary",
    _native_cc_library = "cc_library",
    _native_cc_test = "cc_test",
)
load("@rules_cuda//cuda:defs.bzl", _native_cuda_library = "cuda_library")
load("//bazel:migration_locations.bzl", _GOOGLE3_OR_IOC_LOCATIONS_FROM_FILE = "GOOGLE3_OR_IOC_LOCATIONS")
load("//bazel:python.bzl", "py_library")
load("//bazel/cpp:linters.bzl", "cc_tidy_rule")

def _cc_dynamic_library_impl(
        name,
        visibility,
        deps,
        dynamic_library_deps,
        exports_filter,
        shared_lib_name):
    # In rules_cc 0.2.22, exports_filter is evaluated using current_label.relative(pattern)
    # inside rules_cc, which resolves repository names against rules_cc's repo mapping instead
    # of the caller's repo. Canonicalize labels in the caller package's context so rules_cc
    # receives canonical @@ labels without breaking package-relative labels.
    if type(exports_filter) == "list":
        exports_filter = [str(native.package_relative_label(f)) for f in exports_filter]

    cc_shared_library(
        name = name + "_shared",
        dynamic_deps = [str(dep) + "_shared" for dep in dynamic_library_deps],
        deps = deps,
        exports_filter = exports_filter,
        shared_lib_name = shared_lib_name,
    )

    _native_cc_import(
        name = name,
        shared_library = name + "_shared",
        deps = deps + dynamic_library_deps,
        visibility = visibility,
    )

cc_dynamic_library = macro(
    doc = """
<p>Use <code>cc_dynamic_library()</code> for C++-compiled libraries.
  Any binaries depending on it will be forced to dynamically link in the <code>deps</code>.
</p>
""",
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            configurable = True,
            doc = "The libraries to forcefully be dynamically linked in.",
        ),
        "dynamic_library_deps": attr.label_list(
            configurable = False,
            doc = "Any other dynamic library dependencies.",
        ),
        "exports_filter": attr.string_list(
            configurable = False,
            doc = "Same as in https://bazel.build/reference/be/c-cpp#cc_shared_library_args.",
        ),
        "shared_lib_name": attr.string(
            doc = "Same as in https://bazel.build/reference/be/c-cpp#cc_shared_library_args.",
        ),
    },
    implementation = _cc_dynamic_library_impl,
)

# ----------------------------------------------------------------------------
# MIGRATION MACROS to ease folder moves
# TODO b/477580650: Remove after folder moves are done.
# ----------------------------------------------------------------------------
# The macros below automatically strip C++ include paths to make it easier to
# move C++ code as part of these initiatives:
#  * b/477580650
#  * go/intrinsic-ioc-transition-tdd
#
# If we make the include paths the same, then we can move C++ files without
# changing most files. That enables git to track the file moves, and it
# reduces the burden on PR reviewers while we do these moves. This is intended
# to be temporary. When we finish moving files we will remove these macros and
# change the #include statements.
#
# List of prefixes to strip from include paths:
GOOGLE3_OR_IOC_LOCATIONS = _GOOGLE3_OR_IOC_LOCATIONS_FROM_FILE
_GOOGLE3_OR_IOC_LOCATIONS = GOOGLE3_OR_IOC_LOCATIONS

def cc_library(name, **kwargs):
    """Dynamically strips the prefix so C++ includes remain stable as directories move.

    Args:
      name: Target name of the library.
      **kwargs: Keyword arguments passed to _native_cc_library.
    """
    pkg = native.package_name()

    # Check if we are executing from the top-level workspace (after cutover)
    for location in GOOGLE3_OR_IOC_LOCATIONS:
        if pkg.startswith(location + "/") or pkg == location:
            if "strip_include_prefix" not in kwargs:
                kwargs["strip_include_prefix"] = "/" + location
            break

    _native_cc_library(name = name, **kwargs)

    tags = kwargs.get("tags") or []
    if "clang-tidy" in tags:
        cc_tidy_rule(
            name = name + "_clang_tidy",
            target = ":" + name,
            testonly = kwargs.get("testonly", False),
            visibility = ["//visibility:private"],
        )

def pybind_library(name, **kwargs):
    """Dynamically strips the prefix so C++ includes remain stable as directories move.

    Args:
      name: Target name of the library.
      **kwargs: Keyword arguments passed to _native_pybind_library.
    """
    pkg = native.package_name()

    # Check if we are executing from the top-level workspace (after cutover)
    for location in _GOOGLE3_OR_IOC_LOCATIONS:
        if pkg.startswith(location + "/") or pkg == location:
            if "strip_include_prefix" not in kwargs:
                kwargs["strip_include_prefix"] = "/" + location
            break

    _native_pybind_library(name = name, **kwargs)

def pybind_extension(
        name,
        copts = [],
        features = [],
        linkopts = [],
        tags = [],
        deps = [],
        **kwargs):
    """Builds a Python extension module using pybind11 and wraps it in a py_library."""

    # Mark common dependencies as required for build_cleaner.
    tags = tags + ["req_dep=%s" % dep for dep in PYBIND_DEPS]

    # Redefine the cc_binary with name + .so
    _native_cc_binary(
        name = name + ".so",
        copts = copts + PYBIND_COPTS + ["-fvisibility=hidden"],
        features = features + PYBIND_FEATURES,
        linkopts = linkopts + ["-Wl,-Bsymbolic"],
        linkshared = 1,
        tags = tags,
        deps = deps + PYBIND_DEPS,
        **kwargs
    )

    # Use py_library as the main target named 'name' so that bazel/python.bzl
    # can inject the correct PYTHONPATH.
    py_library(
        name = name,
        data = [":" + name + ".so"],
        testonly = kwargs.get("testonly"),
        visibility = kwargs.get("visibility"),
    )

def cc_binary(name, **kwargs):
    """Dynamically routes internal headers for binaries.

    Args:
      name: Target name of the binary.
      **kwargs: Keyword arguments passed to _native_cc_binary.
    """
    pkg = native.package_name()
    copts = kwargs.pop("copts", [])

    # We use `+` instead of `.append()` because `copts` might be a `select()`!
    # (Note: appending a duplicate -I flag is harmless to the compiler)
    if pkg.startswith("google3/") or pkg == "google3":
        copts = copts + ["-Igoogle3"]

    _native_cc_binary(name = name, copts = copts, **kwargs)

    tags = kwargs.get("tags") or []
    if "clang-tidy" in tags:
        cc_tidy_rule(
            name = name + "_clang_tidy",
            target = ":" + name,
            testonly = kwargs.get("testonly", False),
            visibility = ["//visibility:private"],
        )

def cc_import(name, **kwargs):
    """Dynamically strips the prefix so C++ includes remain stable as directories move.

    Args:
      name: Target name of the import.
      **kwargs: Keyword arguments passed to _native_cc_import.
    """
    pkg = native.package_name()

    # Check if we are executing from the top-level workspace (after cutover)
    for location in _GOOGLE3_OR_IOC_LOCATIONS:
        if pkg.startswith(location + "/") or pkg == location:
            if "strip_include_prefix" not in kwargs:
                kwargs["strip_include_prefix"] = "/" + location
            break

    _native_cc_import(name = name, **kwargs)

def cc_test(name, **kwargs):
    """Dynamically routes internal headers for tests.

    Args:
      name: Target name of the test.
      **kwargs: Keyword arguments passed to _native_cc_test.
    """
    pkg = native.package_name()
    copts = kwargs.pop("copts", [])

    if pkg.startswith("google3/") or pkg == "google3":
        copts = copts + ["-Igoogle3"]

    _native_cc_test(name = name, copts = copts, **kwargs)

    tags = kwargs.get("tags") or []
    if "clang-tidy" in tags:
        cc_tidy_rule(
            name = name + "_clang_tidy",
            target = ":" + name,
            testonly = True,
            visibility = ["//visibility:private"],
        )

def cuda_library(name, hdrs = None, deps = None, visibility = None, testonly = None, **kwargs):
    """Dynamically strips prefixes for CUDA targets by delegating headers to cc_library.

    Args:
      name: Target name of the CUDA library.
      hdrs: Header files for the library.
      deps: Dependencies for the library.
      visibility: Target visibility.
      testonly: Whether target is testonly.
      **kwargs: Additional keyword arguments passed to _native_cuda_library.
    """
    pkg = native.package_name()
    hdrs = hdrs or []
    deps = deps or []

    strip_prefix = None
    for location in _GOOGLE3_OR_IOC_LOCATIONS:
        if pkg.startswith(location + "/") or pkg == location:
            strip_prefix = "/" + location
            break

    # If we are in the inner workspace (pre-cutover), do nothing special.
    if not strip_prefix:
        _native_cuda_library(
            name = name,
            hdrs = hdrs,
            deps = deps,
            visibility = visibility,
            testonly = testonly,
            **kwargs
        )
        return

    # Split the target to get the strip_include_prefix magic
    hdrs_name = name + "_hdrs_internal"
    _native_cc_library(
        name = hdrs_name,
        hdrs = hdrs,
        strip_include_prefix = strip_prefix,
        deps = deps,
        testonly = testonly,
        visibility = ["//visibility:private"],
    )

    cuda_name = name + "_cu_internal"
    _native_cuda_library(
        name = cuda_name,
        deps = deps + [hdrs_name],
        testonly = testonly,
        visibility = ["//visibility:private"],
        **kwargs
    )

    # Bundle back together for downstream consumers
    _native_cc_library(
        name = name,
        deps = [cuda_name, hdrs_name],
        visibility = visibility,
        testonly = testonly,
    )
