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

"""Bazel macros for OpenGL cc_test targets."""

load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("//bazel:cc_macros.bzl", "cc_test")

def cc_opengl_test(name, srcs, deps = [], tags = [], env = {}, **kwargs):
    """Wrapper around cc_test for tests requiring OpenGL and NVIDIA drivers.

    Args:
      name: The name of the test.
      srcs: A list of C++ source files for the test.
      deps: A list of dependencies for the test.
      tags: A list of tags for the test.
      env: A dictionary of key-value pairs to be passed to the test environment.
      **kwargs: Additional arguments to pass to cc_test.
    """

    # Needed so that the test can use the NVIDIA driver.
    env = env | {
        "NVIDIA_DRIVER_CAPABILITIES": "compute,graphics,utility",
    }

    deps = deps + [
        "@sysroot_glvnd_stubs",
    ]

    cc_test(
        name = name,
        srcs = srcs,
        tags = tags,
        deps = deps,
        env = env,
        **kwargs
    )

def _wrap_py_opengl_rule(
        py_rule,
        name,
        srcs,
        data = [],
        env = {},
        **kwargs):
    """Wrapper around Python rules for targets requiring OpenGL and NVIDIA drivers.

    Args:
      py_rule: Actual Python rule to instantiate.
      name: The name of the target.
      srcs: A list of Python source files.
      data: A list of data dependencies.
      env: A dictionary of key-value pairs to be passed to the environment.
      **kwargs: Additional arguments to pass.
    """

    data = data + [
        "@sysroot_glvnd_stubs//:files",
    ]

    # Needed so that the test can use the NVIDIA driver.
    glvnd_stubs_pkg = Label("@sysroot_glvnd_stubs//:files").repo_name
    existing_ld_path = env.get("LD_LIBRARY_PATH") or ""
    env = env | {
        # ".." must be prefixed to the repo_name in order to escape out of "_main".
        "LD_LIBRARY_PATH": "../{}/usr/lib/x86_64-linux-gnu/:{}".format(
            glvnd_stubs_pkg,
            existing_ld_path,
        ),
        "NVIDIA_DRIVER_CAPABILITIES": "compute,graphics,utility",
    }

    py_rule(
        name = name,
        srcs = srcs,
        data = data,
        env = env,
        **kwargs
    )

def py_opengl_binary(**kwargs):
    _wrap_py_opengl_rule(py_binary, **kwargs)

def py_opengl_test(**kwargs):
    _wrap_py_opengl_rule(py_test, **kwargs)
