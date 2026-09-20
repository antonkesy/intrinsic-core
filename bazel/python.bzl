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

"""
Wrapper macros for rules_python (TEMPORARY DUPLICATE).

This file exists to facilitate the migration out of the google3 submodule.
It dynamically injects `imports = ["/google3"]` into Python targets so that
legacy `import intrinsic...` statements don't break after the submodule is removed.
"""

load("@rules_python//python:defs.bzl", _native_py_binary = "py_binary", _native_py_library = "py_library", _native_py_test = "py_test")
load("//bazel:migration_locations.bzl", "GOOGLE3_OR_IOC_LOCATIONS")

def _inject_python_imports(kwargs):
    """Dynamically adds the root directory to PYTHONPATH so legacy imports keep working."""
    pkg = native.package_name()

    # Check if we are executing from one of the migration locations
    in_migration_location = False
    for location in GOOGLE3_OR_IOC_LOCATIONS:
        if pkg.startswith(location + "/") or pkg == location:
            in_migration_location = True
            break

    if not in_migration_location:
        return kwargs

    depth = len(pkg.split("/")) if pkg else 0
    prefix_path = "../" * depth

    imports = list(kwargs.pop("imports", []))
    for root in GOOGLE3_OR_IOC_LOCATIONS:
        import_path = prefix_path + root
        if import_path not in imports:
            imports.append(import_path)
    kwargs["imports"] = imports
    return kwargs

def py_library(name, **kwargs):
    kwargs = _inject_python_imports(kwargs)
    _native_py_library(name = name, **kwargs)

def py_binary(name, **kwargs):
    kwargs = _inject_python_imports(kwargs)
    _native_py_binary(name = name, **kwargs)

def py_test(name, **kwargs):
    kwargs = _inject_python_imports(kwargs)
    _native_py_test(name = name, **kwargs)
