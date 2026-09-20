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
Wrapper macros for proto_library to facilitate folder migration.

This file exists to facilitate the migration out of the google3 submodule into IOC roots.
It dynamically injects `strip_import_prefix` into proto targets so that legacy
`import "intrinsic/...";` statements don't break after directories move.
"""

# TODO(b/490376984): Remove after Google3 submodule removal

load("@com_google_protobuf//bazel:proto_library.bzl", _native_proto_library = "proto_library")
load("//bazel:migration_locations.bzl", "GOOGLE3_OR_IOC_LOCATIONS")

def proto_library(name, **kwargs):
    """Dynamically strips the prefix so proto imports remain stable as directories move.

    Args:
      name: The name of the target.
      **kwargs: Additional keyword arguments passed to native proto_library.
    """
    pkg = native.package_name()

    # Strip the root location prefix for packages under intrinsic/ (or google3/) so legacy
    # `import "intrinsic/...";` statements resolve across Google3 and IOC repositories,
    # while leaving non-intrinsic packages (such as service/) unstripped.
    if "strip_import_prefix" not in kwargs:
        for location in GOOGLE3_OR_IOC_LOCATIONS:
            is_ioc_intrinsic = pkg == location + "/intrinsic" or pkg.startswith(location + "/intrinsic/")
            is_google3 = location == "google3" and (pkg == "google3" or pkg.startswith("google3/"))
            if is_ioc_intrinsic or is_google3:
                kwargs["strip_import_prefix"] = "/" + location
                break

    _native_proto_library(name = name, **kwargs)
