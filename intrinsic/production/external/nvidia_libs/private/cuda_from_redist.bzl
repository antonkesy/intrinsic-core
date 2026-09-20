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

"""Bazel repository rule for downloading NVIDIA CUDA libraries."""

load(":fetch_nvidia_libs.bzl", "fetch_nvidia_libs_impl")

def _cuda_archive_info_extractor(component_data, platform_key, component_name):
    """Extracts archive info from a CUDA component's manifest data."""
    platform_data = component_data.get(platform_key)
    if not platform_data:
        fail("CUDA Rule: Platform '%s' not available for component '%s'." % (platform_key, component_name))
    return struct(
        relative_path = platform_data.get("relative_path"),
        sha256 = platform_data.get("sha256"),
    )

def _cuda_from_redist_impl(repository_ctx):
    """Implementation for the cuda_from_redist rule."""
    fetch_nvidia_libs_impl(
        repository_ctx = repository_ctx,
        base_url = "https://developer.download.nvidia.com/compute/cuda/redist/",
        version = repository_ctx.attr.cuda_version,
        manifest_sha256 = repository_ctx.attr.manifest_sha256,
        platform_key = "%s-%s" % (repository_ctx.attr.os_name, repository_ctx.attr.arch),
        components = repository_ctx.attr.components,
        component_build_file_template = repository_ctx.attr._component_build_file_template,
        archive_info_extractor = _cuda_archive_info_extractor,
    )
    return repository_ctx.repo_metadata(reproducible = True)

cuda_from_redist = repository_rule(
    implementation = _cuda_from_redist_impl,
    attrs = {
        "arch": attr.string(default = "x86_64"),
        "components": attr.string_list(mandatory = True),
        "cuda_version": attr.string(mandatory = True),
        "manifest_sha256": attr.string(mandatory = True, doc = "The SHA256 checksum of the redistrib JSON manifest."),
        "os_name": attr.string(default = "linux"),
        "_component_build_file_template": attr.label(
            default = Label("//intrinsic/production/external/nvidia_libs/private:BUILD.nvidia_lib.tpl"),
            allow_single_file = True,
        ),
    },
)
