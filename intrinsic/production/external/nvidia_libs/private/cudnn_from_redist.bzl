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

"""Bazel repository rule for downloading NVIDIA CUDNN libraries."""

load(":fetch_nvidia_libs.bzl", "fetch_nvidia_libs_impl")

def _cudnn_archive_info_extractor(repository_ctx):
    """A factory function to create an archive info extractor for cuDNN."""

    def _extractor(component_data, platform_key, component_name):
        platform_data = component_data.get(platform_key)
        if not platform_data:
            fail("CUDNN Rule: Platform '%s' not available for component '%s'." % (platform_key, component_name))

        cudnn_cuda_version = "cuda" + repository_ctx.attr.cudnn_cuda_version
        variant_data = platform_data.get(cudnn_cuda_version)
        if not variant_data:
            fail("CUDNN Rule: CUDA variant '%s' not found for component '%s'." % (cudnn_cuda_version, component_name))

        return struct(
            relative_path = variant_data.get("relative_path"),
            sha256 = variant_data.get("sha256"),
        )

    return _extractor

def _cudnn_from_redist_impl(repository_ctx):
    """Implementation for the cudnn_from_redist rule."""
    fetch_nvidia_libs_impl(
        repository_ctx = repository_ctx,
        base_url = "https://developer.download.nvidia.com/compute/cudnn/redist/",
        version = repository_ctx.attr.cudnn_version,
        manifest_sha256 = repository_ctx.attr.manifest_sha256,
        platform_key = "%s-%s" % (repository_ctx.attr.os_name, repository_ctx.attr.arch),
        components = repository_ctx.attr.components,
        component_build_file_template = repository_ctx.attr._component_build_file_template,
        archive_info_extractor = _cudnn_archive_info_extractor(repository_ctx),
    )
    return repository_ctx.repo_metadata(reproducible = True)

cudnn_from_redist = repository_rule(
    implementation = _cudnn_from_redist_impl,
    attrs = {
        "arch": attr.string(default = "x86_64"),
        "components": attr.string_list(mandatory = True),
        "cudnn_cuda_version": attr.string(default = "12"),
        "cudnn_version": attr.string(mandatory = True),
        "manifest_sha256": attr.string(mandatory = True, doc = "The SHA256 checksum of the redistrib JSON manifest."),
        "os_name": attr.string(default = "linux"),
        "_component_build_file_template": attr.label(
            default = Label("//intrinsic/production/external/nvidia_libs/private:BUILD.nvidia_lib.tpl"),
            allow_single_file = True,
        ),
    },
)
