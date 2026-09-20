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

"""Bazel repository rule for downloading NVIDIA NCCL libraries."""

def _nccl_from_redist_impl(repository_ctx):
    """Implementation for the nccl_from_redist rule."""
    nccl_version = repository_ctx.attr.nccl_version
    cuda_version = repository_ctx.attr.cuda_version
    arch = repository_ctx.attr.arch
    sha256 = repository_ctx.attr.sha256
    base_url = "https://developer.download.nvidia.com/compute/redist/nccl/"

    # NCCL URL format is different and constructed directly.
    # e.g., v2.27.5/nccl_2.27.5-1+cuda12.9_x86_64.txz
    # Note: The CUDA version in the URL might have a different patch version.
    # We will use the major.minor from the user's cuda_version attr.
    cuda_major_minor = ".".join(cuda_version.split(".")[:2])

    archive_prefix = "nccl_{nccl_version}-1+cuda{cuda_ver}_{arch}".format(
        nccl_version = nccl_version,
        cuda_ver = cuda_major_minor,
        arch = arch,
    )
    download_url = "{base}v{nccl_version}/{archive}.txz".format(
        base = base_url,
        nccl_version = nccl_version,
        archive = archive_prefix,
    )

    repository_ctx.download_and_extract(
        url = download_url,
        output = "nccl",
        sha256 = sha256,
        type = "tar.xz",  # .txz is a tar.xz file
        strip_prefix = archive_prefix,
    )

    # We can reuse the same generic BUILD template.
    # The NCCL archive extracts into 'lib' and 'include' directories.
    repository_ctx.template(
        "nccl/BUILD.bazel",
        repository_ctx.attr._component_build_file_template,
        # The template only needs a name for the main filegroup.
        {"%{component_name}%": "nccl"},
    )

    return repository_ctx.repo_metadata(reproducible = True)

nccl_from_redist = repository_rule(
    implementation = _nccl_from_redist_impl,
    attrs = {
        "arch": attr.string(default = "x86_64"),
        "cuda_version": attr.string(mandatory = True, doc = "The target CUDA version, e.g., '12.6.2'. Used to construct the URL."),
        "nccl_version": attr.string(mandatory = True),
        "sha256": attr.string(mandatory = True, doc = "The SHA256 checksum of the NCCL .txz archive."),
        "_component_build_file_template": attr.label(
            default = Label("//intrinsic/production/external/nvidia_libs/private:BUILD.nvidia_lib.tpl"),
            allow_single_file = True,
        ),
    },
)
