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

"""Internal utilities for fetching NVIDIA libraries."""

def _fetch_nvidia_libs_impl(
        repository_ctx,
        base_url,
        version,
        manifest_sha256,
        platform_key,
        components,
        component_build_file_template,
        archive_info_extractor):
    """
    Generic implementation to download and extract NVIDIA libraries.

    Args:
        repository_ctx: The repository context.
        base_url: The base URL for downloads.
        version: The library version (e.g., "12.6.2").
        manifest_sha256: The SHA256 of the manifest file.
        platform_key: The platform identifier string (e.g., "linux-x86_64").
        components: A list of component names to download.
        component_build_file_template: The label for the BUILD file template.
        archive_info_extractor: A function that takes a component's manifest data
                                and returns a struct with 'relative_path' and 'sha256'.
    """
    manifest_filename = "redistrib_%s.json" % version
    repository_ctx.download(
        url = base_url + manifest_filename,
        output = manifest_filename,
        sha256 = manifest_sha256,
    )
    manifest_content = repository_ctx.read(manifest_filename)
    parsed_manifest = json.decode(manifest_content)

    for component in components:
        component_data = parsed_manifest.get(component)
        if not component_data:
            fail("Rule: Component '%s' not found in manifest for version %s." % (component, version))

        archive_info = archive_info_extractor(component_data, platform_key, component)
        if not archive_info.relative_path or not archive_info.sha256:
            fail("Rule: Missing 'relative_path' or 'sha256' for component '%s' in manifest." % component)

        archive_filename = archive_info.relative_path.split("/")[-1]
        archive_type = "zip" if archive_filename.endswith(".zip") else "tar.xz"
        strip_prefix_val = archive_filename.removesuffix(".tar.xz").removesuffix(".zip")

        repository_ctx.download_and_extract(
            url = base_url + archive_info.relative_path,
            output = component,
            sha256 = archive_info.sha256,
            type = archive_type,
            strip_prefix = strip_prefix_val,
        )

        repository_ctx.template(
            component + "/BUILD.bazel",
            component_build_file_template,
            {"%{component_name}%": component},
        )

fetch_nvidia_libs_impl = _fetch_nvidia_libs_impl
