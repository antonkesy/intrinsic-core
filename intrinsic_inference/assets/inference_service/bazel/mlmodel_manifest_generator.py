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

"""Generates a serialized data manifest for use in the intrinsic_data rule."""

from absl import app
from absl import flags
from google.protobuf import text_format

from intrinsic_inference.assets.inference_service.bazel import backend_parsers
from intrinsic_inference.assets.inference_service.v1 import ml_model_asset_pb2
from intrinsic_inference.core.v1 import ml_model_pb2
from intrinsic.assets.data.proto.v1 import data_manifest_pb2

_CONFIG = flags.DEFINE_string(
    "config", None, "Path to the backend config file", required=True
)
_BACKEND = flags.DEFINE_string(
    "backend",
    "triton",
    "The backend type ('triton' by default)",
)
_ID = flags.DEFINE_string(
    "id", None, "Asset ID, e.g. ai.intrinsic.my_model", required=True
)
_DISPLAY_NAME = flags.DEFINE_string(
    "display_name", None, "Asset display name", required=True
)
_DESCRIPTION = flags.DEFINE_string("description", None, "Asset description")
_VENDOR = flags.DEFINE_string("vendor", "Intrinsic", "Asset vendor")
_VERSION = flags.DEFINE_string("version", "0.0.1", "Model version")
_SOURCE_PROJECT = flags.DEFINE_string("source_project", None, "Assets project.")
_MODEL_FILE = flags.DEFINE_multi_string(
    "model_file",
    [],
    "Format: target_path:package_relative_source_path",
)
_OUTPUT = flags.DEFINE_string("output", None, "Path to output textproto")


def main(argv):
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  try:
    parser = backend_parsers.get_parser(_BACKEND.value)
    model_config, backend_config = parser.parse_config(
        _CONFIG.value, _VERSION.value
    )
  except Exception as e:
    raise ValueError(
        f"Failed to parse config for backend '{_BACKEND.value}' at"
        f" {_CONFIG.value}: {e}"
    ) from e

  # Build MlModel proto.
  ml_model = ml_model_pb2.MlModel()
  ml_model.model_config.CopyFrom(model_config)

  # Pack backend_config as Any.
  ml_model.backend_config.Pack(backend_config)
  ml_model_asset = ml_model_asset_pb2.MlModelAsset()

  # Populate model_data and referenced_data.
  for mf in _MODEL_FILE.value:
    target_path, reference = mf.split(":", 1)
    ml_model.model_data[target_path].reference = reference
    ml_model_asset.referenced_data[target_path].reference = reference
    if reference.startswith("intcas://"):
      if not _SOURCE_PROJECT.value:
        raise ValueError(f"source_project not specified for file: {reference}")
      ml_model_asset.referenced_data[target_path].source_project = (
          _SOURCE_PROJECT.value
      )

  ml_model_asset.ml_model.CopyFrom(ml_model)

  # Build DataManifest.
  manifest = data_manifest_pb2.DataManifest()

  # Split ID into package and name.
  # We require the ID to be in the format 'pkg.name' where 'pkg' has at least
  # two parts (e.g. 'ai.intrinsic.my_model').
  id_parts = _ID.value.split(".")
  if len(id_parts) < 3:
    raise ValueError(
        f"Invalid ID '{_ID.value}'. Expected format 'pkg.name' where 'pkg' has"
        " at least two parts (e.g., 'ai.intrinsic.model_name')."
    )

  manifest.metadata.id.package = ".".join(id_parts[:-1])
  manifest.metadata.id.name = id_parts[-1]

  manifest.metadata.display_name = _DISPLAY_NAME.value
  manifest.metadata.vendor.display_name = _VENDOR.value
  manifest.metadata.documentation.description = _DESCRIPTION.value

  # Pack MlModelAsset into manifest.data Any proto field.
  manifest.data.Pack(ml_model_asset)

  # Write output.
  with open(_OUTPUT.value, "w", encoding="utf-8") as f:
    f.write(text_format.MessageToString(manifest, as_utf8=True))


if __name__ == "__main__":
  app.run(main)
