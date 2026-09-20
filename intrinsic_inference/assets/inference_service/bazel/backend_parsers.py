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

"""Backend parsers for ML model manifest generation."""

import abc

from google.protobuf import message
from google.protobuf import text_format
from triton_common.protobuf import model_config_pb2

from intrinsic_inference.core.v1 import ml_model_pb2


class BackendParser(abc.ABC):
  """Interface for parsing backend-specific configurations."""

  @abc.abstractmethod
  def parse_config(
      self, config_path: str, version: str
  ) -> tuple[ml_model_pb2.MlModelConfig, message.Message]:
    """Parses the backend config file.

    Args:
      config_path: Path to the backend config file.
      version: Model version.

    Returns:
      A tuple containing:
        - ml_model_pb2.MlModelConfig: The platform-agnostic model config.
        - message.Message: The backend-specific config proto (to be packed into
          Any).
    """
    pass


class TritonParser(BackendParser):
  """Parser for Triton backend configuration."""

  def _parse_triton_datatype(self, triton_type: int) -> str:
    """Maps Triton DataType enum to platform-agnostic datatype string."""
    prefix = "TYPE_"
    name = model_config_pb2.DataType.Name(triton_type)
    if name.startswith(prefix):
      return name[len(prefix) :]
    return name

  def parse_config(
      self, config_path: str, version: str
  ) -> tuple[ml_model_pb2.MlModelConfig, message.Message]:
    triton_config = model_config_pb2.ModelConfig()
    with open(config_path, "r", encoding="utf-8") as f:
      text_format.Parse(f.read(), triton_config)

    model_config = ml_model_pb2.MlModelConfig()
    model_config.name = triton_config.name
    model_config.version = version

    for model_input in triton_config.input:
      tensor = model_config.inputs.add()
      tensor.name = model_input.name
      tensor.datatype = self._parse_triton_datatype(model_input.data_type)
      tensor.shape.extend(model_input.dims)

    for model_output in triton_config.output:
      tensor = model_config.outputs.add()
      tensor.name = model_output.name
      tensor.datatype = self._parse_triton_datatype(model_output.data_type)
      tensor.shape.extend(model_output.dims)

    return model_config, triton_config


_PARSERS = {
    "triton": TritonParser,
}


def get_parser(backend_name: str) -> BackendParser:
  if backend_name not in _PARSERS:
    raise ValueError(f"Unsupported backend: {backend_name}")
  return _PARSERS[backend_name]()
