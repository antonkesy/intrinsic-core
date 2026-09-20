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

import triton_python_backend_utils as pb_utils


class TritonPythonModel:

  def initialize(self, args):
    pass

  def execute(self, requests):
    responses = []
    for request in requests:
      input_tensor = pb_utils.get_input_tensor_by_name(request, "input_0")
      output_tensor = pb_utils.Tensor("output_0", input_tensor.as_numpy())
      responses.append(
          pb_utils.InferenceResponse(output_tensors=[output_tensor])
      )
    return responses

  def finalize(self):
    pass
