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

"""PredictRequest type for calls to Skill.predict."""

import dataclasses
from typing import Generic
from typing import TypeVar

from google.protobuf import message

TParamsType = TypeVar('TParamsType', bound=message.Message)


@dataclasses.dataclass(frozen=True)
class PredictRequest(Generic[TParamsType]):
  """A request for a call to Skill.predict.

  Attributes:
    internal_data: Skill-specific data that can be communicated from previous
      calls to `predict`. Can be useful for optimizing skill execution by
      pre-computing plan-related information.
    params: The skill parameters proto. For static typing, PredictRequest can be
      parameterized with the required type of this message.
  """

  internal_data: bytes
  params: TParamsType
