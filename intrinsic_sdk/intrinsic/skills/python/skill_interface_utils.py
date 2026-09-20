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

"""Utils for Skill implementations."""

from __future__ import annotations

from intrinsic.resources.proto import resource_handle_pb2
from intrinsic.skills.internal import execute_context_impl

# isort: off

from intrinsic.skills.internal import predict_context_impl
from intrinsic.skills.internal import preview_context_impl

# isort: on
from intrinsic.skills.python import skill_interface


def preview_via_execute(
    skill: skill_interface.Skill[
        skill_interface.TParamsType, skill_interface.TResultType
    ],
    request: skill_interface.PreviewRequest[skill_interface.TParamsType],
    context: skill_interface.PreviewContext,
) -> skill_interface.TResultType:
  """Implements Skill.preview by calling Skill.execute.

  A skill can use this function to implement `preview` by calling
  `preview_via_execute` from within its implementation. E.g.:
  ```
  class MySkill(Skill):
    def preview(self, request: PreviewRequest, context: PreviewContext) -> ...:
      ...
      return preview_via_execute(self, request, context)
  ```

  A skill should only use this util to implement `preview` if its `execute`
  method does not require resources or modify the object world.

  Args:
    skill: The skill instance.
    request: The preview request.
    context: The preview context.

  Returns:
    The response from calling `skill.execute`.
  """
  return skill.execute(
      preview_to_execute_request(request),
      preview_to_execute_context(
          context=context,
          resource_handles={},
      ),
  )



def preview_via_predict(
    skill: skill_interface.Skill[
        skill_interface.TParamsType, skill_interface.TResultType
    ],
    request: skill_interface.PreviewRequest[skill_interface.TParamsType],
    context: skill_interface.PreviewContext,
    result: skill_interface.TResultType,
) -> skill_interface.TResultType:
  """Implements Skill.preview as the most likely outcome of Skill.predict.

  A skill can use this function to implement `preview` by calling
  `preview_via_predict` from within its implementation. E.g.:
  ```
  class MySkill(Skill):
    def preview(self, request: PreviewRequest, context: PreviewContext) -> ...:
      ...
      return preview_via_predict(
          skill=self, request=request, context=context, result=MyResultProto()
      )
  ```

  Args:
    skill: The skill instance.
    request: The preview request.
    context: The preview context.
    result: The result proto into which to unpack the most likely result, or
      None if the skill does not produce a result.

  Returns:
    The unpacked most likely result from calling predict.
  """
  predict_result = skill.predict(
      preview_to_predict_request(request),
      preview_to_predict_context(context),
  )

  if len(predict_result.outcomes) < 1:
    raise ValueError(f"{type(skill).__name__} produced no predictions.")

  most_likely_outcome = max(
      predict_result.outcomes, key=lambda outcome: outcome.probability
  )

  # Record the world updates from the most likely outcome.
  previous_start_time = 0.0
  previous_duration = 0.0
  for idx, expected_state in enumerate(most_likely_outcome.expected_states):
    start_time = (
        (expected_state.start_time.ToNanoseconds() / 1e9)
        if expected_state.HasField("start_time")
        else previous_start_time + previous_duration
    )
    duration = expected_state.time_until_update.ToNanoseconds() / 1e9

    elapsed = start_time - previous_start_time
    if elapsed < 0:
      raise ValueError(
          "Expected state start times must be monotonically increasing"
          f" (state[{idx}] start time: {start_time}, state[{idx - 1}] start"
          f" time: {previous_start_time})."
      )

    for update in expected_state.world_updates.updates:
      context.record_world_update(
          update=update, elapsed=elapsed, duration=duration
      )

      # All updates in this expected state started at the same time, so all but
      # the first update should have `elapsed = 0`.
      elapsed = 0.0

    previous_start_time = start_time
    previous_duration = duration

  # Get the result of the most likely outcome.
  if result is not None:
    most_likely_outcome.result.Unpack(result)

  return result





def preview_to_execute_request(
    request: skill_interface.PreviewRequest[skill_interface.TParamsType],
) -> skill_interface.ExecuteRequest[skill_interface.TParamsType]:
  """Converts a PreviewRequest to an ExecuteRequest."""
  return skill_interface.ExecuteRequest(

      internal_data=request.internal_data,

      params=request.params,
  )


def preview_to_execute_context(
    context: skill_interface.PreviewContext,
    resource_handles: dict[str, resource_handle_pb2.ResourceHandle],
) -> skill_interface.ExecuteContext:
  """Converts a PreviewContext to an ExecuteContext."""
  return execute_context_impl.ExecuteContextImpl(
      canceller=context.canceller,

      geometry_service=context.geometry_service,

      logging_context=context.logging_context,
      motion_planner=context.motion_planner,
      object_world=context.object_world,
      resource_handles=resource_handles,
      context_id=context.context_id,
  )



def preview_to_predict_request(
    request: skill_interface.PreviewRequest[skill_interface.TParamsType],
) -> skill_interface.PredictRequest[skill_interface.TParamsType]:
  """Converts a PreviewRequest to a PredictRequest."""
  return skill_interface.PredictRequest(
      internal_data=request.internal_data,
      params=request.params,
  )


def preview_to_predict_context(
    context: skill_interface.PreviewContext,
) -> skill_interface.PredictContext:
  """Converts a PreviewContext to a PredictContext."""
  resource_handles = (
      context._resource_handles  # pylint: disable=protected-access
      if isinstance(context, preview_context_impl.PreviewContextImpl)
      else {}
  )

  return predict_context_impl.PredictContextImpl(

      geometry_service=context.geometry_service,

      motion_planner=context.motion_planner,
      object_world=context.object_world,
      resource_handles=resource_handles,
  )



