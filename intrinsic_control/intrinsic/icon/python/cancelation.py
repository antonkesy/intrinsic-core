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

"""A class that manages the cancelation of skills running ICON."""

import datetime
import threading
from typing import Optional

from intrinsic.icon.actions import stop_utils
from intrinsic.icon.python import icon_api
from intrinsic.skills.python import skill_canceller

# The maximum amount of time to wait until the final stop action is done. This
# timeout should never be reached.
CANCELATION_TIMEOUT_S = 5.0


class IconSkillCanceller:
  """A class that can cancel a skill using ICON."""

  def __init__(
      self,
      session: icon_api.Session,
      canceller: skill_canceller.SkillCanceller,
      start_action: icon_api.Action,
      final_stop_action: icon_api.Action,
      success_condition: icon_api.Condition,
  ):
    """Initializes the IconSkillCanceller.

    Args:
      session: The active ICON session.
      canceller: The skill canceller.
      start_action: The action to start.
      final_stop_action: The final stop action in the state machine. This action
        will be active if the state machine finishes successfully. This action
        will be started if the skill is canceled.
      success_condition: The condition on the final_stop_action that will be
        checked to determine if the action was successful. Usually this is
        is_done or is_settled.

    Raises:
      TypeError: If final_stop_action is not a Stop action.
    """
    if final_stop_action.proto.action_type_name != stop_utils.ACTION_TYPE_NAME:
      raise TypeError(
          "final_stop_action must be a stop action, but got"
          f" {final_stop_action.proto.action_type_name}."
      )
    self._condition = threading.Condition()
    self._canceled = False
    self._done = False
    self._timeout = False
    self._session = session
    self._canceller = canceller
    self._start_action_id = start_action.id
    self._final_stop_action_id = final_stop_action.id

    def cancel_callback():
      with self._condition:
        self._canceled = True
        self._condition.notify_all()

    self._canceller.register_callback(cancel_callback)

    def success_callback(
        timestamp: datetime.datetime,
        previous_action_id: Optional[int],
        current_action_id: Optional[int],
    ):
      del timestamp, previous_action_id, current_action_id
      with self._condition:
        self._condition.notify_all()

    self._session.add_reaction(
        final_stop_action,
        success_condition,
        success_callback,
    )

    self._final_action_is_done = self._session.add_reaction(
        final_stop_action,
        icon_api.Condition.is_done(),
    )

  def was_canceled(self) -> bool:
    with self._condition:
      return self._canceled

  def timed_out(self) -> bool:
    with self._condition:
      return self._timeout

  def start_and_wait(
      self,
      timeout_s: float | None = None,
  ) -> None:
    """Starts an action and waits for it to finish or for cancellation.

    The final stop action is started if the skill is canceled or if the timeout
    is
    reached.

    Args:
      timeout_s: The maximum number of seconds to wait for the action to finish.

    Returns:
      True if timeout was reached, False otherwise.

    Raises:
      RuntimeError: If start_and_wait is called more than once.
    """
    if self._done:
      raise RuntimeError("start_and_wait can only be called once.")

    with self._condition:
      self._session.start_action(self._start_action_id)
      self._canceller.ready()
      self._timeout = not self._condition.wait(timeout=timeout_s)

    if self.was_canceled() or self.timed_out():
      self._session.start_action(self._final_stop_action_id)
      if not self._final_action_is_done.wait(timeout=CANCELATION_TIMEOUT_S):
        raise RuntimeError("Cancelation did not finish within the 5 seconds.")

    self._done = True
