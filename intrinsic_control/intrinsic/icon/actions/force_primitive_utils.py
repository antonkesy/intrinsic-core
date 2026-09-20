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

"""Helper module to construct a ForcePrimitive action."""

from intrinsic.icon.actions import cartesian_admittance_action_pb2
from intrinsic.icon.actions import cartesian_admittance_utils
from intrinsic.icon.actions import force_primitive_pb2
from intrinsic.icon.control.primitives.force_control.proto import force_primitives_pb2
from intrinsic.icon.python import actions
from intrinsic.manipulation.skills.force import contact_stiffness_pb2
from intrinsic.manipulation.skills.force import utils
from intrinsic.math.python import data_types
from intrinsic.math.python import proto_conversion

ACTION_TYPE_NAME = "intrinsic.force_primitive"

StateVariables = cartesian_admittance_utils.StateVariables

StreamingOutputType = cartesian_admittance_action_pb2.CartesianAdmittanceStatus


def build_task_params(
    robot_tip_t_robot_tool: data_types.Pose3,
    robot_base_t_task: data_types.Pose3,
    contact_stiffness: contact_stiffness_pb2.ContactStiffnessParams,
) -> force_primitive_pb2.TaskParams:
  """Create the taskParams proto from Python types."""
  return force_primitive_pb2.TaskParams(
      robot_tip_t_robot_tool=proto_conversion.pose_to_proto(
          robot_tip_t_robot_tool
      ),
      robot_base_t_task=proto_conversion.pose_to_proto(robot_base_t_task),
      environment_stiffness=contact_stiffness,
  )


def create_make_contact_action(
    action_id: int,
    joint_position_part_name: str,
    ft_sensor_part_name: str,
    primitive: force_primitives_pb2.MakeContact,
    force_control_settings: utils.ForceControlParams,
    task_params: force_primitive_pb2.TaskParams,
    disable_reference_lowpass_filter: bool = False,
) -> actions.Action:
  """Create a ForcePrimitive action for the MakeContact primitive."""
  return _create_force_primitive_action(
      action_id,
      joint_position_part_name,
      ft_sensor_part_name,
      force_primitives_pb2.ForcePrimitive(make_contact=primitive),
      force_control_settings,
      task_params,
      disable_reference_lowpass_filter,
  )


def _create_force_primitive_action(
    action_id: int,
    joint_position_part_name: str,
    ft_sensor_part_name: str,
    primitive: force_primitives_pb2.ForcePrimitive,
    force_control_settings: utils.ForceControlParams,
    task_params: force_primitive_pb2.TaskParams,
    disable_reference_lowpass_filter: bool = False,
) -> actions.Action:
  """Creates a ForcePrimitive action."""
  return actions.Action(
      action_id,
      ACTION_TYPE_NAME,
      {"arm": joint_position_part_name, "ft_sensor": ft_sensor_part_name},
      params=force_primitive_pb2.ForcePrimitiveFixedParams(
          force_control_settings=force_control_settings.proto(),
          force_primitive=primitive,
          task_params=task_params,
          disable_reference_lowpass_filter=disable_reference_lowpass_filter,
      ),
  )
