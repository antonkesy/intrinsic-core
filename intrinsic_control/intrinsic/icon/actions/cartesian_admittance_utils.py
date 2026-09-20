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

"""Helper module to construct a CartesianAdmittance action."""

import dataclasses

import numpy as np

from intrinsic.icon.actions import cartesian_admittance_action_pb2
from intrinsic.icon.actions import cartesian_impedance_pb2
from intrinsic.icon.actions import cartesian_impedance_reference_generators_pb2 as reference_generators_pb2
from intrinsic.icon.proto import cart_space_pb2
from intrinsic.icon.proto import matrix_conversions
from intrinsic.icon.python import actions
from intrinsic.kinematics.types import joint_limits_pb2
from intrinsic.manipulation.skills.force import utils
from intrinsic.math.python import data_types

ACTION_TYPE_NAME = "intrinsic.cartesian_admittance_action"

_NUM_TRANS_DOF = 3


@dataclasses.dataclass(frozen=True)
class StateVariables:
  SENSED_FORCE = "intrinsic.sensed_force"
  SETTLED_FOR_SECONDS = "intrinsic.settled_for_seconds"
  IS_SETTLED = "intrinsic.is_settled"
  ELAPSED_TIME_SECONDS = "elapsed_time_seconds"
  DISTANCE_TRAVELED = "intrinsic.translational_distance_traveled"
  MAX_DISPLACEMENT_IN_WINDOW = (
      "maximum_translational_displacement_in_time_window"
  )


def _twist_to_proto(twist: data_types.Twist) -> cart_space_pb2.Twist:
  return cart_space_pb2.Twist(
      x=twist.linear[0],
      y=twist.linear[1],
      z=twist.linear[2],
      rx=twist.angular[0],
      ry=twist.angular[1],
      rz=twist.angular[2],
  )


def _default_reference_generators(
    lowpass_filter_constant: float | None = None,
) -> reference_generators_pb2.ReferenceGenerators:
  """Returns default reference generators for both cartesian and nullspace."""
  cartesian_generator = (
      reference_generators_pb2.SimpleImpedanceReferenceGenerator()
  )
  nullspace_generator = (
      reference_generators_pb2.SimpleNullspaceReferenceGenerator()
  )
  if lowpass_filter_constant:
    cartesian_generator.lowpass_filter_constant = lowpass_filter_constant
    nullspace_generator.lowpass_filter_constant = lowpass_filter_constant
  return reference_generators_pb2.ReferenceGenerators(
      cartesian=reference_generators_pb2.ReferenceGenerators.Cartesian(
          simple_lowpass=cartesian_generator
      ),
      nullspace=reference_generators_pb2.ReferenceGenerators.Nullspace(
          simple_lowpass=nullspace_generator
      ),
  )


def create_cartesian_admittance_action(
    action_id: int,
    joint_position_part_name: str,
    ft_sensor_part_name: str,
    *,
    virtual_cartesian_inertia: np.ndarray,
    cartesian_stiffness: np.ndarray,
    cartesian_damping: np.ndarray | None = None,
    robot_tip_t_robot_tool: data_types.Pose3 | None = None,
    task_t_tool_reference_pose: data_types.Pose3 | None = None,
    tool_reference_twist: data_types.Twist | None = None,
    tool_reference_wrench: data_types.Wrench | None = None,
    joint_limits: joint_limits_pb2.JointLimits | None = None,
    cartesian_limits: cart_space_pb2.CartesianLimits | None = None,
    sensed_wrench_deadband: data_types.Wrench | None = None,
    state_variable_config: (
        cartesian_impedance_pb2.StateVariableConfiguration | None
    ) = None,
    lowpass_filter_constant: float | None = None,
) -> actions.Action:
  """Creates a CartesianAdmittance action.

  This function only sets the most relevant parameters of the action. To set
  advanced parameters directly manipulate the returned action.proto.

  Args:
    action_id: The ID of the action.
    joint_position_part_name:  The name of the part providing the JointPosition
      interface.
    ft_sensor_part_name: The name of the part providing the ForceTorqueSensor
      interface,
    virtual_cartesian_inertia: The desired virtual cartesian inertia matrix,
      expressed in the robot base frame.
    cartesian_stiffness: The cartesian stiffness matrix, expressed in the robot
      base frame.
    cartesian_damping: The cartesian damping matrix, expressed in the robot base
      frame. If not specified, it is automatically derived from stiffness and
      inertia to achieve critical damping.
    robot_tip_t_robot_tool: Offset pose from default robot tip, expressed w.r.t.
      the default tip frame. This allows to define an offset 'target' for the
      Cartesian impedance control law, which does not coincide with the default
      robot tip. Defaults to Identity, which is equivalent to no offset.
    task_t_tool_reference_pose: The desired goal pose of the tool frame (e.g. an
      endeffector), expressed w.r.t. the task frame. If no pose is specified,
      the controller uses the current Cartesian pose of the 'tool' frame
      computed via forward kinematics. This also takes into account above
      'robot_tip_t_robot_tool'.
    tool_reference_twist: The desired goal twist of the target frame (e.g. and
      endffector), expressed w.r.t. the robot base frame. Defaults to zero.
    tool_reference_wrench: The desired wrench at the endeffector, expressed in
      the robot base frame. Defaults to zero.
    joint_limits: The joint limits to apply to the motion.
    cartesian_limits: Cartesian limits for the motion. Action will fail if a
      motion target violates Cartesian limits.
    sensed_wrench_deadband: Deadband to be applied to the sensed wrench. Values
      below the numerical threshold defined in the deadband will be truncated.
    state_variable_config: The configuration of the actions state variables. If
      no proto is provided the configuration is set to the default values. Look
      at the proto for more information on this.
    lowpass_filter_constant: Low-pass filter constant for a simple cartesian
      impedance reference generator, defaults to 0.005, must be within (0.0,
      1.0], where 1.0 corresponds to no filtering, and a small value like 1e-4
      corresponds to very strong filtering.

  Returns:
    The CartesianAdmittance action.
  """
  params = cartesian_impedance_pb2.CartesianImpedanceParameters()

  params.cartesian_target.virtual_cartesian_inertia.CopyFrom(
      matrix_conversions.from_ndarray(virtual_cartesian_inertia)
  )

  params.cartesian_target.cartesian_stiffness.CopyFrom(
      matrix_conversions.from_ndarray(cartesian_stiffness)
  )

  if cartesian_damping is not None:
    params.cartesian_target.cartesian_damping.CopyFrom(
        matrix_conversions.from_ndarray(cartesian_damping)
    )

  if robot_tip_t_robot_tool is not None:
    params.cartesian_target.robot_tip_t_robot_tool.CopyFrom(
        data_types.pose3_to_transform(robot_tip_t_robot_tool)
    )

  if task_t_tool_reference_pose is not None:
    params.cartesian_target.task_t_tool_reference_pose.CopyFrom(
        data_types.pose3_to_transform(task_t_tool_reference_pose)
    )

  if tool_reference_twist is not None:
    params.cartesian_target.tool_reference_twist.CopyFrom(
        _twist_to_proto(tool_reference_twist)
    )

  if tool_reference_wrench is not None:
    params.cartesian_target.tool_reference_wrench.CopyFrom(
        data_types.wrench_to_proto(tool_reference_wrench)
    )

  if joint_limits is not None:
    params.constraints.joint_limits.CopyFrom(joint_limits)

  if cartesian_limits is not None:
    params.constraints.cartesian_limits.CopyFrom(cartesian_limits)

  if sensed_wrench_deadband is not None:
    params.algorithm_configuration.sensed_wrench_deadband.CopyFrom(
        data_types.wrench_to_proto(sensed_wrench_deadband)
    )

  if state_variable_config is not None:
    params.state_variable_configuration.CopyFrom(state_variable_config)

  params.reference_generators.CopyFrom(
      _default_reference_generators(lowpass_filter_constant)
  )

  return actions.Action(
      action_id,
      ACTION_TYPE_NAME,
      {"arm": joint_position_part_name, "ft_sensor": ft_sensor_part_name},
      params=(
          cartesian_admittance_action_pb2.CartesianAdmittanceActionFixedParams(
              cartesian_impedance_parameters=params
          )
      ),
  )


def _create_approach_contact_action(
    action_id: int,
    joint_position_part_name: str,
    ft_sensor_part_name: str,
    virtual_cartesian_inertia: np.ndarray,
    contact_force: float,
    motion_direction_in_base: np.ndarray,
    mechanical_stiffness_at_contact: float,
    tool_translational_stiffness_in_nullspace: float,
    tool_rotational_stiffness_in_nullspace: float,
    robot_tip_t_robot_tool: data_types.Pose3 | None = None,
    joint_limits: joint_limits_pb2.JointLimits | None = None,
    cartesian_limits: cart_space_pb2.CartesianLimits | None = None,
    sensed_wrench_deadband: data_types.Wrench | None = None,
    state_variable_config: (
        cartesian_impedance_pb2.StateVariableConfiguration | None
    ) = None,
    lowpass_filter_constant: float | None = None,
) -> actions.Action:
  """Creates a CartesianAdmittance action for an approach contact motion."""
  directional_stiffness = utils.compute_directional_stiffness(
      translational_motion_direction=motion_direction_in_base,
      translational_stiffness_orthogonal_to_motion_direction=tool_translational_stiffness_in_nullspace,
      rotational_stiffness_orthogonal_to_motion_direction=tool_rotational_stiffness_in_nullspace,
  )

  damping = utils.compute_overdamping(
      virtual_cartesian_inertia, mechanical_stiffness_at_contact
  )

  contact_force_vec = motion_direction_in_base * contact_force

  if (
      cartesian_limits is not None
      and cartesian_limits.max_translational_velocity
  ):
    velocity_limit = max(cartesian_limits.max_translational_velocity)

    approach_velocity = np.dot(
        np.linalg.inv(damping[:_NUM_TRANS_DOF, :_NUM_TRANS_DOF]),
        contact_force_vec,
    )
    approach_velocity_norm = np.linalg.norm(approach_velocity)
    if approach_velocity_norm > velocity_limit:
      raise ValueError(
          "The defined force can result in a velocity of {:.3f} m/s, but the"
          " max_translational_velocity is {:.3f} m/s.".format(
              approach_velocity_norm, velocity_limit
          ),
      )

  return create_cartesian_admittance_action(
      action_id=action_id,
      joint_position_part_name=joint_position_part_name,
      ft_sensor_part_name=ft_sensor_part_name,
      virtual_cartesian_inertia=virtual_cartesian_inertia,
      cartesian_stiffness=directional_stiffness,
      cartesian_damping=damping,
      robot_tip_t_robot_tool=robot_tip_t_robot_tool,
      joint_limits=joint_limits,
      tool_reference_wrench=data_types.Wrench(force=contact_force_vec),
      sensed_wrench_deadband=sensed_wrench_deadband,
      cartesian_limits=cartesian_limits,
      state_variable_config=state_variable_config,
      lowpass_filter_constant=lowpass_filter_constant,
  )


def create_approach_contact_action(
    action_id: int,
    joint_position_part_name: str,
    ft_sensor_part_name: str,
    virtual_cartesian_inertia: np.ndarray,
    contact_force: float,
    motion_direction_in_base: np.ndarray,
    mechanical_stiffness_at_contact: float,
    tool_translational_stiffness_in_nullspace: float,
    tool_rotational_stiffness_in_nullspace: float,
    robot_tip_t_robot_tool: data_types.Pose3 | None = None,
    joint_limits: joint_limits_pb2.JointLimits | None = None,
    cartesian_limits: cart_space_pb2.CartesianLimits | None = None,
    sensed_wrench_deadband: data_types.Wrench | None = None,
    state_variable_config: (
        cartesian_impedance_pb2.StateVariableConfiguration | None
    ) = None,
) -> actions.Action:
  """Creates a CartesianAdmittance action for an approach contact motion.

  The returned CartesianAdmittance action is parametrized to move the robot into
  contact, controlled by the contact_force as reference wrench.

  Args:
    action_id: The ID of the action.
    joint_position_part_name:  The name of the part providing the JointPosition
      interface.
    ft_sensor_part_name: The name of the part providing the ForceTorqueSensor
      interface,
    virtual_cartesian_inertia: The desired virtual cartesian inertia matrix,
      expressed in the robot base frame.
    contact_force: The desired contact force between the robot tool and the
      surface. Points in motion_direction_in_base. Unit is N.
    motion_direction_in_base: The robot tip moves in this direction to make
      contact.
    mechanical_stiffness_at_contact: Mechanical stiffness (in Netwons per meter)
      of robot and environment when system in contact.
    tool_translational_stiffness_in_nullspace: Translational stiffness (in
      `N/m`) orthogonal to the motion direction.
    tool_rotational_stiffness_in_nullspace: Rotational stiffness (in `N m/rad`)
      orthogonal to the motion direction.
    robot_tip_t_robot_tool: Offset pose from default robot tip, expressed w.r.t.
      the default tip frame. This allows to define an offset 'target' for the
      Cartesian impedance control law, which does not coincide with the default
      robot tip. Defaults to Identity, which is equivalent to no offset.
    joint_limits: The joint limits to apply to the motion.
    cartesian_limits: Cartesian limits for the motion. Action will fail if a
      motion target violates Cartesian limits.
    sensed_wrench_deadband: Deadband to be applied to the sensed wrench. Values
      below the numerical threshold defined in the deadband will be truncated.
    state_variable_config: The configuration of the actions state variables. If
      no proto is provided the configuration is set to the default values. Look
      at the proto for more information on this.

  Returns:
    A CartesianAdmittance action.
  """
  return _create_approach_contact_action(
      action_id,
      joint_position_part_name,
      ft_sensor_part_name,
      virtual_cartesian_inertia,
      contact_force,
      motion_direction_in_base,
      mechanical_stiffness_at_contact,
      tool_translational_stiffness_in_nullspace,
      tool_rotational_stiffness_in_nullspace,
      robot_tip_t_robot_tool,
      joint_limits,
      cartesian_limits,
      sensed_wrench_deadband,
      state_variable_config,
      lowpass_filter_constant=None,
  )


def create_stabilize_contact_action(
    action_id: int,
    joint_position_part_name: str,
    ft_sensor_part_name: str,
    virtual_cartesian_inertia: np.ndarray,
    contact_force: float,
    motion_direction_in_base: np.ndarray,
    mechanical_stiffness_at_contact: float,
    tool_translational_stiffness_in_nullspace: float,
    tool_rotational_stiffness_in_nullspace: float,
    robot_tip_t_robot_tool: data_types.Pose3 | None = None,
    joint_limits: joint_limits_pb2.JointLimits | None = None,
    cartesian_limits: cart_space_pb2.CartesianLimits | None = None,
    sensed_wrench_deadband: data_types.Wrench | None = None,
    state_variable_config: (
        cartesian_impedance_pb2.StateVariableConfiguration | None
    ) = None,
) -> actions.Action:
  """Creates a CartesianAdmittance action to stabilize a contact.

  The returned CartesianAdmittance action is parametrized to stabilize a robot
  which is already in contact.

  Args:
    action_id: The ID of the action.
    joint_position_part_name:  The name of the part providing the JointPosition
      interface.
    ft_sensor_part_name: The name of the part providing the ForceTorqueSensor
      interface,
    virtual_cartesian_inertia: The desired virtual cartesian inertia matrix,
      expressed in the robot base frame.
    contact_force: The desired contact force between the robot tool and the
      surface. Points in motion_direction_in_base. Unit is N.
    motion_direction_in_base: The robot tip moves in this direction to make
      contact.
    mechanical_stiffness_at_contact: Mechanical stiffness (in Netwons per meter)
      of robot and environment when system in contact.
    tool_translational_stiffness_in_nullspace: Translational stiffness (in
      `N/m`) orthogonal to the motion direction.
    tool_rotational_stiffness_in_nullspace: Rotational stiffness (in `N m/rad`)
      orthogonal to the motion direction.
    robot_tip_t_robot_tool: Offset pose from default robot tip, expressed w.r.t.
      the default tip frame. This allows to define an offset 'target' for the
      Cartesian impedance control law, which does not coincide with the default
      robot tip. Defaults to Identity, which is equivalent to no offset.
    joint_limits: The joint limits to apply to the motion.
    cartesian_limits: Cartesian limits for the motion. Action will fail if a
      motion target violates Cartesian limits.
    sensed_wrench_deadband: Deadband to be applied to the sensed wrench. Values
      below the numerical threshold defined in the deadband will be truncated.
    state_variable_config: The configuration of the actions state variables. If
      no proto is provided the configuration is set to the default values. Look
      at the proto for more information on this.

  Returns:
    A CartesianAdmittance action.
  """
  return _create_approach_contact_action(
      action_id,
      joint_position_part_name,
      ft_sensor_part_name,
      virtual_cartesian_inertia,
      contact_force,
      motion_direction_in_base,
      mechanical_stiffness_at_contact,
      tool_translational_stiffness_in_nullspace,
      tool_rotational_stiffness_in_nullspace,
      robot_tip_t_robot_tool,
      joint_limits,
      cartesian_limits,
      sensed_wrench_deadband,
      state_variable_config,
      lowpass_filter_constant=1.0,
  )
