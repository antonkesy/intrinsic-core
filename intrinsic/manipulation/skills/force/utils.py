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

"""force_skill_utils contains functions for creating force control related skills."""

import dataclasses

import numpy as np

from intrinsic.icon.control.algorithms.python import directional_stiffness
from intrinsic.icon.control.algorithms.python import overdamping
from intrinsic.icon.equipment import force_control_settings_pb2
from intrinsic.icon.equipment import icon_equipment_pb2
from intrinsic.icon.proto import cart_space_pb2
from intrinsic.math.python import data_types
from intrinsic.math.python import proto_conversion
from intrinsic.resources.proto import resource_handle_pb2

_TRANSLATIONAL_DOF = 3
_ROTATIONAL_DOF = 3

_FT_SENSOR_KEY = "Icon2ForceTorqueSensorPart"


def compute_directional_stiffness(
    translational_motion_direction: np.ndarray,
    translational_stiffness_orthogonal_to_motion_direction: float,
    rotational_stiffness_orthogonal_to_motion_direction: float,
) -> np.ndarray:
  """Computes stiffness matrix based on a given motion direction.

  Stiffness is set by:
  - 0 along the translational motion direction, such that the robot can move
  freely along this direction or its opposite direction, without being blocked
  by the stiffness.
  - translational_stiffness_orthogonal_to_motion_direction orthogonal to
  translational translational motion direction
  - rotational_stiffness_orthogonal_to_motion_direction along all rotational
  directions.

  Args:
    translational_motion_direction: A (3,) vector describing the translational
      direction of motion.
    translational_stiffness_orthogonal_to_motion_direction: Virtual
      translational stiffness of target orthogonal to translational motion
      direction.
    rotational_stiffness_orthogonal_to_motion_direction: Virtual rotational
      stiffness of target orthogonal to motion direction.

  Returns:
    A 6x6 stiffness matrix which is expressed in the same frame as where
    `translational_directions` vectors are defined.
  """
  return directional_stiffness.compute_directional_stiffness(
      translational_motion_direction,
      translational_stiffness_orthogonal_to_motion_direction,
      rotational_stiffness_orthogonal_to_motion_direction,
  )


def compute_overdamping(
    inertia: np.ndarray, stiffness_compliance_device: float
) -> np.ndarray:
  """Computes damping matrix following the Surdilovic overdamping criterium.

  Args:
    inertia: A (6,6) inertia matrix.
    stiffness_compliance_device: Stiffness of mechanical elements in contact.

  Returns:
    A (6,6) damping matrix.
  """
  return overdamping.compute_overdamping(inertia, stiffness_compliance_device)


def create_cartesian_limits(
    max_translational_velocity: float | None = None,
    max_translational_acceleration: float | None = None,
    max_translational_jerk: float | None = None,
    max_rotational_velocity: float | None = None,
    max_rotational_acceleration: float | None = None,
    max_rotational_jerk: float | None = None,
) -> cart_space_pb2.CartesianLimits:
  """Gets the default cartesian_limits.

  Args:
    max_translational_velocity: The maximal velocity in xyz direction.
    max_translational_acceleration: The maximal acceleration in xyz direction.
    max_translational_jerk: The maximal jerk in xyz direction.
    max_rotational_velocity: The maximal rotaional joint velocity.
    max_rotational_acceleration: The maximal rotational joint acceleration.
    max_rotational_jerk: the maximal rotational joint jerk.

  Returns:
    The cartesian_limits with a box as position limits.
  """

  limits = cart_space_pb2.CartesianLimits()

  if max_translational_velocity is not None:
    limits.min_translational_velocity[:] = [
        -max_translational_velocity
    ] * _TRANSLATIONAL_DOF

    limits.max_translational_velocity[:] = [
        max_translational_velocity
    ] * _TRANSLATIONAL_DOF

  if max_translational_acceleration is not None:
    limits.min_translational_acceleration[:] = [
        -max_translational_acceleration
    ] * _TRANSLATIONAL_DOF
    limits.max_translational_acceleration[:] = [
        max_translational_acceleration
    ] * _TRANSLATIONAL_DOF

  if max_translational_jerk is not None:
    limits.min_translational_jerk[:] = [
        -max_translational_jerk
    ] * _TRANSLATIONAL_DOF

    limits.max_translational_jerk[:] = [
        max_translational_jerk
    ] * _TRANSLATIONAL_DOF

  if max_rotational_velocity is not None:
    limits.max_rotational_velocity = max_rotational_velocity

  if max_rotational_acceleration is not None:
    limits.max_rotational_acceleration = max_rotational_acceleration

  if max_rotational_jerk is not None:
    limits.max_rotational_jerk = max_rotational_jerk

  return limits


def compute_projected_wrench_deadband_norm(
    sensed_wrench_deadband: data_types.Wrench,
    motion_direction_in_base: np.ndarray,
) -> float:
  """Compute the norm of the deadband projected on the motion direction.

  The sensed wrench deadband is projected onto the motion direction and its norm
  is returned.

  Args:
    sensed_wrench_deadband: The deadband of the force torque sensor.
    motion_direction_in_base: Motion direction of the TCP in robot base frame.

  Returns:
    The projected_wrench_deadband_norm.
  """

  if motion_direction_in_base.shape != (3,):
    raise ValueError(
        f"motion_direction_in_base has shape {motion_direction_in_base.shape},"
        " should be [x, y, z]."
    )

  if np.linalg.norm(motion_direction_in_base) == 0.0:
    raise ValueError("motion_direction_in_base cannot be the zero vector.")

  motion_direction_normalised = (
      1.0 / np.linalg.norm(motion_direction_in_base) * motion_direction_in_base
  )

  projected_wrench_deadband = np.dot(
      np.diag(sensed_wrench_deadband.force),
      motion_direction_normalised.transpose(),
  )
  return np.linalg.norm(projected_wrench_deadband)


def merge_cartesian_limits(
    limits: cart_space_pb2.CartesianLimits,
    other_limits: cart_space_pb2.CartesianLimits,
) -> None:
  """Overwrites fields set in `other_limits` in `limits`."""
  if other_limits.min_translational_position:
    limits.min_translational_position[:] = (
        other_limits.min_translational_position
    )
  if other_limits.max_translational_position:
    limits.max_translational_position[:] = (
        other_limits.max_translational_position
    )

  if other_limits.min_translational_velocity:
    limits.min_translational_velocity[:] = (
        other_limits.min_translational_velocity
    )
  if other_limits.max_translational_velocity:
    limits.max_translational_velocity[:] = (
        other_limits.max_translational_velocity
    )
  if other_limits.min_translational_acceleration:
    limits.min_translational_acceleration[:] = (
        other_limits.min_translational_acceleration
    )
  if other_limits.max_translational_acceleration:
    limits.max_translational_acceleration[:] = (
        other_limits.max_translational_acceleration
    )
  if other_limits.min_translational_jerk:
    limits.min_translational_jerk[:] = other_limits.min_translational_jerk
  if other_limits.max_translational_jerk:
    limits.max_translational_jerk[:] = other_limits.max_translational_jerk
  if other_limits.max_rotational_velocity:
    limits.max_rotational_velocity = other_limits.max_rotational_velocity
  if other_limits.max_rotational_acceleration:
    limits.max_rotational_acceleration = (
        other_limits.max_rotational_acceleration
    )
  if other_limits.max_rotational_jerk:
    limits.max_rotational_jerk = other_limits.max_rotational_jerk


@dataclasses.dataclass(frozen=True)
class ForceControlParams:
  """Wrapper for ImpedanceControlEquipmentSettings proto."""

  sensed_wrench_deadband: data_types.Wrench
  excessive_force_threshold: float
  excessive_torque_threshold: float
  virtual_translational_inertia: float
  virtual_rotational_inertia: float

  @property
  def virtual_inertia(self) -> np.ndarray:
    return np.diag(
        [self.virtual_translational_inertia] * _TRANSLATIONAL_DOF
        + [self.virtual_rotational_inertia] * _ROTATIONAL_DOF
    )

  @classmethod
  def _from_proto(
      cls,
      proto: force_control_settings_pb2.ForceControlSettings,
  ) -> "ForceControlParams":
    return ForceControlParams(
        sensed_wrench_deadband=proto_conversion.wrench_from_proto(
            proto.sensed_wrench_deadband
        ),
        excessive_force_threshold=proto.excessive_force_threshold,
        excessive_torque_threshold=proto.excessive_torque_threshold,
        virtual_translational_inertia=proto.virtual_translational_inertia,
        virtual_rotational_inertia=proto.virtual_rotational_inertia,
    )

  def proto(self) -> force_control_settings_pb2.ForceControlSettings:
    return force_control_settings_pb2.ForceControlSettings(
        sensed_wrench_deadband=data_types.wrench_to_proto(
            self.sensed_wrench_deadband
        ),
        excessive_force_threshold=self.excessive_force_threshold,
        excessive_torque_threshold=self.excessive_torque_threshold,
        virtual_translational_inertia=self.virtual_translational_inertia,
        virtual_rotational_inertia=self.virtual_rotational_inertia,
    )

  @classmethod
  def from_resource_handle(
      cls,
      resource_handle: resource_handle_pb2.ResourceHandle,
  ) -> "ForceControlParams":
    """Creates ForceControlParams from resource_handle.

    Loads the force control params from the ft-sensor-part config.

    Args:
      resource_handle: The resource handle. Has to contain a ft-sensor-part with
        force_control_settings.

    Returns:
      The ForceControlParams
    """
    if _FT_SENSOR_KEY in resource_handle.resource_data:
      part_config = icon_equipment_pb2.Icon2ForceTorqueSensorPart()
      resource_handle.resource_data[_FT_SENSOR_KEY].contents.Unpack(part_config)
      if part_config.HasField("force_control_settings"):
        return ForceControlParams._from_proto(
            part_config.force_control_settings
        )

    raise ValueError("Force control settings not found in resource_data.")
