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

"""A skill that controls a pinch gripper."""

from absl import logging
from google.protobuf.internal import containers
import grpc

from intrinsic.assets import interface_utils
from intrinsic.assets.dependencies import utils as asset_utils
from intrinsic.hardware.gripper.eoat import eoat_service_pb2
from intrinsic.hardware.gripper.eoat import eoat_service_pb2_grpc
from intrinsic.manipulation.skills import control_pinch_gripper_error_codes as error_codes
from intrinsic.manipulation.skills import control_pinch_gripper_pb2
from intrinsic.skills.proto import footprint_pb2
from intrinsic.skills.python import skill_interface as skl
from intrinsic.world.proto import object_world_refs_pb2
from intrinsic.world.proto import object_world_updates_pb2
from intrinsic.world.python import object_world_resources

_PREDICTED_MOVEMENT_DURATION_SECS: float = 1.0

_EQUIPMENT_SLOT: str = "pinch_gripper"

PinchGripperCommands = control_pinch_gripper_pb2.PinchGripperCommand


def _pinch_gripper_interface_uri() -> str:
  return (
      f"{interface_utils.GRPC_URI_PREFIX}"
      f"{eoat_service_pb2.DESCRIPTOR.services_by_name['PinchGripper'].full_name}"
  )


class ControlPinchGripper(skl.Skill):
  """A skill that controls a pinch gripper."""

  def execute(
      self,
      request: skl.ExecuteRequest[PinchGripperCommands],
      context: skl.ExecuteContext,
  ) -> None:
    """Executes a given pinch gripper command and updates the world."""
    world = context.object_world
    gripper_object = world.get_kinematic_object(
        request.params.pinch_gripper.object.name
    )

    channel = asset_utils.connect(
        request.params.pinch_gripper,
        _pinch_gripper_interface_uri(),
    )
    stub = eoat_service_pb2_grpc.PinchGripperStub(channel)

    command, expected_position = self._parse_params(
        request.params, gripper_object
    )
    try:
      if command == "grasp":
        logging.info("pinch_gripper.grasp:\n%s", request.params.grasp)
        stub.Grasp(request.params.grasp)
      elif command == "release":
        logging.info("pinch_gripper.release:\n%s", request.params.release)
        stub.Release(request.params.release)
      else:
        raise skl.SkillError(
            error_codes.PARAMETERIZATION_ERROR_CODE,
            f"Command is not supported: {request.params}!",
        )
    except grpc.RpcError as e:
      details = (
          e.details() if hasattr(e, "details") and e.details() else None
      ) or str(e)
      raise skl.SkillError(
          error_codes.GRIPPER_SERVICE_ERROR_CODE,
          f"Pinch gripper service RPC failed: {details}",
          debug_message=str(e),
      ) from e

    logging.info("Move gripper joints to %s", expected_position)

    world.update_joint_positions(
        gripper_object, joint_positions=expected_position
    )

  def get_footprint(
      self,
      request: skl.GetFootprintRequest[PinchGripperCommands],
      context: skl.GetFootprintContext,
  ) -> footprint_pb2.Footprint:
    """Returns the footprint of this skill."""
    return footprint_pb2.Footprint(
        object_reservation=[
            footprint_pb2.ObjectWorldReservation(
                type=footprint_pb2.ObjectWorldReservation.WRITE,
                object=object_world_refs_pb2.ObjectReferenceByName(
                    object_name=request.params.pinch_gripper.object.name
                ),
            )
        ],
        lock_the_universe=False,
    )

  def preview(
      self,
      request: skl.PreviewRequest[PinchGripperCommands],
      context: skl.PreviewContext,
  ) -> None:
    gripper_object = context.object_world.get_kinematic_object(
        request.params.pinch_gripper.object.name
    )

    positions = gripper_object.joint_positions
    logging.info("Found gripper with joint positions: %s", positions)

    _, expected_position = self._parse_params(request.params, gripper_object)

    world_update = object_world_updates_pb2.ObjectWorldUpdate(
        update_object_joints=(
            object_world_updates_pb2.UpdateObjectJointsRequest(
                object=gripper_object.reference,
                joint_positions=expected_position,
                joint_names=gripper_object.joint_entity_names,
            )
        )
    )
    context.record_world_update(
        update=world_update,
        elapsed=0.0,
        duration=_PREDICTED_MOVEMENT_DURATION_SECS,
    )

  def _parse_params(
      self,
      params: PinchGripperCommands,
      gripper_object: object_world_resources.KinematicObject,
  ) -> tuple[str, containers.RepeatedScalarFieldContainer]:
    """Determine the expected joint position given a command."""
    if params.WhichOneof("command") == "grasp":
      return (
          "grasp",
          gripper_object.joint_application_limits.min_position.values,
      )
    elif params.WhichOneof("command") == "release":
      return (
          "release",
          gripper_object.joint_application_limits.max_position.values,
      )
    else:
      raise skl.SkillError(
          error_codes.PARAMETERIZATION_ERROR_CODE,
          f"Command is not supported: {params}!",
      )
