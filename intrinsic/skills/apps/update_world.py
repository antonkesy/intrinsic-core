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

"""Skill wrapper to perform an object world update.

This skills takes an ObjectWorldUpdate as input and applies the
specified operation to the specified world object or frame in the world.
This allows the use of any functionality from ObjectWorldUpdate via a skill
which enables its use in behavior trees.
"""

from intrinsic.skills.apps import update_world_pb2
from intrinsic.skills.proto import footprint_pb2
from intrinsic.skills.python import skill_interface as skl
from intrinsic.util.decorators import overrides
from intrinsic.world.proto import object_world_refs_pb2
from intrinsic.world.proto import object_world_service_pb2
from intrinsic.world.proto import object_world_updates_pb2
from intrinsic.world.python import object_world_client
from intrinsic.world.python import object_world_resources


class UpdateWorld(skl.Skill):
  """Updates the world based on a provided ObjectWorldUpdate request.

  The input ObjectWorldUpdate can be any of the update messages supported by the
  ObjectWorldUpdate. Note that if the message passed to ObjectWorldUpdate
  contains the field `world_id`, this should not be set.

  For example, to reparent an object via the UpdateWorld skill, the following
  can be used in Python:

  skills.update_world(
      update=skills.update_world.ObjectWorldUpdate(
          reparent_object=skills.update_world.ReparentObjectRequest(
              # world_id - DO NOT SET THIS, see comment above.
              object=world.some_object,
              parent_object=skills.update_world.ObjectReferenceWithEntityFilter(
                  reference=world.new_parent_object,
                  entity_filter=skills.update_world.ObjectEntityFilter(
                      include_base_entity=True,
                  )
              )
          )
      )
  )

  Another example: The following updates the pose transform of an object in the
  world via the UpdateWorld skill:

  executive.run(
      skills.update_world(
          update=skills.update_world.ObjectWorldUpdate(
              update_transform=skills.update_world.UpdateTransformRequest(
                  node_a=world.root,
                  node_b=world.my_object,
                  a_t_b=data_types.Pose3(),
                  node_to_update=world.my_object,
              )
          )
      )
  )
  """

  @overrides(skl.Skill)
  def get_footprint(
      self,
      request: skl.GetFootprintRequest[update_world_pb2.UpdateWorldParams],
      context: skl.GetFootprintContext,
  ) -> footprint_pb2.Footprint:
    """Returns a the footprint that locks all or part of the world."""

    user_updates = _resolve_updates(request.params)

    try:
      footprint = footprint_pb2.Footprint(lock_the_universe=False)
      for update in user_updates.updates:
        footprint.object_reservation.extend(
            _get_footprint_resources_for_update(context.object_world, update)
        )

      return footprint

    except (TypeError, NotImplementedError):
      # Return the default below.
      pass

    # The object world updates contain fields for which resource consumption
    # has not been defined. Locking the universe is an overly conservative
    # option that will suffice.
    return footprint_pb2.Footprint(lock_the_universe=True)

  @overrides(skl.Skill)
  def execute(
      self,
      request: skl.ExecuteRequest[update_world_pb2.UpdateWorldParams],
      context: skl.ExecuteContext,
  ) -> None:
    """Executes the update_world skill.

    Executes the functionality from the ObjectWorldUpdate provided in the
    parameters of the request.

    Args:
      request: The execute request.
      context: Provides access to the world and other services that a skill may
        use.

    Raises:
      TypeError: if parameters were not provided correctly.
    """
    world_client = context.object_world
    world_client_stub = world_client.stub

    request = object_world_service_pb2.UpdateWorldResourcesRequest(
        world_id=world_client.world_id,
        world_updates=_resolve_updates(request.params),
    )

    world_client_stub.UpdateWorldResources(request)

    return None

  @overrides(skl.Skill)
  def preview(
      self,
      request: skl.PreviewRequest[update_world_pb2.UpdateWorldParams],
      context: skl.PreviewContext,
  ) -> None:
    updates = _resolve_updates(request.params)
    for update in updates.updates:
      context.record_world_update(update=update, elapsed=0, duration=0)


def _object_reference_to_reservation(
    world: object_world_client.ObjectWorldClient,
    object_ref: object_world_refs_pb2.ObjectReference,
) -> footprint_pb2.ObjectWorldReservation:
  """Returns a resource for the given object."""
  world_node = world.get_object(object_ref)

  if not world_node.proto.name_is_global_alias:
    raise TypeError("WorldObject is not unique so skipping resource")

  return footprint_pb2.ObjectWorldReservation(
      object=object_world_refs_pb2.ObjectReferenceByName(
          object_name=world_node.name
      )
  )


def _transform_node_reference_to_reservation(
    world: object_world_client.ObjectWorldClient,
    transform_node: object_world_refs_pb2.TransformNodeReference,
) -> footprint_pb2.ObjectWorldReservation:
  """Returns a resource for the given transform node."""
  world_node = world.get_transform_node(transform_node)

  if isinstance(world_node, object_world_resources.WorldObject):
    if not world_node.proto.name_is_global_alias:
      raise TypeError("WorldObject is not unique so skipping resource")

    return footprint_pb2.ObjectWorldReservation(
        object=object_world_refs_pb2.ObjectReferenceByName(
            object_name=world_node.name
        )
    )
  elif isinstance(world_node, object_world_resources.Frame):
    # TODO(stoyang): Frame parent object may not be uniquely identifiable
    return footprint_pb2.ObjectWorldReservation(
        frame=object_world_refs_pb2.FrameReferenceByName(
            object_name=world_node.object_name, frame_name=world_node.name
        )
    )
  else:
    raise TypeError("Unknown TransformNode type")


def _resolve_updates(
    params: update_world_pb2.UpdateWorldParams,
) -> object_world_updates_pb2.ObjectWorldUpdates:
  """Unpacks the update input."""
  if params.HasField("updates"):
    return params.updates

  return object_world_updates_pb2.ObjectWorldUpdates(updates=[params.update])


def _get_footprint_resources_for_update(
    world_client,
    update: object_world_updates_pb2.ObjectWorldUpdate,
) -> list[footprint_pb2.ObjectWorldReservation]:
  """Identifies a list of resources needed for the given update."""
  if update.HasField("update_transform"):
    return [
        _transform_node_reference_to_reservation(
            world_client,
            update.update_transform.node_a,
        ),
        _transform_node_reference_to_reservation(
            world_client,
            update.update_transform.node_b,
        ),
        _transform_node_reference_to_reservation(
            world_client,
            update.update_transform.node_to_update,
        ),
    ]

  elif update.HasField("reparent_object"):
    reparent_req = update.reparent_object
    reservations = [
        _object_reference_to_reservation(
            world_client,
            reparent_req.object,
        )
    ]

    if reparent_req.HasField("parent_object"):
      reservations.append(
          _object_reference_to_reservation(
              world_client,
              reparent_req.parent_object.reference,
          )
      )
    elif reparent_req.HasField("parent_frame"):
      # Ignore: frames don't have any volume.
      pass
    elif reparent_req.HasField("new_parent"):
      reservations.append(
          _object_reference_to_reservation(
              world_client,
              reparent_req.new_parent.reference,
          )
      )

    return reservations

  raise NotImplementedError("Cannot define resources for given update.")
