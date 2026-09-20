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

"""Provides functionality to interact with a running simulation.

Deprecated. See migration note in `reset()` method below.
TODO(b/489493047): Clean up after deprecation period.

Typical usage example:
  from intrinsic.executive.jupyter.workcell import intrinsic

  workcell = intrinsic.connect()
  simulation = workcell.simulation

  simulation.reset()
"""

import warnings

import grpc

from intrinsic.math.python import data_types  # pylint: disable=line-too-long 
from intrinsic.math.python import proto_conversion
from intrinsic.simulation.service.proto.v1 import simulation_service_pb2
from intrinsic.simulation.service.proto.v1 import simulation_service_pb2_grpc
from intrinsic.solutions import errors
from intrinsic.util.grpc import error_handling
from intrinsic.world.proto import object_world_service_pb2
from intrinsic.world.proto import object_world_service_pb2_grpc
from intrinsic.world.python import object_world_resources  # pylint: disable=line-too-long 

SimulationServiceStub = simulation_service_pb2_grpc.SimulationServiceStub
ObjectWorldServiceStub = object_world_service_pb2_grpc.ObjectWorldServiceStub

_SIM_WORLD_ID = 'sim_world'  


class Simulation:
  """Provides commands to interact with a running simulation."""

  def __init__(
      self,
      simulation_service: SimulationServiceStub,
      object_world_service: ObjectWorldServiceStub,
  ):
    """Constructs a new Simulation object.

    Args:
      simulation_service: The gRPC stub to be used for communication with the
        simulation service.
      object_world_service: The gRPC stub to be used for communication with the
        object world service.
    """
    self._simulation_service: SimulationServiceStub = simulation_service
    self._object_world_service: ObjectWorldServiceStub = object_world_service

  @classmethod
  def connect(cls, grpc_channel: grpc.Channel) -> 'Simulation':
    """Create a Simulation instance using the given gRPC address.

    Args:
      grpc_channel: Address of the simulation gRPC service to use.

    Returns:
      A newly created Simulation instance.
    """
    simulation_service = SimulationServiceStub(grpc_channel)
    object_world_service = ObjectWorldServiceStub(grpc_channel)
    return cls(simulation_service, object_world_service)

  @error_handling.retry_on_grpc_unavailable
  def _call_simulation_service_reset(
      self, request: simulation_service_pb2.ResetSimulationRequest
  ):
    return self._simulation_service.ResetSimulation(request)

  @error_handling.retry_on_grpc_unavailable
  def reset(self) -> None:
    """Resets the simulation world to its initial state.

    Deprecated: To reset the simulation world before running a process, set the
    `start_from_world_state` parameter in `Solution.executive.run()` instead.
    See execution.py for more details.

    Also makes sure that all affected components such as ICON are in a working
    state.
    """
    warnings.warn(
        'Simulation.reset() is deprecated. To reset the simulation'
        ' world before running a process, set the `start_from_world_state'
        ' parameter in `solution.executive.run()` instead. See docstring for'
        ' `solution.executive.run()` for more details.',
        DeprecationWarning,
        stacklevel=2,
    )

    request = simulation_service_pb2.ResetSimulationRequest()
    self._call_simulation_service_reset(request)


  @error_handling.retry_on_grpc_unavailable
  def _call_object_world_service_get_transform(
      self, request: object_world_service_pb2.GetTransformRequest
  ) -> object_world_service_pb2.GetTransformResponse:
    return self._object_world_service.GetTransform(request)

  @error_handling.retry_on_grpc_unavailable
  def get_transform(
      self,
      node_a: object_world_resources.TransformNode,
      node_b: object_world_resources.TransformNode,
  ) -> data_types.Pose3:
    """Get the current transform a_t_b from the running simulation.

    The transform is retrieved for the two frames in the simulation world that
    correspond to the given TransformNodes 'node_a' and 'node_b'. The
    returned transform is a_t_b which transforms points in the frame of
    'node_a' to the frame of 'node_b'.

    Args:
      node_a: A TransformNode.
      node_b: A TransformNode.

    Returns:
      Transform a_t_b.
    """
    if not isinstance(
        node_a, object_world_resources.TransformNode
    ) or not isinstance(node_b, object_world_resources.TransformNode):
      raise TypeError(
          f'Cannot get a transform between "{node_a.__repr__()}"'
          f' and "{node_b.__repr__()}"'
          ' - expected two world TransformNode objects.'
      )

    if node_a.world_id != node_b.world_id:
      raise errors.InvalidArgumentError(
          'Both transform nodes have to live in the same world. a is in'
          f'{node_a.world_id}, b in {node_b.world_id}'
      )

    request = object_world_service_pb2.GetTransformRequest(
        world_id=_SIM_WORLD_ID
    )
    request.node_a.CopyFrom(node_a.transform_node_reference)
    request.node_b.CopyFrom(node_b.transform_node_reference)
    response = self._call_object_world_service_get_transform(request)

    return proto_conversion.pose_from_proto(response.a_t_b)


