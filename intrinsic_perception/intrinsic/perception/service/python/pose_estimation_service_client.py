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

"""Provides a client for using the PoseEstimationService."""

from __future__ import annotations

import grpc

from intrinsic.perception.proto.v1 import pose_estimation_service_pb2
from intrinsic.perception.proto.v1 import pose_estimation_service_pb2_grpc
from intrinsic.util.grpc import error_handling


class PoseEstimationServiceClient:
  """Client for the PoseEstimationService."""

  def __init__(
      self,
      stub: pose_estimation_service_pb2_grpc.PoseEstimationServiceStub,
  ):
    """Constructs a new PoseEstimationServiceClient object.

    Args:
      stub: The gRPC stub to be used for communication with the pose estimation service.
    """
    self._stub = stub

  @classmethod
  def from_channel(
      cls, grpc_channel: grpc.Channel
  ) -> PoseEstimationServiceClient:
    """Create a new PoseEstimationServiceClient from a gRPC channel."""
    return cls(
        pose_estimation_service_pb2_grpc.PoseEstimationServiceStub(grpc_channel)
    )

  @error_handling.retry_on_grpc_unavailable
  def run_pose_estimation(
      self,
      request: pose_estimation_service_pb2.RunPoseEstimationRequest,
  ) -> pose_estimation_service_pb2.RunPoseEstimationResponse:
    """Executes pose estimation with the given configuration and input data."""
    return self._stub.RunPoseEstimation(request)
