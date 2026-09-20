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

"""
Service handler of the inference service, this implements the OIP API and routes
requests to the inference server implementation running inside the service.
"""

from __future__ import annotations

import dataclasses
import threading

from absl import logging
import grpc
from tritonclient.grpc import service_pb2 as triton_pb2
from tritonclient.grpc import service_pb2_grpc as triton_pb2_grpc

from intrinsic_inference.assets.inference_service.v1 import inference_service_config_pb2
from intrinsic_inference.core import inference_runner as inference_runner_lib
from intrinsic_inference.core import model_controller_base
from intrinsic_inference.core.v1 import ml_model_pb2
from intrinsic.assets.services.proto.v1 import service_state_pb2 as state_proto
from intrinsic.assets.services.proto.v1 import service_state_pb2_grpc as state_grpc
from intrinsic.util.grpc import error_handling

GRPC_OPTIONS = [
    ("grpc.max_receive_message_length", -1),
    ("grpc.max_send_message_length", -1),
]
MAX_PUBSUB_RETRIEVAL_TIMEOUT_IN_SECONDS = 30.0


@dataclasses.dataclass
class RuntimeState:
  """Mutable runtime state of the inference service, protected by _state_lock.

  Attributes:
    status: The current runtime status of the service instance.
    config: Configuration of the inference service.
    inference_runner: Runner that manages communication with the backend server.
  """

  status: state_proto.SelfState
  config: inference_service_config_pb2.InferenceServiceConfig
  inference_runner: inference_runner_lib.InferenceRunner

class InferenceServiceServicer(
    triton_pb2_grpc.GRPCInferenceServiceServicer,
    state_grpc.ServiceStateServicer,
):
  """Inference service."""

  def __init__(
      self,
      inference_runner: inference_runner_lib.InferenceRunner,
  ):
    # Constructor is fast and side-effect free.
    # _update_state_safe, Disable) access properties (e.g., inference_runner)
    #  that also acquire it.
    self._state_lock = threading.RLock()
    # RLock is required because Enable acquires this lock and then calls
    # start(), which also attempts to acquire it.
    self._admin_lock = threading.RLock()
    self._runtime_state = RuntimeState(
        status=state_proto.SelfState(
            state_code=state_proto.SelfState.STATE_CODE_DISABLED,
        ),
        config=inference_service_config_pb2.InferenceServiceConfig(),
        inference_runner=inference_runner,
    )

  @property
  def state(self) -> state_proto.SelfState:
    with self._state_lock:
      return self._runtime_state.status

  @state.setter
  def state(self, new_status: state_proto.SelfState):
    with self._state_lock:
      self._runtime_state.status = new_status

  @property
  def config(self) -> inference_service_config_pb2.InferenceServiceConfig:
    with self._state_lock:
      return self._runtime_state.config

  @config.setter
  def config(
      self, new_config: inference_service_config_pb2.InferenceServiceConfig
  ):
    with self._state_lock:
      self._runtime_state.config = new_config

  @property
  def inference_runner(self) -> inference_runner_lib.InferenceRunner:
    with self._state_lock:
      return self._runtime_state.inference_runner

  @property
  def installed_models(self) -> dict[str, ml_model_pb2.MlModel]:
    with self._state_lock:
      return self.inference_runner.installed_models

  @property
  def installed_model_states(
      self,
  ) -> dict[str, model_controller_base.ModelAndState]:
    with self._state_lock:
      return self.inference_runner.installed_model_states

  def _update_state_safe(
      self, new_state_code: int, extended_status: dict | None = None
  ) -> bool:
    """Helper to transition state unless the service has been disabled."""
    with self._state_lock:
      if (
          self._runtime_state.status.state_code
          == state_proto.SelfState.STATE_CODE_DISABLED
          and self.inference_runner.is_started
      ):
        return False
      self._runtime_state.status = state_proto.SelfState(
          state_code=new_state_code,
          extended_status=extended_status,
      )
      return True

  def start(
      self,
  ) -> None:
    """Starts the inference runner."""
    with self._admin_lock:
      with self._state_lock:
        self._runtime_state.status = state_proto.SelfState(
            state_code=state_proto.SelfState.STATE_CODE_UNSPECIFIED
        )
      try:
        self.inference_runner.start()
        # When everything is done, service transitions to 'Enabled'.
        if (
            self.inference_runner.server_state.state
            == inference_runner_lib.ServerState.READY
        ):
          self._update_state_safe(state_proto.SelfState.STATE_CODE_ENABLED)
        else:
          message = self.inference_runner.server_state.message
          self._update_state_safe(
              state_proto.SelfState.STATE_CODE_ERROR,
              {
                  "title": "Initialization Error",
                  "debug_report": {"message": message},
              },
          )
      except Exception as e:
        logging.error("Initialization failed: %s", str(e))
        self._update_state_safe(
            state_proto.SelfState.STATE_CODE_ERROR,
            {
                "title": "Initialization Error",
                "debug_report": {"message": str(e)},
            },
        )
        raise

  def stop(self) -> None:
    """Does a clean shutdown of the inference runner."""
    self.inference_runner.stop()
  def _check_if_enabled(self, context: grpc.ServicerContext) -> None:
    # Lock-free read of state code (Hot Path).
    # This is safe because status object replacement is atomic.
    if (
        self._runtime_state.status.state_code
        == state_proto.SelfState.STATE_CODE_ENABLED
    ):
      return

    # Failure Path: Lock and get consistent snapshot of full state for error details.
    with self._state_lock:
      current_state = self._runtime_state.status

    if current_state.state_code == state_proto.SelfState.STATE_CODE_DISABLED:
      if self.inference_runner.server_state.state in [
          inference_runner_lib.ServerState.UNKNOWN,
          inference_runner_lib.ServerState.LIVE,
      ]:
        error_msg = (
            "The inference service is not yet initialized, it will"
            " automatically enable itself once it is ready."
        )
      elif (
          self.inference_runner.server_state.state
          == inference_runner_lib.ServerState.READY
      ):
        error_msg = (
            "Cannot make calls to a disabled Service; use the Service manager"
            " to enable the 'Inference Service'."
        )
      else:  # Error state.
        error_msg = self.inference_runner.server_state.message
      status = error_handling.make_grpc_status(
          code=grpc.StatusCode.FAILED_PRECONDITION,
          message=error_msg,
          details=[],
      )
      logging.info(error_msg)
      context.abort_with_status(status)
    elif current_state.state_code == state_proto.SelfState.STATE_CODE_ERROR:
      error_msg = "Cannot make calls to a Service in error state!"
      details = []
      if current_state.HasField("extended_status"):
        details.append(current_state.extended_status)
      status = error_handling.make_grpc_status(
          code=grpc.StatusCode.FAILED_PRECONDITION,
          message=error_msg,
          details=details,
      )
      logging.info(error_msg)
      context.abort_with_status(status)
    else:
      error_msg = (
          "The inference service is not ready to serve requests (state:"
          f" {current_state.state_code})."
      )
      status = error_handling.make_grpc_status(
          code=grpc.StatusCode.FAILED_PRECONDITION,
          message=error_msg,
          details=[],
      )
      logging.info(error_msg)
      context.abort_with_status(status)

  def GetState(
      self,
      request: state_proto.GetStateRequest,  # pylint: disable=unused-argument
      context: grpc.ServicerContext,  # pylint: disable=unused-argument
  ) -> state_proto.SelfState:
    return self.state

  def Enable(
      self,
      request: state_proto.EnableRequest,  # pylint: disable=unused-argument
      context: grpc.ServicerContext,
  ) -> state_proto.EnableResponse:
    # Serialize the enable flow to prevent concurrent initialization storms.
    with self._admin_lock:
      with self._state_lock:
        current_state_code = self._runtime_state.status.state_code
        if current_state_code == state_proto.SelfState.STATE_CODE_ENABLED:
          return state_proto.EnableResponse()
        elif current_state_code == state_proto.SelfState.STATE_CODE_ERROR:
          logging.info("Error was acknowledged, resetting service state.")
          self._runtime_state.status = state_proto.SelfState(
              state_code=state_proto.SelfState.STATE_CODE_UNSPECIFIED
          )
        elif current_state_code == state_proto.SelfState.STATE_CODE_DISABLED:
          self._runtime_state.status = state_proto.SelfState(
              state_code=state_proto.SelfState.STATE_CODE_UNSPECIFIED
          )

      self.start()

      with self._state_lock:
        if (
            self._runtime_state.status.state_code
            != state_proto.SelfState.STATE_CODE_ENABLED
        ):
          context.abort(
              grpc.StatusCode.FAILED_PRECONDITION, "Failed to enable service."
          )

    return state_proto.EnableResponse()

  def Disable(
      self,
      request: state_proto.DisableRequest,  # pylint: disable=unused-argument
      context: grpc.ServicerContext,
  ) -> state_proto.DisableResponse:
    # The admin lock is acquired here to avoid a potential race condition where
    # Disable() executes after Enable() released the state lock but before it
    # finished self.start().
    with self._admin_lock:
      with self._state_lock:
        current_state_code = self._runtime_state.status.state_code
        if current_state_code == state_proto.SelfState.STATE_CODE_ERROR:
          context.abort(
              grpc.StatusCode.FAILED_PRECONDITION,
              "Cannot disable Service in error state.",
          )
        elif current_state_code == state_proto.SelfState.STATE_CODE_DISABLED:
          # Already disabled, do nothing.
          return state_proto.DisableResponse()

        self._runtime_state.status = state_proto.SelfState(
            state_code=state_proto.SelfState.STATE_CODE_DISABLED
        )

      self.stop()
    return state_proto.DisableResponse()

  def ServerLive(
      self,
      request: triton_pb2.ServerLiveRequest,
      context: grpc.ServicerContext,
  ) -> triton_pb2.ServerLiveResponse:
    self._check_if_enabled(context)
    return self.inference_runner.ServerLive(request)

  def ServerReady(
      self,
      request: triton_pb2.ServerReadyRequest,
      context: grpc.ServicerContext,
  ) -> triton_pb2.ServerReadyResponse:
    self._check_if_enabled(context)
    return self.inference_runner.ServerReady(request)

  def ModelReady(
      self,
      request: triton_pb2.ModelReadyRequest,
      context: grpc.ServicerContext,
  ) -> triton_pb2.ModelReadyResponse:
    self._check_if_enabled(context)
    return self.inference_runner.ModelReady(request)

  def ServerMetadata(
      self,
      request: triton_pb2.ServerMetadataRequest,
      context: grpc.ServicerContext,
  ) -> triton_pb2.ServerMetadataResponse:
    self._check_if_enabled(context)
    response = self.inference_runner.ServerMetadata(request)
    # Clear default triton extensions.
    del response.extensions[:]
    return response

  def ModelMetadata(
      self,
      request: triton_pb2.ModelMetadataRequest,
      context: grpc.ServicerContext,
  ) -> triton_pb2.ModelMetadataResponse:
    self._check_if_enabled(context)
    return self.inference_runner.ModelMetadata(request)

  def ModelInfer(
      self,
      request: triton_pb2.ModelInferRequest,
      context: grpc.ServicerContext,
  ) -> triton_pb2.ModelInferResponse:
    """Forwards the inference request and returns the response."""
    self._check_if_enabled(context)
    return self.inference_runner.ModelInfer(request)
