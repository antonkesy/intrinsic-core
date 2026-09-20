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

from concurrent import futures
import logging
import pathlib
from typing import Sequence

from absl import app
from absl import flags
from google.protobuf import text_format
import grpc

from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.proto import ioc_service_config_pb2
from intrinsic_perception.intrinsic.perception.service.ioc_pose_estimator.service import ioc_pose_estimator_service
from intrinsic.perception.proto.v1 import pose_estimation_service_pb2_grpc as v1_pose_estimation_service_pb2_grpc
from intrinsic.resources.proto import runtime_context_pb2

logging.basicConfig(
    level=logging.INFO,
    format=(
        '[%(asctime)s] <%(filename)s:%(lineno)d> %(levelname)s]-\t %(message)s'
    ),
)


_DEFAULT_PORT = 50051

_RUNTIME_CONTEXT_FILE = flags.DEFINE_string(
    'runtime_context_file',
    '/etc/intrinsic/runtime_config.pb',
    (
        'Path to the runtime context file containing'
        ' intrinsic_proto.config.RuntimeContext binary proto.'
    ),
)

_GRPC_OPTIONS = [
    # Remove limit on message size for e.g. images.
    ('grpc.max_receive_message_length', -1),
    ('grpc.max_send_message_length', -1),
    ('grpc.max_message_length', -1),
]


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')
  logging.info('\n')

  # Load runtime context + config.
  runtime_context = None
  config = None

  ioc_service_config = ioc_service_config_pb2.IocPoseEstimatorServiceConfig()

  if pathlib.Path(_RUNTIME_CONTEXT_FILE.value).exists():
    with open(_RUNTIME_CONTEXT_FILE.value, 'rb') as f:
      runtime_context = runtime_context_pb2.RuntimeContext.FromString(f.read())
      logging.info(
          'Runtime context: \n%s', text_format.MessageToString(runtime_context)
      )
      if runtime_context.HasField('config'):
        if runtime_context.config.Is(
            ioc_service_config_pb2.IocPoseEstimatorServiceConfig.DESCRIPTOR
        ):
          config = ioc_service_config_pb2.IocPoseEstimatorServiceConfig()
          runtime_context.config.Unpack(config)
          ioc_service_config = config

      if config is not None:
        logging.info(
            'Service config (from runtime context): %s',
            text_format.MessageToString(config, print_unknown_fields=True),
        )
      else:
        logging.warning(
            'Runtime context does not contain a valid config. Using default.'
        )

  logging.info('--- Starting IOC Pose Estimator Server ---\n')

  server = grpc.server(
      futures.ThreadPoolExecutor(max_workers=10), options=_GRPC_OPTIONS
  )

  servicer = ioc_pose_estimator_service.IocPoseEstimatorService(
      config=ioc_service_config
  )

  v1_pose_estimation_service_pb2_grpc.add_PoseEstimationServiceServicer_to_server(
      servicer,
      server,
  )

  if runtime_context is not None and runtime_context.port > 0:
    endpoint = f'[::]:{runtime_context.port}'
    added_port = server.add_insecure_port(endpoint)
    if added_port != runtime_context.port:
      raise RuntimeError(f'Failed to use port {runtime_context.port}')
  else:
    endpoint = f'[::]:{_DEFAULT_PORT}'
    added_port = server.add_insecure_port(endpoint)

  server.start()
  logging.info('-----------------------------------')
  logging.info('-- Server listening at port : %s ...', endpoint)
  logging.info('-----------------------------------')

  server.wait_for_termination()


if __name__ == '__main__':
  app.run(main)
  app.run(main)
