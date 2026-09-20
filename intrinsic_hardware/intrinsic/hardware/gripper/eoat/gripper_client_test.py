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

from unittest import mock

from absl.testing import absltest
from absl.testing import parameterized
import grpc

from intrinsic.hardware.gripper.eoat import eoat_service_pb2
from intrinsic.hardware.gripper.eoat import eoat_service_pb2_grpc
from intrinsic.hardware.gripper.eoat import gripper_client as gc
from intrinsic.hardware.gripper.service.proto import generic_gripper_pb2
from intrinsic.hardware.gripper.service.proto import generic_gripper_pb2_grpc
from intrinsic.icon.python import errors
from intrinsic.resources.proto import resource_handle_pb2
from intrinsic.util.grpc import connection

GRASP_REQUESTS = {
    gc.GripperTypes.SUCTION: eoat_service_pb2.GraspRequest(),
    gc.GripperTypes.PINCH: eoat_service_pb2.GraspRequest(),
    gc.GripperTypes.ADAPTIVE_PINCH: generic_gripper_pb2.GraspRequest(),
}
GRASP_RESPONSES = {
    gc.GripperTypes.SUCTION: eoat_service_pb2.GraspResponse(),
    gc.GripperTypes.PINCH: eoat_service_pb2.GraspResponse(),
    gc.GripperTypes.ADAPTIVE_PINCH: generic_gripper_pb2.GraspResponse(),
}
RELEASE_REQUESTS = {
    gc.GripperTypes.SUCTION: eoat_service_pb2.ReleaseRequest(),
    gc.GripperTypes.PINCH: eoat_service_pb2.ReleaseRequest(),
    gc.GripperTypes.ADAPTIVE_PINCH: generic_gripper_pb2.ReleaseRequest(),
}
RELEASE_RESPONSES = {
    gc.GripperTypes.SUCTION: eoat_service_pb2.ReleaseResponse(),
    gc.GripperTypes.PINCH: eoat_service_pb2.ReleaseResponse(),
    gc.GripperTypes.ADAPTIVE_PINCH: generic_gripper_pb2.ReleaseResponse(),
}
GRIPPING_INDICATED_REQUESTS = {
    gc.GripperTypes.SUCTION: eoat_service_pb2.GrippingIndicatedRequest(),
    gc.GripperTypes.PINCH: eoat_service_pb2.GrippingIndicatedRequest(),
    gc.GripperTypes.ADAPTIVE_PINCH: (
        generic_gripper_pb2.GrippingIndicatedRequest()
    ),
}
GRIPPING_INDICATED_RESPONSES = {
    gc.GripperTypes.SUCTION: eoat_service_pb2.GrippingIndicatedResponse(
        indicated=True
    ),
    gc.GripperTypes.PINCH: eoat_service_pb2.GrippingIndicatedResponse(
        indicated=True
    ),
    gc.GripperTypes.ADAPTIVE_PINCH: (
        generic_gripper_pb2.GrippingIndicatedResponse(indicated=True)
    ),
}
GRIPPER_TYPES = [
    gc.GripperTypes.SUCTION,
    gc.GripperTypes.PINCH,
    gc.GripperTypes.ADAPTIVE_PINCH,
]


def make_stub(gripper_type):
  if gripper_type == gc.GripperTypes.SUCTION:
    stub = mock.MagicMock(
        spec=eoat_service_pb2_grpc.SuctionGripperStub,
        Grasp=mock.MagicMock(),
        Release=mock.MagicMock(),
        BlowOff=mock.MagicMock(),
        GrippingIndicated=mock.MagicMock(),
    )
    stub.BlowOff.return_value = eoat_service_pb2.BlowOffResponse()
  elif gripper_type == gc.GripperTypes.PINCH:
    stub = mock.MagicMock(
        spec=eoat_service_pb2_grpc.PinchGripperStub,
        Grasp=mock.MagicMock(),
        Release=mock.MagicMock(),
        GrippingIndicated=mock.MagicMock(),
    )
  elif gripper_type == gc.GripperTypes.ADAPTIVE_PINCH:
    stub = mock.MagicMock(
        spec=generic_gripper_pb2_grpc.GenericGripperStub,
        Grasp=mock.MagicMock(),
        Release=mock.MagicMock(),
        GrippingIndicated=mock.MagicMock(),
    )
  else:
    raise ValueError(f'Unknown gripper type: {gripper_type}')

  stub.Grasp.return_value = GRASP_RESPONSES[gripper_type]
  stub.Release.return_value = RELEASE_RESPONSES[gripper_type]
  stub.GrippingIndicated.return_value = GRIPPING_INDICATED_RESPONSES[
      gripper_type
  ]
  return stub


class GripperClientTest(parameterized.TestCase):

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_local(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    credentials = grpc.local_channel_credentials()
    mock_local_channel_credentials.return_value = credentials
    gripper_client = gc.GripperClient.connect(insecure=False)
    self.assertIsInstance(gripper_client, gc.GripperClient)
    mock_secure_channel.assert_called_once_with('localhost:8128', credentials)
    mock_insecure_channel.assert_not_called()
    mock_intercept_channel.assert_called_once()
    mock_channel_ready_future.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_insecure(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    gripper_client = gc.GripperClient.connect(
        grpc_host='foo', grpc_port=1234, insecure=True
    )
    self.assertIsInstance(gripper_client, gc.GripperClient)
    mock_local_channel_credentials.assert_not_called()
    mock_secure_channel.assert_not_called()
    mock_insecure_channel.assert_called_once_with('foo:1234')
    mock_intercept_channel.assert_called_once()
    mock_channel_ready_future.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_error(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    credentials = grpc.local_channel_credentials()
    mock_local_channel_credentials.return_value = credentials
    mock_channel_ready_future.side_effect = grpc.FutureTimeoutError('foo')
    with self.assertRaises(errors.Client.ServerError):
      gc.GripperClient.connect(insecure=False)
    mock_intercept_channel.assert_not_called()
    mock_secure_channel.assert_called_once_with('localhost:8128', credentials)
    mock_insecure_channel.assert_not_called()
    mock_channel_ready_future.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_local_params(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    credentials = grpc.local_channel_credentials()
    mock_local_channel_credentials.return_value = credentials
    gripper_client = gc.GripperClient.connect_with_params(
        connection.ConnectionParams.local_port(8128), insecure=False
    )
    self.assertIsInstance(gripper_client, gc.GripperClient)
    mock_secure_channel.assert_called_once_with('localhost:8128', credentials)
    mock_insecure_channel.assert_not_called()
    mock_intercept_channel.assert_called_once()
    mock_channel_ready_future.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_with_gripper_handle(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    credentials = grpc.local_channel_credentials()
    mock_local_channel_credentials.return_value = credentials

    # Gripper handle.
    # Tests the constructor works fine when `is_simulated` is False.
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )

    gripper_client = gc.GripperClient.connect_with_gripper_handle(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info
        ),
    )
    self.assertIsInstance(gripper_client, gc.GripperClient)
    mock_secure_channel.assert_not_called()
    mock_insecure_channel.assert_called_once_with('foo:1234')
    mock_intercept_channel.assert_called_once()
    mock_channel_ready_future.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_insecure_params(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    gripper_client = gc.GripperClient.connect_with_params(
        connection.ConnectionParams.no_ingress('foo:1234'), insecure=True
    )
    self.assertIsInstance(gripper_client, gc.GripperClient)
    mock_local_channel_credentials.assert_not_called()
    mock_secure_channel.assert_not_called()
    mock_insecure_channel.assert_called_once_with('foo:1234')
    mock_intercept_channel.assert_called_once()
    mock_channel_ready_future.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_connect_error_params(
      self,
      mock_insecure_channel,
      mock_secure_channel,
      mock_local_channel_credentials,
      mock_channel_ready_future,
      mock_intercept_channel,
  ):
    credentials = grpc.local_channel_credentials()
    mock_local_channel_credentials.return_value = credentials
    mock_channel_ready_future.side_effect = grpc.FutureTimeoutError('foo')
    with self.assertRaises(errors.Client.ServerError):
      gc.GripperClient.connect_with_params(
          connection.ConnectionParams.local_port(8128), insecure=False
      )
    mock_secure_channel.assert_called_once_with('localhost:8128', credentials)
    mock_insecure_channel.assert_not_called()
    mock_intercept_channel.assert_not_called()
    mock_channel_ready_future.assert_called_once()

  @parameterized.product(
      gripper_type=GRIPPER_TYPES,
  )
  def test_construct_gripper_client(self, gripper_type):
    stub = make_stub(gripper_type)
    gripper_client = gc.GripperClient(stub, gripper_type)
    self.assertIsInstance(gripper_client, gc.GripperClient)

  @parameterized.product(
      gripper_type=GRIPPER_TYPES,
  )
  def test_construct_gripper_client_wrong_type_raises(self, gripper_type):
    stub = make_stub(gripper_type)
    for gt in [
        gc.GripperTypes.SUCTION,
        gc.GripperTypes.PINCH,
        gc.GripperTypes.ADAPTIVE_PINCH,
        gc.GripperTypes.UNKNOWN,
    ]:
      if gt != gripper_type:
        with self.assertRaises(ValueError):
          gc.GripperClient(stub, gt)

  @parameterized.product(
      gripper_type=GRIPPER_TYPES,
      timeout=[None, 1],
  )
  def test_grasp(self, gripper_type, timeout):
    stub = make_stub(gripper_type)
    gripper_client = gc.GripperClient(stub, gripper_type, rpc_timeout=timeout)
    gripper_client.grasp()
    stub.Grasp.assert_called_once_with(
        GRASP_REQUESTS[gripper_type], timeout=timeout
    )

  @parameterized.product(
      gripper_type=GRIPPER_TYPES,
      timeout=[None, 1],
  )
  def test_release(self, gripper_type, timeout):
    stub = make_stub(gripper_type)
    gripper_client = gc.GripperClient(stub, gripper_type, rpc_timeout=timeout)
    gripper_client.release()
    stub.Release.assert_called_once_with(
        RELEASE_REQUESTS[gripper_type], timeout=timeout
    )

  @parameterized.product(
      gripper_type=[gc.GripperTypes.SUCTION],
      timeout=[None, 1],
  )
  def test_blow_off_suction(self, gripper_type, timeout):
    stub = make_stub(gripper_type)
    gripper_client = gc.GripperClient(stub, gripper_type, rpc_timeout=timeout)
    gripper_client.blow_off()
    stub.BlowOff.assert_called_once_with(
        eoat_service_pb2.BlowOffRequest(), timeout=timeout
    )

  @parameterized.product(
      gripper_type=[gc.GripperTypes.PINCH, gc.GripperTypes.ADAPTIVE_PINCH],
      timeout=[None, 1],
  )
  def test_blow_off_not_supported(self, gripper_type, timeout):
    stub = make_stub(gripper_type)
    gripper_client = gc.GripperClient(stub, gripper_type, rpc_timeout=timeout)
    with self.assertRaises(ValueError):
      gripper_client.blow_off()

  @parameterized.product(
      gripper_type=GRIPPER_TYPES,
      timeout=[None, 1],
  )
  def test_gripping_indicated(self, gripper_type, timeout):
    stub = make_stub(gripper_type)
    gripper_client = gc.GripperClient(stub, gripper_type, rpc_timeout=timeout)
    success = gripper_client.gripping_indicated()

    # Check post-conditions.
    stub.GrippingIndicated.assert_called_once_with(
        GRIPPING_INDICATED_REQUESTS[gripper_type], timeout=timeout
    )
    self.assertTrue(success)


if __name__ == '__main__':
  absltest.main()
