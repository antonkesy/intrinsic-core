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

import logging
from unittest import mock

from absl.testing import absltest
from absl.testing import parameterized
import grpc

from intrinsic.hardware.gripper.eoat import gripper_client
from intrinsic.hardware.gripper.eoat import gripper_python_utils
from intrinsic.hardware.gripper.service.proto import generic_gripper_pb2
from intrinsic.resources.proto import resource_handle_pb2
from intrinsic.util.grpc import connection


class SuctionGripperTest(parameterized.TestCase):

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_constructor_works_when_not_is_simulated(
      self,
      *_,
  ):
    # Tests the constructor works fine when `is_simulated` is False.
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
    )
    self.assertIsInstance(suction_gripper, gripper_python_utils.SuctionGripper)
    self.assertIsNotNone(suction_gripper._client)

  def test_constructor_works_when_is_simulated(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    self.assertIsInstance(suction_gripper, gripper_python_utils.SuctionGripper)
    self.assertIsNone(suction_gripper._client)

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_grasp_works_when_not_simulated(self, *_):
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
    )
    suction_gripper._client = mock.MagicMock()
    suction_gripper.grasp()

    # Check post-conditions
    suction_gripper._client.grasp.assert_called_once_with()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_release_works_when_not_simulated(self, *_):
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
    )
    suction_gripper._client = mock.MagicMock()
    suction_gripper.release()

    # Check post-conditions
    suction_gripper._client.release.assert_called_once_with()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_blow_off_works_when_not_simulated(self, *_):
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
    )
    suction_gripper._client = mock.MagicMock()
    suction_gripper.blow_off()

    # Check post-conditions
    suction_gripper._client.blow_off.assert_called_once_with()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_gripping_indicated_works_when_not_simulated(self, *_):
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
    )
    suction_gripper._client = mock.MagicMock()
    suction_gripper.gripping_indicated()

    # Check post-conditions
    suction_gripper._client.gripping_indicated.assert_called_once_with()

  def test_grasp_works_when_is_simulated(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    suction_gripper.grasp()

  def test_release_works_when_is_simulated(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    suction_gripper.release()

  def test_blow_off_works_when_is_simulated(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    suction_gripper.blow_off()

  def test_gripping_indicated_works_when_is_simulated(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    self.assertTrue(suction_gripper.gripping_indicated())

  def test_type_is_correct(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    self.assertEqual(suction_gripper.type, gripper_client.GripperTypes.SUCTION)

  def test_name_is_correct(self):
    gripper_handle = mock.Mock()
    gripper_handle.name = 'SuctionGripper'
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=gripper_handle,
        is_simulated=True,
    )
    self.assertEqual(suction_gripper.name, gripper_handle.name)

  def test_command_not_supported(self):
    suction_gripper = gripper_python_utils.SuctionGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
    )
    with self.assertRaises(NotImplementedError):
      suction_gripper.command(generic_gripper_pb2.CommandRequest())


class PinchGripperTest(parameterized.TestCase):

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_constructor_works_when_not_is_simulated(
      self,
      *_,
  ):
    # Tests the constructor works fine when `is_simulated` is False.
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock.MagicMock(),
    )
    self.assertIsInstance(pinch_gripper, gripper_python_utils.PinchGripper)
    self.assertIsNotNone(pinch_gripper._client)

  def test_constructor_works_when_is_simulated(self):
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock.MagicMock(),
    )
    self.assertIsInstance(pinch_gripper, gripper_python_utils.PinchGripper)
    self.assertIsNone(pinch_gripper._client)

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_grasp_works_when_not_simulated(self, *_):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.min_position.values = [1, 2, 3]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [1, 2, 3])

    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock_world,
    )
    pinch_gripper._client = mock.MagicMock()
    pinch_gripper.grasp()

    # Check post-conditions
    pinch_gripper._client.grasp.assert_called_once_with()
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_release_works_when_not_simulated(self, *_):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.max_position.values = [4, 5, 6]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [4, 5, 6])

    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock_world,
    )
    pinch_gripper._client = mock.MagicMock()
    pinch_gripper.release()

    # Check post-conditions
    pinch_gripper._client.release.assert_called_once_with()
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_gripping_indicated_works_when_not_simulated(self, *_):
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock.MagicMock(),
    )
    pinch_gripper._client = mock.MagicMock()
    pinch_gripper.gripping_indicated()

    # Check post-conditions
    pinch_gripper._client.gripping_indicated.assert_called_once_with()

  def test_grasp_works_when_is_simulated(self):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.min_position.values = [1, 2, 3]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [1, 2, 3])

    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)

    # Call "grasp".
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock_world,
    )
    pinch_gripper.grasp()

    # Check post-conditions
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  def test_release_works_when_is_simulated(self):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.max_position.values = [4, 5, 6]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [4, 5, 6])

    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)

    # Call "release".
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock_world,
    )
    pinch_gripper.release()

    # Check post-conditions
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  def test_gripping_indicated_works_when_is_simulated(self):
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock.MagicMock(),
    )
    self.assertTrue(pinch_gripper.gripping_indicated())

  def test_type_is_correct(self):
    suction_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=mock.Mock(),
        world=mock.MagicMock(),
        is_simulated=True,
    )
    self.assertEqual(suction_gripper.type, gripper_client.GripperTypes.PINCH)

  def test_name_is_correct(self):
    gripper_handle = mock.Mock()
    gripper_handle.name = 'PinchGripper'
    suction_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=gripper_handle,
        world=mock.MagicMock(),
        is_simulated=True,
    )
    self.assertEqual(suction_gripper.name, gripper_handle.name)

  def test_command_not_supported(self):
    pinch_gripper = gripper_python_utils.PinchGripper(
        gripper_handle=mock.Mock(),
        world=mock.MagicMock(),
        is_simulated=True,
    )
    with self.assertRaises(NotImplementedError):
      pinch_gripper.command(generic_gripper_pb2.CommandRequest())


class AdaptivePinchGripperTest(parameterized.TestCase):

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_constructor_works_when_not_is_simulated(
      self,
      *_,
  ):
    # Tests the constructor works fine when `is_simulated` is False.
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock.MagicMock(),
    )
    self.assertIsInstance(
        adaptive_pinch_gripper, gripper_python_utils.AdaptivePinchGripper
    )
    self.assertIsNotNone(adaptive_pinch_gripper._client)

  def test_constructor_works_when_is_simulated(self):
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock.MagicMock(),
    )
    self.assertIsInstance(
        adaptive_pinch_gripper, gripper_python_utils.AdaptivePinchGripper
    )
    self.assertIsNone(adaptive_pinch_gripper._client)

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_grasp_works_when_not_simulated(self, *_):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.max_position.values = [1, 2, 3]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [1, 2, 3])

    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock_world,
    )
    adaptive_pinch_gripper._client = mock.MagicMock()
    adaptive_pinch_gripper.grasp()

    # Check post-conditions
    adaptive_pinch_gripper._client.grasp.assert_called_once_with()
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_release_works_when_not_simulated(self, *_):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.min_position.values = [4, 5, 6]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [4, 5, 6])

    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock_world,
    )
    adaptive_pinch_gripper._client = mock.MagicMock()
    adaptive_pinch_gripper.release()

    # Check post-conditions
    adaptive_pinch_gripper._client.release.assert_called_once_with()
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_gripping_indicated_works_when_not_simulated(self, *_):
    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock.MagicMock(),
    )
    adaptive_pinch_gripper._client = mock.MagicMock()
    adaptive_pinch_gripper.gripping_indicated()

    # Check post-conditions
    adaptive_pinch_gripper._client.gripping_indicated.assert_called_once_with()

  @parameterized.parameters(
      (generic_gripper_pb2.CommandRequest(position=0.5)),
      (generic_gripper_pb2.CommandRequest(position_percentage=70.0)),
      (generic_gripper_pb2.CommandRequest(velocity=0.5)),
      (generic_gripper_pb2.CommandRequest(velocity_percentage=70.0)),
      (generic_gripper_pb2.CommandRequest(effort=0.5)),
      (generic_gripper_pb2.CommandRequest(effort_percentage=70.0)),
      (
          generic_gripper_pb2.CommandRequest(
              position=0.5, velocity=0.5, effort=0.5
          )
      ),
  )
  @mock.patch.object(grpc, 'intercept_channel', autospec=True)
  @mock.patch.object(grpc, 'channel_ready_future', autospec=True)
  @mock.patch.object(grpc, 'local_channel_credentials', autospec=True)
  @mock.patch.object(grpc, 'secure_channel', autospec=True)
  @mock.patch.object(grpc, 'insecure_channel', autospec=True)
  def test_command_works_when_not_simulated(
      self,
      command: generic_gripper_pb2.CommandRequest,
      *_,
  ):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.max_position.values = [1, 2, 3]
    mock_gripper_object.joint_application_limits.min_position.values = [0, 0, 0]

    connection_params = connection.ConnectionParams.no_ingress('foo:1234')
    connection_info = resource_handle_pb2.ResourceConnectionInfo(
        grpc=resource_handle_pb2.ResourceGrpcConnectionInfo(
            address=connection_params.address,
            server_instance=connection_params.instance_name,
            header=connection_params.header,
        )
    )
    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock()
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=resource_handle_pb2.ResourceHandle(
            connection_info=connection_info,
        ),
        is_simulated=False,
        world=mock_world,
    )
    adaptive_pinch_gripper._client = mock.MagicMock()
    # Set position
    adaptive_pinch_gripper._client.command.return_value = (
        generic_gripper_pb2.CommandResponse(position=0.5, position_reached=True)
    )
    adaptive_pinch_gripper.command(command=command)

    # Check post-conditions
    adaptive_pinch_gripper._client.command.assert_called_once_with(command)
    mock_world.update_joint_positions.assert_called_once()
    self.assertSequenceAlmostEqual(
        mock_world.update_joint_positions.call_args.kwargs['joint_positions'],
        [0.75, 1.75, 2.75],
    )

  def test_grasp_works_when_is_simulated(self):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.max_position.values = [1, 2, 3]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [1, 2, 3])

    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)

    # Call "grasp".
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock_world,
    )
    adaptive_pinch_gripper.grasp()

    # Check post-conditions
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  def test_release_works_when_is_simulated(self):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.min_position.values = [4, 5, 6]

    def mock_ujp(kinematic_object, joint_positions, **kwargs):
      # Mocks the `update_joint_positions` function, verifies parameter values.
      del kwargs
      self.assertEqual(kinematic_object, mock_gripper_object)
      self.assertEqual(joint_positions, [4, 5, 6])

    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )
    mock_world.update_joint_positions = mock.Mock(side_effect=mock_ujp)

    # Call "release".
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock_world,
    )
    adaptive_pinch_gripper.release()

    # Check post-conditions
    mock_world.get_kinematic_object.assert_called_once()
    mock_world.update_joint_positions.assert_called_once()

  def test_gripping_indicated_works_when_is_simulated(self):
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock.MagicMock(),
    )
    self.assertTrue(adaptive_pinch_gripper.gripping_indicated())

  @parameterized.parameters(
      (generic_gripper_pb2.CommandRequest(position=0.5), [0.75, 1.75, 2.75]),
      (
          generic_gripper_pb2.CommandRequest(position_percentage=70.0),
          [0.7, 1.4, 2.1],
      ),
      (
          generic_gripper_pb2.CommandRequest(
              position=0.5, velocity=0.5, effort=0.5
          ),
          [0.75, 1.75, 2.75],
      ),
  )
  def test_command_set_position_works_when_is_simulated(
      self,
      command: generic_gripper_pb2.CommandRequest,
      expected_joint_positions: list[float],
  ):
    mock_gripper_object = mock.MagicMock()
    mock_gripper_object.joint_application_limits.max_position.values = [1, 2, 3]
    mock_gripper_object.joint_application_limits.min_position.values = [0, 0, 0]

    mock_world = mock.MagicMock()
    mock_world.get_kinematic_object = mock.Mock(
        return_value=mock_gripper_object
    )

    mock_world.update_joint_positions = mock.Mock()
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=mock.Mock(),
        is_simulated=True,
        world=mock_world,
    )
    # Set position
    adaptive_pinch_gripper.command(command=command)

    # Check post-conditions
    mock_world.get_kinematic_object.assert_called_once()
    logging.info(
        'expected_joint_positions: %s',
        mock_world.update_joint_positions.call_args.kwargs['joint_positions'],
    )
    mock_world.update_joint_positions.assert_called_once()
    self.assertSequenceAlmostEqual(
        mock_world.update_joint_positions.call_args.kwargs['joint_positions'],
        expected_joint_positions,
    )

  def test_type_is_correct(self):
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=mock.Mock(),
        world=mock.MagicMock(),
        is_simulated=True,
    )
    self.assertEqual(
        adaptive_pinch_gripper.type, gripper_client.GripperTypes.ADAPTIVE_PINCH
    )

  def test_name_is_correct(self):
    gripper_handle = mock.Mock()
    gripper_handle.name = 'AdaptivePinchGripper'
    adaptive_pinch_gripper = gripper_python_utils.AdaptivePinchGripper(
        gripper_handle=gripper_handle,
        world=mock.MagicMock(),
        is_simulated=True,
    )
    self.assertEqual(adaptive_pinch_gripper.name, gripper_handle.name)


if __name__ == '__main__':
  absltest.main()
