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

"""Utils library to grab images from cameras.

This library contains a series of function useful to grab images from a set of
input cameras. The function gather_cameras_data_concurrent is a more efficient
way of doing this as it concurrently capture images from multiple cameras to
reduce the time needed for it.
"""

import concurrent.futures
import datetime
import logging
from typing import List
from typing import Optional

from intrinsic.perception.client.v1.python.camera import cameras
from intrinsic.perception.client.v1.python.camera import data_classes

_MAX_FRAME_WAIT_TIME_SECONDS = 180


def capture_from_camera(
    camera: cameras.Camera, sensor_names: Optional[List[str]] = None
) -> data_classes.CaptureResult:
  """Capture data from a single multi_sensor camera.

  Args:
    camera: camera from which the data is gathered.
    sensor_names: name of the sensors within the camera to capture images from.
      If None all images from all the sensors are gathered.

  Returns:
    Result of the capture from the input camera and specified sensors.
  """

  logging.info(
      "sensor_names: %s, camera.factory_sensor_info: %s",
      sensor_names,
      camera.factory_sensor_info,
  )
  # This is a temporary solution to support the current IPS2 SKU2
  # camera driver which does not have a sensor named P0.
  if (
      sensor_names is not None
      and "P0" in sensor_names
      and "P0" not in camera.factory_sensor_info.keys()
  ):
    sensor_names.remove("P0")
    sensor_names.append("RGB0")
  # Need to do the opposite when RGB0 is detected but is actually P0
  elif (
      sensor_names is not None
      and "RGB0" in sensor_names
      and "RGB0" not in camera.factory_sensor_info.keys()
  ):
    sensor_names.remove("RGB0")
    sensor_names.append("P0")
  logging.info(
      "Capturing sensors %s from camera %s", sensor_names, camera.display_name
  )
  capture_result = camera.capture(
      sensor_names=sensor_names,
      timeout=datetime.timedelta(seconds=_MAX_FRAME_WAIT_TIME_SECONDS),
  )
  for sensor_name, sensor_image in capture_result.sensor_images.items():
    logging.info(
        "Captured image of size %s from %s with camera %s",
        sensor_image.array.shape,
        sensor_name,
        camera.display_name,
    )
  return capture_result


# TODO(dallolio): unify this function and the one in
# pose_estimation_service_utils into a unique library.
def gather_cameras_data_concurrent(
    input_cameras: List[cameras.Camera],
    sensor_names: Optional[List[str]] = None,
) -> List[data_classes.CaptureResult]:
  """Given skill context and cameras slots it concurrently grabs cameras data.

  Args:
    input_cameras: list of cameras from which the data is gathered.
    sensor_names: name of the sensors within a camera to capture images from. If
      None all images from all the sensors are gathered.

  Returns:
    Results of the capture from the input cameras and specified sensors.
  """
  # We need as many sensor_names as input cameras to run the capture
  # concurrently.
  sensor_names = [sensor_names] * len(input_cameras)
  with concurrent.futures.ThreadPoolExecutor() as executor:
    # Concurrent execution with different slot values.
    capture_results = list(
        executor.map(capture_from_camera, input_cameras, sensor_names)
    )

  return capture_results
