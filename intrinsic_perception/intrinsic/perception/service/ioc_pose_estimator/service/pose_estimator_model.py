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

"""FoundationPose estimation model wrapper for IocPoseEstimatorService.

Handles Triton asset installation, readiness checking, and FoundationPose 6D pose inference.
"""

import collections
import io
import logging
import os
import time
from typing import Optional

import cv2
from google.protobuf import text_format
import grpc
import numpy as np
from specification.protocol import open_inference_grpc_pb2
from specification.protocol import open_inference_grpc_pb2_grpc
import trimesh
from triton_common.protobuf import model_config_pb2

from intrinsic_inference.core.utils import oip_mappings
from intrinsic_inference.core.utils import oip_utils
from incode.util.runfiles import runfiles
from intrinsic.assets import id_utils
from intrinsic.assets.proto.v1 import resolved_dependency_pb2

_MAX_MESH_CACHE_SIZE = 10
_CONNECT_INFERENCE_SERVICE_TIMEOUT_SECONDS = 600
_CONNECT_INFERENCE_SERVICE_POLLING_INTERVAL_SECONDS = 20
_ML_MODEL_ASSET_URI = (
    "data://intrinsic_proto.ml.inference_service.v1.MlModelAsset"
)
_DEFAULT_CONFIG_PATH = "intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/model_files/pose/config.pbtxt"


def _load_triton_config(
    config_path: Optional[str] = None,
    default_relative_path: str = _DEFAULT_CONFIG_PATH,
) -> model_config_pb2.ModelConfig:
  """Loads and parses a Triton ModelConfig protobuf text file.

  Args:
    config_path: Explicit path to the config.pbtxt file. If None, resolves via
      runfiles using default_relative_path.
    default_relative_path: Relative runfiles path to use if config_path is None.

  Returns:
    The parsed ModelConfig message.

  Raises:
    FileNotFoundError: If the config file cannot be found.
  """
  resolved_path = config_path
  if resolved_path is None:
    try:
      r_path = runfiles.RlocationCurrentRepository(default_relative_path)
      if r_path and os.path.exists(r_path):
        resolved_path = r_path
      elif os.path.exists(default_relative_path):
        resolved_path = default_relative_path
    except Exception:
      if os.path.exists(default_relative_path):
        resolved_path = default_relative_path

  if not resolved_path or not os.path.exists(resolved_path):
    raise FileNotFoundError(
        f"Triton config file not found at '{resolved_path}' "
        f"(default: '{default_relative_path}')."
    )

  config = model_config_pb2.ModelConfig()
  with open(resolved_path, "r", encoding="utf-8") as f:
    text_format.Parse(f.read(), config)
  return config


def _extract_model_id(
    dependency: resolved_dependency_pb2.ResolvedDependency,
) -> str:
  """Extracts and validates the model asset ID from a ResolvedDependency.

  Args:
    dependency: ResolvedDependency describing the ML model asset.

  Returns:
    The validated model ID string in `<package>.<name>` format.

  Raises:
    ValueError: If dependency is not provided or has no valid data.id interface.
  """
  if not dependency:
    raise ValueError("model_dependency is required and cannot be None.")

  iface = dependency.interfaces.get(_ML_MODEL_ASSET_URI)
  if iface is None:
    for candidate in dependency.interfaces.values():
      if candidate.HasField("data") and candidate.data.HasField("id"):
        iface = candidate
        break

  if (
      iface is None
      or not iface.HasField("data")
      or not iface.data.HasField("id")
  ):
    raise ValueError(
        "ResolvedDependency does not contain a valid data interface with an ID."
        f" Interfaces present: {list(dependency.interfaces.keys())}"
    )

  try:
    return id_utils.id_from_proto(iface.data.id)
  except id_utils.IdValidationError as e:
    raise ValueError(
        "Invalid asset ID in ResolvedDependency data interface:"
        f" {iface.data.id}"
    ) from e


def _create_model_infer_request(
    inputs: dict[str, np.ndarray],
    model_name: str,
    output_names: Optional[list[str]] = None,
) -> open_inference_grpc_pb2.ModelInferRequest:
  """Creates an OIP ModelInferRequest proto with concrete runtime input shapes."""
  requested_outputs = [
      open_inference_grpc_pb2.ModelInferRequest.InferRequestedOutputTensor(
          name=out_name,
      )
      for out_name in output_names or []
  ]
  return open_inference_grpc_pb2.ModelInferRequest(
      model_name=model_name,
      inputs=[
          open_inference_grpc_pb2.ModelInferRequest.InferInputTensor(
              name=k,
              datatype=oip_mappings.numpy_to_oip_type(v.dtype),
              shape=v.shape,
          )
          for k, v in inputs.items()
      ],
      outputs=requested_outputs,
      raw_input_contents=[
          np.ascontiguousarray(v).tobytes() for v in inputs.values()
      ],
  )


class PoseEstimationModel:
  """Wrapper class around Triton pose estimation model (FoundationPose)."""

  def __init__(
      self,
      ml_service_stub: open_inference_grpc_pb2_grpc.GRPCInferenceServiceStub,
      model_dependency: resolved_dependency_pb2.ResolvedDependency,
      config_path: Optional[str] = None,
      triton_config: Optional[model_config_pb2.ModelConfig] = None,
  ):
    """Initializes PoseEstimationModel.

    Args:
      ml_service_stub: gRPC inference service stub.
      model_dependency: Required ResolvedDependency specifying the model ID.
      config_path: Optional path to the Triton config.pbtxt file.
      triton_config: Optional pre-parsed ModelConfig protobuf message.
    """
    if ml_service_stub is None:
      raise ValueError("ml_service_stub is required.")
    self._ml_service_stub = ml_service_stub
    self._model_id = _extract_model_id(model_dependency)
    self._triton_config = triton_config or _load_triton_config(config_path)
    self._output_names = [out.name for out in self._triton_config.output]
    self._mesh_cache = collections.OrderedDict()

  @property
  def triton_config(self) -> model_config_pb2.ModelConfig:
    """Returns the loaded Triton ModelConfig."""
    return self._triton_config

  @property
  def output_names(self) -> list[str]:
    """Returns the output tensor names parsed from Triton config."""
    return self._output_names

  def wait_for_model_ready(self) -> bool:
    """Polls OIP ModelReady until Triton loads the pose estimation model."""
    if self._ml_service_stub is None:
      raise ValueError("ML service stub is None, cannot check model readiness.")

    model_name = self._model_id
    start_time = time.time()

    # loop to check if model is ready
    while time.time() - start_time < _CONNECT_INFERENCE_SERVICE_TIMEOUT_SECONDS:
      model_ready_request = open_inference_grpc_pb2.ModelReadyRequest(
          name=model_name
      )
      try:
        response = self._ml_service_stub.ModelReady(model_ready_request)
        if response.ready:
          logging.info("Pose estimation model %s is ready.", model_name)
          return True
      except grpc.RpcError as e:
        logging.debug("Error checking pose estimation model readiness: %s", e)
      time.sleep(_CONNECT_INFERENCE_SERVICE_POLLING_INTERVAL_SECONDS)
    logging.warning(
        "Pose estimation model %s did not become ready within %d seconds.",
        model_name,
        _CONNECT_INFERENCE_SERVICE_TIMEOUT_SECONDS,
    )
    return False

  def run_inference(
      self,
      rgb: np.ndarray,
      depth: np.ndarray,
      mask: np.ndarray,
      intrinsic_matrix: np.ndarray,
      cad_bytes: bytes,
      num_iterations: int = 3,
      batch_size: Optional[int] = None,
      return_vis: bool = False,
  ) -> (
      tuple[np.ndarray, np.ndarray, np.ndarray]
      | tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
  ):
    """Runs FoundationPose inference via OIP.

    Args:
      rgb: RGB image of shape (H, W, 3) and uint8 dtype.
      depth: Depth map in meters of shape (H, W) and float32 dtype.
      mask: Binary segmentation mask(s) of shape (N, H, W) and uint8
        dtype, where N is the number of detected object instances.
      intrinsic_matrix: Camera intrinsic matrix K of shape (3, 3) and float32
        dtype.
      cad_bytes: Raw byte contents of the target object's 3D CAD mesh file
        (e.g., OBJ format).
      num_iterations: Number of FoundationPose refinement iterations to perform
        (defaults to 3).
      batch_size: Optional batch size for candidate pose processing in
        FoundationPose. If None, uses model default.
      return_vis: If True, returns an annotated OpenCV visualization image as
        the 4th tuple element.

    Returns:
      A tuple of (rotations, translations, confidences) or
      (rotations, translations, confidences, vis) if return_vis is True:
        - rotations: Estimated 3x3 rotation matrices of shape (N, 3, 3) and
          float32 dtype representing the rotation of each object instance in
          the camera frame.
        - translations: Estimated 3D translation vectors [x, y, z] of shape
          (N, 3) and float32 dtype in meters in the camera frame.
        - confidences: Pose score / confidence logit values of shape (N, 1) and
          float32 dtype (higher/closer to 0 is better).
        - vis: (Optional) Annotated image of shape (H, W, 3) and uint8 dtype.

    Raises:
      ValueError: If the ML service stub is None or cad_bytes is None.
    """
    if self._ml_service_stub is None:
      raise ValueError(
          "ML service stub is None, cannot run pose estimation inference."
      )
    if cad_bytes is None:
      raise ValueError("cad_bytes is None, cannot run pose estimation.")

    model_name = self._model_id
    logging.info("Checking if %s is ready...", model_name)
    self.wait_for_model_ready()

    rgb_np = np.ascontiguousarray(rgb, dtype=np.uint8)
    depth_np = np.ascontiguousarray(depth, dtype=np.float32)
    mask_np = np.ascontiguousarray(mask, dtype=np.uint8)
    if mask_np.ndim == 2:
      mask_np = np.expand_dims(mask_np, axis=0)
    intrinsic_matrix_np = np.ascontiguousarray(
        intrinsic_matrix, dtype=np.float32
    )
    cad_bytes_np = np.ascontiguousarray(
        np.frombuffer(cad_bytes, dtype=np.uint8)
    )
    num_iterations_np = np.array([num_iterations], dtype=np.int32)

    inputs_dict = {
        "MASK": mask_np,
        "RGB": rgb_np,
        "DEPTH": depth_np,
        "CAM_K": intrinsic_matrix_np,
        "CAD_MODEL_BYTES": cad_bytes_np,
        "NUM_ITERATIONS": num_iterations_np,
    }
    if batch_size is not None:
      inputs_dict["BATCH_SIZE"] = np.array([batch_size], dtype=np.int32)

    inference_request = _create_model_infer_request(
        inputs_dict,
        model_name=self._model_id,
        output_names=self._output_names,
    )

    logging.info("Running pose estimator inference...")
    start_time = time.perf_counter()
    response = self._ml_service_stub.ModelInfer(inference_request)
    inference_duration = time.perf_counter() - start_time
    logging.info(
        "Pose estimator inference completed in %.3f seconds.",
        inference_duration,
    )

    rotations = oip_utils.extract_np_tensor_from_oip_response(
        "ROTATION", response
    )
    translations = oip_utils.extract_np_tensor_from_oip_response(
        "TRANSLATION", response
    )
    confidences = oip_utils.extract_np_tensor_from_oip_response(
        "CONFIDENCE", response
    )

    if return_vis:
      vis = self._visualize_predictions(
          rgb,
          rotations,
          translations,
          confidences,
          intrinsic_matrix,
          cad_bytes=cad_bytes,
      )
      return rotations, translations, confidences, vis
    else:
      return rotations, translations, confidences

  def _visualize_predictions(
      self,
      img: np.ndarray,
      rotations: np.ndarray,
      translations: np.ndarray,
      confidences: np.ndarray,
      intrinsic_matrix: np.ndarray,
      cad_bytes: Optional[bytes] = None,
  ) -> np.ndarray:
    """Visualizes 6D pose predictions by drawing projected 3D object contours.

    Args:
      img: Input RGB image numpy array of shape (H, W, 3) or (1, 3, H, W).
      rotations: Rotation matrices of shape (N, 3, 3).
      translations: Translation vectors of shape (N, 3).
      confidences: Confidence scores of shape (N,) or (N, 1).
      intrinsic_matrix: Camera intrinsic 3x3 matrix.
      cad_bytes: Optional raw Wavefront OBJ bytes of the CAD model.

    Returns:
      A numpy array (H, W, 3) uint8 with overlaid object contours.
    """
    img_draw = img.copy()
    if img_draw.ndim == 4:
      img_draw = np.squeeze(img_draw, axis=0)
    if (
        img_draw.ndim == 3
        and img_draw.shape[0] in (1, 3, 4)
        and img_draw.shape[2] not in (1, 3, 4)
    ):
      img_draw = np.transpose(img_draw, (1, 2, 0))
    img_draw = np.ascontiguousarray(img_draw, dtype=np.uint8)

    if len(rotations) == 0:
      return img_draw

    mesh = None
    if cad_bytes:
      cache_key = hash(cad_bytes)
      if hasattr(self, "_mesh_cache") and cache_key in self._mesh_cache:
        mesh = self._mesh_cache[cache_key]
        if isinstance(self._mesh_cache, collections.OrderedDict):
          self._mesh_cache.move_to_end(cache_key)
      else:
        try:
          mesh = trimesh.load(
              io.BytesIO(cad_bytes), file_type="obj", force="mesh"
          )
          if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
          if hasattr(self, "_mesh_cache"):
            self._mesh_cache[cache_key] = mesh
            if isinstance(self._mesh_cache, collections.OrderedDict):
              self._mesh_cache.move_to_end(cache_key)
              while len(self._mesh_cache) > _MAX_MESH_CACHE_SIZE:
                self._mesh_cache.popitem(last=False)
            elif len(self._mesh_cache) > _MAX_MESH_CACHE_SIZE:
              self._mesh_cache.pop(next(iter(self._mesh_cache)))
        except Exception as e:
          logging.debug(
              "Could not load CAD mesh for contour visualization: %s", e
          )

    h, w = img_draw.shape[:2]

    for idx in range(len(rotations)):
      r_mat = np.array(rotations[idx], dtype=np.float32).reshape(3, 3)
      t_vec = np.array(translations[idx], dtype=np.float32).flatten()
      conf_val = (
          float(confidences[idx].item()) if idx < len(confidences) else 0.0
      )

      contours = None
      if (
          mesh is not None
          and hasattr(mesh, "vertices")
          and len(mesh.vertices) > 0
      ):
        pts_cam = (r_mat @ mesh.vertices.T + t_vec[:, None]).T  # (V, 3)
        valid_z = pts_cam[:, 2] > 1e-4

        if np.any(valid_z):
          proj = (intrinsic_matrix @ pts_cam.T).T
          with np.errstate(divide="ignore", invalid="ignore"):
            u = np.where(valid_z, proj[:, 0] / proj[:, 2], 0.0)
            v = np.where(valid_z, proj[:, 1] / proj[:, 2], 0.0)
          u = np.clip(np.round(u), -10000, 10000)
          v = np.clip(np.round(v), -10000, 10000)
          pts_2d = np.column_stack([u, v]).astype(np.int32)

          if hasattr(mesh, "faces") and len(mesh.faces) > 0:
            valid_faces = valid_z[mesh.faces].all(axis=1)
            if np.any(valid_faces):
              faces_to_render = mesh.faces[valid_faces]
              triangles = pts_2d[faces_to_render]

              poly_mask = np.zeros((h, w), dtype=np.uint8)
              cv2.fillPoly(poly_mask, list(triangles), 255)

              contours, _ = cv2.findContours(
                  poly_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
              )
              cv2.drawContours(img_draw, contours, -1, (0, 255, 0), 2)
          else:
            valid_pts_2d = pts_2d[valid_z]
            if len(valid_pts_2d) >= 3:
              hull = cv2.convexHull(valid_pts_2d)
              cv2.polylines(
                  img_draw,
                  [hull],
                  isClosed=True,
                  color=(0, 255, 0),
                  thickness=2,
              )
              contours = [hull]

      # Draw label
      label = f"Pose {idx}: {conf_val:.2f}"
      font = cv2.FONT_HERSHEY_SIMPLEX
      (text_w, text_h), _ = cv2.getTextSize(label, font, 0.4, 1)

      if contours and len(contours) > 0:
        all_pts = np.vstack(contours)
        lx = int(np.min(all_pts[:, 0, 0]))
        ly = int(np.min(all_pts[:, 0, 1]))
        lx = max(0, min(lx, w - text_w))
        ly = max(text_h + 4, min(ly, h - 5))
      else:
        lx, ly = 20, 30 + idx * 25

      cv2.rectangle(
          img_draw,
          (lx, ly - text_h - 4),
          (lx + text_w, ly),
          (0, 255, 0),
          -1,
      )
      cv2.putText(
          img_draw,
          label,
          (lx, max(ly - 5, 0)),
          font,
          0.4,
          (0, 0, 0),
          1,
          cv2.LINE_AA,
      )

    return img_draw
