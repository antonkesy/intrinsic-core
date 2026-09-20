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

"""Utils for retrieving inline geometry from geometry proto."""

import grpc
import numpy as np

from intrinsic.geometry.api.python import io
from intrinsic.geometry.proto import geometry_service_pb2
from intrinsic.geometry.proto import geometry_service_pb2_grpc
from intrinsic.geometry.proto.v1 import exact_geometry_pb2
from intrinsic.geometry.proto.v1 import geometric_transform_pb2
from intrinsic.geometry.proto.v1 import geometry_pb2
from intrinsic.geometry.proto.v1 import inline_geometry_pb2
from intrinsic.geometry.proto.v1 import renderable_pb2
from intrinsic.geometry.proto.v1 import triangle_mesh_pb2
from intrinsic.math.python import data_types
from intrinsic.math.python import proto_conversion
from intrinsic.storage.content_addressable_storage.proto import cas_service_pb2_grpc
from intrinsic.storage.content_addressable_storage.python import client_helpers


def get_inline_geometry_from_geometry(
    geometry: geometry_pb2.Geometry,
    geometry_service_stub: geometry_service_pb2_grpc.GeometryServiceStub,
) -> inline_geometry_pb2.InlineGeometry:
  """Returns the inline geometry for the given geometry.

  A geometry contains either inline geometry data or geometry storage refs, this
  function returns the inline geometry data by fetching the inline geometry from
  the given geometry service.

  Args:
    geometry: The geometry to get the inline geometry for.
    geometry_service_stub: The geometry service stub to use to get the geometry
      from the geometry service.

  Returns:
    The inline geometry for the given geometry.

  Raises:
    ValueError:
      - If the geometry does not have inline geometry data or geometry
      storage refs.
      - If the geometry service fails to get the geometry.
  """
  if geometry.HasField("inline_geometry_data"):
    return geometry.inline_geometry_data
  elif geometry.HasField("geo_ref"):
    try:
      geometry_with_metadata = geometry_service_stub.GetGeometry(
          geometry_service_pb2.GetGeometryRequest(
              geometry_storage_refs=geometry.geo_ref
          )
      )
      if not geometry_with_metadata.HasField("inline_geometry"):
        raise ValueError(
            "Geometry service returned no inline geometry for refs"
            f" {geometry.geo_ref}"
        )
      return geometry_with_metadata.inline_geometry
    except grpc.RpcError as e:
      raise ValueError(
          "Failed to get geometry from geometry service for refs"
          f" {geometry.geo_ref}"
      ) from e
  else:
    raise ValueError(
        "Geometry does not have inline geometry data or geometry storage refs."
    )


def geometric_transform_to_ndarray(
    geometric_transform: geometric_transform_pb2.GeometricTransform,
) -> np.ndarray:
  """Converts a geometric transform to a numpy array.

  Args:
    geometric_transform: The geometric transform to convert to a numpy array.

  Returns:
    The geometric transform as a numpy array.

  Raises:
    ValueError:
      - If the geometric transform fails to convert to an ndarray.
  """
  try:
    return proto_conversion.ndarray_from_matrix_proto(
        io.geometric_transform_to_matrix(geometric_transform)
    )
  except Exception as e:
    raise ValueError(
        "Failed to convert geometric transform to ndarray"
        f" {geometric_transform}"
    ) from e


def geometric_transform_from_ndarray(
    ndarray: np.ndarray,
) -> geometric_transform_pb2.GeometricTransform:
  """Converts a numpy array to a geometric transform."""
  if ndarray.shape != (4, 4):
    raise ValueError(f"expected a 4x4 array, got shape {ndarray.shape}.")

  return geometric_transform_pb2.GeometricTransform(
      matrix4d=proto_conversion.ndarray_to_matrix_proto(ndarray)
  )


def geometric_transform_from_pose3(
    pose3: data_types.Pose3,
) -> geometric_transform_pb2.GeometricTransform:
  """Converts a pose3 to a geometric transform."""
  return geometric_transform_pb2.GeometricTransform(
      matrix4d=proto_conversion.ndarray_to_matrix_proto(pose3.matrix4x4())
  )


def triangle_mesh_from_geometry(
    geometry: (
        inline_geometry_pb2.InlineGeometry | exact_geometry_pb2.ExactGeometry
    ),
) -> triangle_mesh_pb2.TriangleMesh:
  """Converts a geometry containing an exact geometry to a triangle mesh.

  Args:
    geometry: The geometry proto that contains a exact geometry to convert to a
      triangle mesh.

  Returns:
    The triangle mesh proto.

  Raises:
    ValueError:
      - If the geometry doesn't contain an exact geometry.
      - If the exact geometry fails to convert to a triangle mesh.
  """
  if isinstance(geometry, inline_geometry_pb2.InlineGeometry):
    exact_geometry = geometry.exact_geometry
  elif isinstance(geometry, exact_geometry_pb2.ExactGeometry):
    exact_geometry = geometry
  else:
    raise ValueError(
        "Geometry must be an InlineGeometry or ExactGeometry, got"
        f" {type(geometry)}"
    )
  try:
    return io.exact_geometry_to_triangle_mesh(exact_geometry)
  except Exception as e:
    raise ValueError(
        f"Failed to convert exact geometry to triangle mesh {exact_geometry}"
    ) from e


def get_renderable_from_geometry(
    geometry: geometry_pb2.Geometry,
    storage: (
        geometry_service_pb2_grpc.GeometryServiceStub
        | cas_service_pb2_grpc.ContentAddressableStorageServiceStub
        | None
    ),
    grpc_metadata: list[tuple[str, str]] | None = None,
) -> renderable_pb2.Renderable:
  """Returns the renderable for the given geometry."""
  if geometry.HasField("inline_geometry_data"):
    if geometry.inline_geometry_data.HasField("renderable"):
      raw_renderable = geometry.inline_geometry_data.renderable
    else:
      raw_renderable = geometry.inline_geometry_data.generated_renderable
  elif geometry.HasField("geo_ref"):
    if isinstance(storage, geometry_service_pb2_grpc.GeometryServiceStub):
      try:
        geometry_with_metadata = storage.GetRenderable(
            geometry_service_pb2.GetRenderableRequest(
                geometry_storage_refs=geometry.geo_ref
            ),
            metadata=grpc_metadata,
        )
        if not geometry_with_metadata.HasField("renderable"):
          raise ValueError(
              "Geometry service returned no renderable for refs"
              f" {geometry.geo_ref}"
          )
      except grpc.RpcError as e:
        raise ValueError(
            "Failed to get renderable from geometry service for refs"
            f" {geometry.geo_ref}"
        ) from e
      raw_renderable = geometry_with_metadata.renderable
    elif isinstance(
        storage, cas_service_pb2_grpc.ContentAddressableStorageServiceStub
    ):
      try:
        renderable_data = client_helpers.get(
            storage,
            geometry.geo_ref.renderable_ref,
            grpc_metadata=grpc_metadata,
        )
      except Exception as e:
        raise ValueError(
            f"Failed to get renderable from CAS for refs {geometry.geo_ref}"
        ) from e
      raw_renderable = renderable_pb2.Renderable(glb_bytes=renderable_data)
    else:
      raise ValueError(
          "Requires a geometry service stub or CAS stub to retrieve renderable"
          f" from {geometry.geo_ref}, got storage {storage}"
      )
  else:
    raise ValueError(
        "Geometry does not have inline geometry data or geometry storage refs."
    )

  if geometry.HasField("material_overrides"):
    return io.apply_material_properties(
        raw_renderable, geometry.material_overrides
    )
  else:
    return raw_renderable
