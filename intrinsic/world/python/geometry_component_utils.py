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

"""Utils for geometry components."""

from collections.abc import Mapping
from typing import Any
from typing import Union

from intrinsic.geometry.proto.v1 import exact_geometry_pb2
from intrinsic.geometry.proto.v1 import geometry_pb2
from intrinsic.geometry.proto.v1 import geometry_storage_refs_pb2
from intrinsic.geometry.proto.v1 import inline_geometry_pb2
from intrinsic.geometry.proto.v1 import primitive_shape_pb2
from intrinsic.geometry.proto.v1 import primitives_pb2
from intrinsic.geometry.proto.v1 import transformed_geometry_pb2
from intrinsic.geometry.proto.v1 import transformed_primitive_shape_pb2
from intrinsic.geometry.proto.v1 import transformed_primitive_shape_set_pb2
from intrinsic.world.proto import geometry_component_pb2
from intrinsic.world.python import geometry_types

GeometryLike = Union[
    transformed_geometry_pb2.TransformedGeometry,
    geometry_storage_refs_pb2.GeometryStorageRefs,
]

# Type aliases for geometry set-like objects.
GeometrySetLike = Union[
    geometry_component_pb2.GeometryComponent.GeometrySet,
    GeometryLike,
    Mapping[str, GeometryLike],
]


def create_geometry_component(
    geometries: Mapping[str, GeometrySetLike] | GeometrySetLike,
) -> geometry_component_pb2.GeometryComponent:
  """Creates a geometry component from a variation of geometry-like objects.

  This function attempts to make creating GeometryComponent protos from
  GeometryStorageRefs less verbose.

  Args:
    geometries: Either a mapping of named geometries to geometry set-like
      objects, or a single geometry set-like object to be used for both
      collision and visual geometry. Note an empty dict for GeometrySetLike will
      create named geometry with an empty geometry set. A mapping of string to a
      single GeometryLike object will be treated as a single GeometrySetLike.

  Returns:
    A geometry component with the given geometry storage refs.

  Raises:
    ValueError: If the input is not a supported type.
  """
  if isinstance(geometries, Mapping):
    if geometries:
      # For the case where a Mapping[str, GeometryLike] is passed in,
      # we treat this as a single GeometrySetLike.
      geometry_set_like = next(iter(geometries.values()))
      if not _is_geometry_like(geometry_set_like):
        return geometry_component_pb2.GeometryComponent(
            named_geometries={
                t: _to_geometry_component_geometry_set(g)
                for t, g in geometries.items()
            }
        )

  geometries = _to_geometry_component_geometry_set(geometries)

  return geometry_component_pb2.GeometryComponent(
      named_geometries={
          geometry_types.KIND_COLLISION_GEOMETRY: geometries,
          geometry_types.KIND_VISUAL_GEOMETRY: geometries,
      }
  )


def create_dummy_geometry_component():
  """Creates a dummy valid geometry component."""
  return create_geometry_component({
      "visual": geometry_component_pb2.GeometryComponent.GeometrySet(
          named_geometries={
              "sphere": transformed_geometry_pb2.TransformedGeometry(
                  geometry=geometry_pb2.Geometry(
                      inline_geometry_data=inline_geometry_pb2.InlineGeometry(
                          exact_geometry=exact_geometry_pb2.ExactGeometry(
                              primitive_set=transformed_primitive_shape_set_pb2.TransformedPrimitiveShapeSet(
                                  primitives=[
                                      transformed_primitive_shape_pb2.TransformedPrimitiveShape(
                                          shape=primitive_shape_pb2.PrimitiveShape(
                                              sphere=primitives_pb2.Sphere(
                                                  radius=1.0
                                              )
                                          )
                                      )
                                  ]
                              )
                          )
                      )
                  )
              )
          }
      )
  })


def get_geometries_v1(
    geometry_component: geometry_component_pb2.GeometryComponent,
    geometry_type: str,
) -> Mapping[str, transformed_geometry_pb2.TransformedGeometry]:
  """Returns the geometries from a geometry component.

  Args:
    geometry_component: The geometry component proto to get the geometries from.
    geometry_type: The type of geometry to get. Use
      geometry_types.KIND_VISUAL_GEOMETRY or
      geometry_types.KIND_COLLISION_GEOMETRY for the corresponding strings.

  Returns:
    A named geometry set of mapping of names to v1 TransformedGeometry protos.
  """

  if geometry_type not in geometry_component.named_geometries:
    raise ValueError(
        f"No {geometry_type} geometry found for geometry component"
    )

  geometry_set = geometry_component.named_geometries[geometry_type]
  return geometry_set.named_geometries


def _to_geometry_component_geometry_set(
    geometry_set_like: GeometrySetLike,
) -> geometry_component_pb2.GeometryComponent.GeometrySet:
  """Converts a geometry set-like object to a GeometryComponent.GeometrySet."""
  if isinstance(
      geometry_set_like, geometry_component_pb2.GeometryComponent.GeometrySet
  ):
    return geometry_set_like

  if isinstance(geometry_set_like, Mapping):
    return geometry_component_pb2.GeometryComponent.GeometrySet(
        named_geometries={
            t: _to_transformed_geometry_proto(g)
            for t, g in geometry_set_like.items()
        }
    )

  if _is_geometry_like(geometry_set_like):
    return geometry_component_pb2.GeometryComponent.GeometrySet(
        named_geometries={
            "0": _to_transformed_geometry_proto(geometry_set_like)
        }
    )

  raise ValueError(
      f"Unsupported geometry set type: {type(geometry_set_like)}, cannot"
      " convert to intrinsic_proto.world.GeometryComponent.GeometrySet"
  )


def _to_transformed_geometry_proto(
    geometry_like: GeometryLike,
) -> transformed_geometry_pb2.TransformedGeometry:
  """Converts a geometry-like object to a TransformedGeometry proto."""
  if isinstance(geometry_like, transformed_geometry_pb2.TransformedGeometry):
    return geometry_like
  elif isinstance(geometry_like, geometry_storage_refs_pb2.GeometryStorageRefs):
    return transformed_geometry_pb2.TransformedGeometry(
        geometry=geometry_pb2.Geometry(geo_ref=geometry_like)
    )
  else:
    raise ValueError(
        f"Unsupported geometry type: {type(geometry_like)}, cannot convert to"
        " intrinsic_proto.geometry.v1.TransformedGeometry"
    )


def _is_geometry_like(maybe_geometry_like: Any) -> bool:
  """Returns True if the given object is a GeometryLike."""
  return isinstance(
      maybe_geometry_like, transformed_geometry_pb2.TransformedGeometry
  ) or isinstance(
      maybe_geometry_like, geometry_storage_refs_pb2.GeometryStorageRefs
  )
