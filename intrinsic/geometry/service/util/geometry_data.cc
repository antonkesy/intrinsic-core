// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/geometry/service/util/geometry_data.h"

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/api/file_io.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/api/io.h"
#include "intrinsic/geometry/compatibility/io.h"
#include "intrinsic/geometry/proto/geometry_service.pb.h"
#include "intrinsic/geometry/proto/geometry_service_types.pb.h"
#include "intrinsic/geometry/proto/geometry_storage_refs.pb.h"
#include "intrinsic/geometry/proto/lazy_exact_geometry.pb.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::geo {
namespace {
absl::StatusOr<Geometry> ToGeometry(
    const intrinsic_proto::geometry::v1::TransformedPrimitiveShapeSet&
        primitive_shape_set_proto) {
  INTR_ASSIGN_OR_RETURN(
      std::vector<TransformedPrimitiveShapePtr> primitive_shapes,
      geometry_details::ToPrimitiveSet(primitive_shape_set_proto));
  INTR_ASSIGN_OR_RETURN(ExactGeometry exact_geometry,
                        ExactGeometry::Create(std::move(primitive_shapes)));
  return Geometry(exact_geometry, std::nullopt);
}
}  // namespace

absl::StatusOr<Geometry> ToGeometry(
    const intrinsic_proto::geometry::GeometryData& data) {
  switch (data.data_case()) {
    case intrinsic_proto::geometry::GeometryData::kGeometryV0:
      return geometry_compatibility::ToGeometry(data.geometry_v0());
    case intrinsic_proto::geometry::GeometryData::kInlineGeometry:
      return ToGeometry(data.inline_geometry());
    case intrinsic_proto::geometry::GeometryData::kObjData:
      return LoadMeshBufferToGeometry(data.obj_data(), "obj");
    case intrinsic_proto::geometry::GeometryData::kStlBytes:
      return LoadMeshBufferToGeometry(data.stl_bytes(), "stl");
    case intrinsic_proto::geometry::GeometryData::kGltfBytes:
      return LoadMeshBufferToGeometry(data.gltf_bytes(), "glb");
    case intrinsic_proto::geometry::GeometryData::kPtsBytes:
      return LoadPointsBufferToGeometry(data.pts_bytes(), "pts");
    case intrinsic_proto::geometry::GeometryData::kPrimitiveSetV0:
      return geometry_compatibility::ToGeometry(
          data.primitive_set_v0().primitives());
    case intrinsic_proto::geometry::GeometryData::kPrimitiveSet:
      return ToGeometry(data.primitive_set());
    case intrinsic_proto::geometry::GeometryData::DATA_NOT_SET:
      return absl::InvalidArgumentError("No geometry data set");
  }
}

}  // namespace intrinsic::geo
