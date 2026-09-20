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

#include "intrinsic/geometry/api/io.h"

#include <pybind11/pybind11.h>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/apply_material_properties.h"
#include "intrinsic/geometry/api/exact_geometry.h"
#include "intrinsic/geometry/compatibility/io.h"
#include "intrinsic/geometry/internal/mesh/mesh.h"
#include "intrinsic/geometry/proto/v1/exact_geometry.pb.h"
#include "intrinsic/geometry/proto/v1/transformed_geometry.pb.h"
#include "intrinsic/math/proto/matrix.pb.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/status_macros.h"
#include "pybind11/cast.h"
#include "pybind11/detail/common.h"
#include "pybind11/pybind11.h"
#include "pybind11/pytypes.h"
#include "pybind11_abseil/absl_casters.h"
#include "pybind11_abseil/status_casters.h"
#include "pybind11_protobuf/native_proto_caster.h"

namespace intrinsic::geo {
namespace {
absl::StatusOr<intrinsic_proto::Matrixd> ToMatrixProto(
    const intrinsic_proto::geometry::v1::GeometricTransform&
        geometric_transform) {
  INTR_ASSIGN_OR_RETURN(eigenmath::Matrix4d matrix,
                        ToAffineTransform(geometric_transform));
  return ::intrinsic::ToProto(matrix);
}

absl::StatusOr<intrinsic_proto::geometry::v1::TriangleMesh>
ExactGeometryToTriangleMeshProto(
    const intrinsic_proto::geometry::v1::ExactGeometry exact_geometry_proto) {
  INTR_ASSIGN_OR_RETURN(ExactGeometry exact_geometry,
                        ToGeometry(exact_geometry_proto));
  if (!exact_geometry.HasMesh()) {
    return absl::InvalidArgumentError(
        "Unable to convert ExactGeometry to TriangleMesh proto.");
  }
  INTR_ASSIGN_OR_RETURN(
      auto mesh_proto, ToTriangleMeshProtoV1(exact_geometry.GetMesh()->Value()),
      _ << "Unable to convert Mesh to TriangleMesh proto.");
  return mesh_proto;
}

absl::StatusOr<intrinsic_proto::geometry::v1::Renderable>
ApplyMaterialPropertiesToProto(
    const intrinsic_proto::geometry::v1::Renderable& renderable,
    const intrinsic_proto::geometry::v1::MaterialProperties&
        material_properties) {
  INTR_ASSIGN_OR_RETURN(std::string glb_bytes,
                        ApplyMaterialPropertiesToGlb(renderable.glb_bytes(),
                                                     material_properties));
  intrinsic_proto::geometry::v1::Renderable result;
  result.set_glb_bytes(std::move(glb_bytes));
  return result;
}

}  // namespace

namespace py = pybind11;

PYBIND11_MODULE(io, m) {
  pybind11_protobuf::ImportNativeProtoCasters();
  pybind11::google::ImportStatusModule();

  m.doc() = "Python bindings for geometry proto io.";
  m.def("geometric_transform_to_matrix", &ToMatrixProto,
        py::arg("geometric_transform"));
  m.def("exact_geometry_to_triangle_mesh", &ExactGeometryToTriangleMeshProto,
        py::arg("exact_geometry_proto"));
  m.def("apply_material_properties", &ApplyMaterialPropertiesToProto,
        py::arg("renderable"), py::arg("material_properties"));
}
}  // namespace intrinsic::geo
