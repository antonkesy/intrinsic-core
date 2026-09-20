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

#ifndef INTRINSIC_GEOMETRY_API_MESH_STATS_H_
#define INTRINSIC_GEOMETRY_API_MESH_STATS_H_

#include "absl/status/statusor.h"
#include "intrinsic/geometry/api/axis_aligned_bounding_box_3d.h"
#include "intrinsic/geometry/internal/kernel_3/exact_predicates_inexact_constructions_kernel.h"
#include "intrinsic/marshal/riegeli_proto_coder.h"
#include "intrinsic/scene/proto/v1/geometry_stats.pb.h"

namespace intrinsic::geo {
struct MeshStats {
  EpicPoint3 vertex_centroid = EpicPoint3(0.0, 0.0, 0.0);
  EpicPoint3 volumetric_centroid = EpicPoint3(0.0, 0.0, 0.0);
  int num_vertices = 0;
  int num_triangles = 0;
  bool closed = true;
  bool contains_primitives = false;
  double volume = 0.0;
  AxisAlignedBoundingBox3d aabb = AxisAlignedBoundingBox3d();
  double min_circumradius = 0.0;
};

absl::StatusOr<intrinsic_proto::scene_object::v1::GeometryStats> ToProto(
    const MeshStats& mesh_stats);
absl::StatusOr<MeshStats> FromProto(
    const intrinsic_proto::scene_object::v1::GeometryStats& proto);

// TODO(b/461588498) Use these methods to serialize/deserialize MeshStats.
}  // namespace intrinsic::geo

REGISTER_RIEGELI_PROTO_CODER_EXPLICIT(
    intrinsic::geo::MeshStats, intrinsic_proto::scene_object::v1::GeometryStats,
    intrinsic::geo::ToProto, intrinsic::geo::FromProto);

namespace intrinsic {
using ::intrinsic::geo::FromProto;
using ::intrinsic::geo::ToProto;
}  // namespace intrinsic
#endif  // INTRINSIC_GEOMETRY_API_MESH_STATS_H_
