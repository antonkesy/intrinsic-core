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

#include "intrinsic/geometry/api/mesh_stats.h"

#include "Eigen/Core"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/axis_aligned_bounding_box_3d.h"
#include "intrinsic/geometry/internal/kernel_3/exact_predicates_inexact_constructions_kernel.h"
#include "intrinsic/math/proto/point.pb.h"
#include "intrinsic/math/proto/vector3.pb.h"
#include "intrinsic/scene/proto/v1/geometry_stats.pb.h"

namespace intrinsic::geo {
absl::StatusOr<intrinsic_proto::scene_object::v1::GeometryStats> ToProto(
    const MeshStats& mesh_stats) {
  intrinsic_proto::scene_object::v1::GeometryStats proto;
  proto.set_num_vertices(mesh_stats.num_vertices);
  proto.set_num_triangles(mesh_stats.num_triangles);
  proto.set_closed(mesh_stats.closed);
  proto.set_contains_primitives(mesh_stats.contains_primitives);
  proto.set_volume(mesh_stats.volume);

  if (!mesh_stats.aabb.IsEmpty()) {
    proto.mutable_aabb_center()->set_x(mesh_stats.aabb.GetCenter().x());
    proto.mutable_aabb_center()->set_y(mesh_stats.aabb.GetCenter().y());
    proto.mutable_aabb_center()->set_z(mesh_stats.aabb.GetCenter().z());

    proto.mutable_aabb_extent()->set_x(mesh_stats.aabb.GetDiagonal().x());
    proto.mutable_aabb_extent()->set_y(mesh_stats.aabb.GetDiagonal().y());
    proto.mutable_aabb_extent()->set_z(mesh_stats.aabb.GetDiagonal().z());
  }

  proto.mutable_vertex_centroid()->set_x(mesh_stats.vertex_centroid.x());
  proto.mutable_vertex_centroid()->set_y(mesh_stats.vertex_centroid.y());
  proto.mutable_vertex_centroid()->set_z(mesh_stats.vertex_centroid.z());

  proto.mutable_volumetric_centroid()->set_x(
      mesh_stats.volumetric_centroid.x());
  proto.mutable_volumetric_centroid()->set_y(
      mesh_stats.volumetric_centroid.y());
  proto.mutable_volumetric_centroid()->set_z(
      mesh_stats.volumetric_centroid.z());
  proto.set_min_circumradius(mesh_stats.min_circumradius);
  return proto;
}

absl::StatusOr<MeshStats> FromProto(
    const intrinsic_proto::scene_object::v1::GeometryStats& proto) {
  MeshStats mesh_stats;
  mesh_stats.num_vertices = proto.num_vertices();
  mesh_stats.num_triangles = proto.num_triangles();
  mesh_stats.closed = proto.closed();
  mesh_stats.contains_primitives = proto.contains_primitives();
  mesh_stats.volume = proto.volume();
  mesh_stats.min_circumradius = proto.min_circumradius();

  if (proto.has_aabb_center() && proto.has_aabb_extent()) {
    EpicPoint3 aabb_center =
        EpicPoint3(proto.aabb_center().x(), proto.aabb_center().y(),
                   proto.aabb_center().z());
    eigenmath::Vector3d aabb_half_lengths =
        eigenmath::Vector3d(proto.aabb_extent().x(), proto.aabb_extent().y(),
                            proto.aabb_extent().z()) /
        2.0;

    eigenmath::Vector3d center_vec(aabb_center.x(), aabb_center.y(),
                                   aabb_center.z());
    eigenmath::Vector3d min_vec = center_vec - aabb_half_lengths;
    eigenmath::Vector3d max_vec = center_vec + aabb_half_lengths;

    EpicPoint3 aabb_min = EpicPoint3(min_vec.x(), min_vec.y(), min_vec.z());
    EpicPoint3 aabb_max = EpicPoint3(max_vec.x(), max_vec.y(), max_vec.z());
    mesh_stats.aabb = AxisAlignedBoundingBox3d(aabb_min, aabb_max);
  }

  mesh_stats.vertex_centroid =
      EpicPoint3(proto.vertex_centroid().x(), proto.vertex_centroid().y(),
                 proto.vertex_centroid().z());
  mesh_stats.volumetric_centroid = EpicPoint3(proto.volumetric_centroid().x(),
                                              proto.volumetric_centroid().y(),
                                              proto.volumetric_centroid().z());
  mesh_stats.min_circumradius = proto.min_circumradius();
  return mesh_stats;
}

}  // namespace intrinsic::geo
