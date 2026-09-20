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

#ifndef INTRINSIC_GEOMETRY_API_FILE_IO_H_
#define INTRINSIC_GEOMETRY_API_FILE_IO_H_

#include <string>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/geometry.h"
#include "intrinsic/geometry/proto/v1/exact_geometry.pb.h"
#include "intrinsic/geometry/proto/v1/geometric_transform.pb.h"
#include "intrinsic/geometry/proto/v1/geometry.pb.h"
#include "intrinsic/geometry/proto/v1/geometry_storage_refs.pb.h"
#include "intrinsic/geometry/proto/v1/inline_geometry.pb.h"
#include "intrinsic/geometry/proto/v1/primitive_shape.pb.h"
#include "intrinsic/geometry/proto/v1/primitives.pb.h"
#include "intrinsic/geometry/proto/v1/renderable.pb.h"
#include "intrinsic/geometry/proto/v1/transformed_geometry.pb.h"
#include "intrinsic/geometry/proto/v1/transformed_primitive_shape.pb.h"
#include "intrinsic/geometry/proto/v1/transformed_primitive_shape_set.pb.h"

namespace intrinsic::geo {
// Returns a Geometry object after loading a pts file containing a point cloud
// at `filename`.
absl::StatusOr<Geometry> LoadPointsFileToGeometry(
    absl::string_view filename,
    const eigenmath::Vector3d& scale = eigenmath::Vector3d::Ones());

// Returns a Geometry object after loading a pts file containing a point cloud.
absl::StatusOr<Geometry> LoadPointsBufferToGeometry(
    const std::string& file_content, const std::string& extension,
    const eigenmath::Vector3d& scale = eigenmath::Vector3d::Ones());

// Returns a Geometry object after loading a obj/dae file.
absl::StatusOr<Geometry> LoadMeshFileToGeometry(
    absl::string_view filename,
    const eigenmath::Vector3d& scale = eigenmath::Vector3d::Ones());

// Returns a Geometry object after parsing the obj/dae file contents
absl::StatusOr<Geometry> LoadMeshBufferToGeometry(
    const std::string& file_content, const std::string& extension,
    const eigenmath::Vector3d& scale = eigenmath::Vector3d::Ones());

// Returns the supported set of mesh file extensions for the Intrinsic platform.
const absl::flat_hash_set<std::string>& SupportedMeshExtensions();

// Returns the supported set of point cloud file extensions for the Intrinsic
// platform.
const absl::flat_hash_set<std::string>& SupportedPointsExtensions();

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::LoadMeshBufferToGeometry;
using ::intrinsic::geo::LoadMeshFileToGeometry;
using ::intrinsic::geo::LoadPointsBufferToGeometry;
using ::intrinsic::geo::LoadPointsFileToGeometry;
}  // namespace intrinsic
#endif  // INTRINSIC_GEOMETRY_API_FILE_IO_H_
