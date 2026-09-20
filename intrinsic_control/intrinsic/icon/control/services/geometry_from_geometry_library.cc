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

#include "intrinsic/icon/control/services/geometry_from_geometry_library.h"

#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/geometry/storage/geometry_service_storage.h"

namespace intrinsic::icon {

absl::StatusOr<std::unique_ptr<GeometryFromGeometryLibrary>>
GeometryFromGeometryLibrary::Create(
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
        geometry_service_stub) {
  auto service = absl::WrapUnique(new GeometryFromGeometryLibrary());
  service->geometry_library_ =
      GetGeometryServiceGeometryLibrary(std::move(geometry_service_stub));
  return service;
}

const GeometryLibrary* absl_nonnull
GeometryFromGeometryLibrary::GetGeometryLibrary() const {
  return geometry_library_.get();
}

}  // namespace intrinsic::icon
