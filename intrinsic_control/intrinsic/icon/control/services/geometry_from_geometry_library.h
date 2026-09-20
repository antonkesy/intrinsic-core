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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_GEOMETRY_FROM_GEOMETRY_LIBRARY_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_GEOMETRY_FROM_GEOMETRY_LIBRARY_H_

#include <memory>

#include "absl/base/nullability.h"
#include "absl/status/statusor.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/icon/control/services/geometry_library_service.h"
#include "intrinsic/icon/control/services/service.h"

namespace intrinsic::icon {

class GeometryFromGeometryLibrary final
    : public ServiceImplBase<GeometryFromGeometryLibrary,
                             GeometryLibraryService> {
 public:
  static absl::StatusOr<std::unique_ptr<GeometryFromGeometryLibrary>> Create(
      absl_nonnull
      std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
          geometry_service_stub);

  const GeometryLibrary* absl_nonnull GetGeometryLibrary() const override;

 private:
  GeometryFromGeometryLibrary() = default;

  std::shared_ptr<GeometryLibrary> geometry_library_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_GEOMETRY_FROM_GEOMETRY_LIBRARY_H_
