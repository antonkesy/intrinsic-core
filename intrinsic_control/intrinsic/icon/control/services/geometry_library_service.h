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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_GEOMETRY_LIBRARY_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_GEOMETRY_LIBRARY_SERVICE_H_

#include "absl/base/nullability.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace intrinsic::icon {

// GeometryLibraryService provides (read) access to a geometry library to other
// icon components. This serves as a central entry points for geometry access.
class GeometryLibraryService {
 public:
  virtual ~GeometryLibraryService() = default;

  virtual const GeometryLibrary* absl_nonnull GetGeometryLibrary() const
      INTRINSIC_NON_REALTIME_ONLY = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_GEOMETRY_LIBRARY_SERVICE_H_
