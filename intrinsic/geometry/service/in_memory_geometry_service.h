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

#ifndef INTRINSIC_GEOMETRY_SERVICE_IN_MEMORY_GEOMETRY_SERVICE_H_
#define INTRINSIC_GEOMETRY_SERVICE_IN_MEMORY_GEOMETRY_SERVICE_H_

#include <memory>

#include "intrinsic/geometry/service/geometry_service_impl.h"
#include "intrinsic/geometry/storage/in_memory_storage.h"
#include "intrinsic/longrunning/cc/operations_proxy.h"

namespace intrinsic::geo {
// An in memory geometry service. This service is backed by a hash table.
class InMemoryGeometryService : public GeometryServiceImpl {
 public:
  InMemoryGeometryService();

  std::shared_ptr<longrunning::OperationsProxy> GetOperationsProxy();

 private:
  MapGeometryLibrary geometry_storage_;
};

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::InMemoryGeometryService;
}  // namespace intrinsic

#endif  // INTRINSIC_GEOMETRY_SERVICE_IN_MEMORY_GEOMETRY_SERVICE_H_
