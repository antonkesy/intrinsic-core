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

#include "intrinsic/geometry/service/in_memory_geometry_service.h"

#include <memory>

#include "grpcpp/server_context.h"
#include "intrinsic/geometry/service/geometry_service_impl.h"
#include "intrinsic/geometry/storage/geo_cache.h"
#include "intrinsic/geometry/storage/proxy_storage.h"
#include "intrinsic/longrunning/cc/operation_scheduler.h"
#include "intrinsic/longrunning/cc/operations_proxy.h"

namespace intrinsic::geo {
std::shared_ptr<longrunning::OperationsProxy>
InMemoryGeometryService::GetOperationsProxy() {
  return scheduler_;
}

InMemoryGeometryService::InMemoryGeometryService()
    : GeometryServiceImpl(
          std::make_shared<longrunning::OperationScheduler>(),
          [&geo_storage = geometry_storage_](grpc::ServerContext* context,
                                             GeometryFingerprintCache& cache) {
            return std::make_unique<ProxyStorageLibrary>(geo_storage);
          }) {}

}  // namespace intrinsic::geo
