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

#ifndef INTRINSIC_GEOMETRY_STORAGE_GEOMETRY_SERVICE_STORAGE_H_
#define INTRINSIC_GEOMETRY_STORAGE_GEOMETRY_SERVICE_STORAGE_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "grpcpp/server_context.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/storage/geo_cache.h"
#include "intrinsic/geometry/storage/geometry_library.h"

namespace intrinsic::geo {
// Returns a GeometryLibrary instance that is backed by the geometry
// service.
//
// The returned GeometryLibrary stores a reference to the `context`, and
// must not outlive it, in particular a new GeometryLibrary should be created
// for each incoming RPC.
//
// The only exception is when `context` is a nullptr, in which case the call
// metadata will not be propagated. Amongst other call propagation issues (eg.
// call cancellation), this will cause any downstream auth checks to fail.
//
// The returned GeometryLibrary also stores a reference to the `geometry_cache`,
// and must not outlive it. The `geometry_cache` is used to store references to
// geometry that has already been uploaded to the geometry service and can be
// used to avoid repeat uploads across calls to GeometryLibraries.
std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
        stub,
    GeometryFingerprintCacheInterface& geometry_cache,
    grpc::ServerContext* context = nullptr);

// Same as above, but using a non-owned stub pointer.
std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    intrinsic_proto::geometry::GeometryService::StubInterface* stub,
    GeometryFingerprintCacheInterface& geometry_cache,
    grpc::ServerContext* context = nullptr);

// Same as above, but creates a stub for the given `grpc_target` address itself.
std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    absl::string_view grpc_target,
    GeometryFingerprintCacheInterface& geometry_cache,
    absl::Duration timeout = connect::kGrpcClientConnectDefaultTimeout,
    grpc::ServerContext* context = nullptr);

// Same as above, but does not cache across library instances.
std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
        stub,
    grpc::ServerContext* context = nullptr);

std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    intrinsic_proto::geometry::GeometryService::StubInterface* stub,
    grpc::ServerContext* context = nullptr);

std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    absl::string_view grpc_target,
    absl::Duration timeout = connect::kGrpcClientConnectDefaultTimeout,
    grpc::ServerContext* context = nullptr);

std::unique_ptr<GeometryLibrary> GetGeometryServiceGeometryLibrary(
    intrinsic_proto::geometry::GeometryService::StubInterface* stub,
    int cache_size);

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::GetGeometryServiceGeometryLibrary;
}  // namespace intrinsic

#endif  // INTRINSIC_GEOMETRY_STORAGE_GEOMETRY_SERVICE_STORAGE_H_
