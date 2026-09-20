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

#ifndef INTRINSIC_GEOMETRY_STORAGE_CAS_STORAGE_H_
#define INTRINSIC_GEOMETRY_STORAGE_CAS_STORAGE_H_

#include <map>
#include <memory>

#include "grpcpp/server_context.h"
#include "grpcpp/support/string_ref.h"
#include "intrinsic/geometry/storage/geo_cache.h"
#include "intrinsic/geometry/storage/geometry_library.h"
#include "intrinsic/storage/content_addressable_storage/proto/cas_service.grpc.pb.h"

namespace intrinsic::geo {
// Returns a GeometryLibrary that is backed by content addressable storage
// (CAS).
//
// The CAS GeometryLibrary uses a `GeometryFingerprintCache` to maintain a map
// from geometry fingerprint to `GeometryStorageRef`. This is to avoid repeat
// hits to CAS for *uploading* geometry – if a piece of geometry was uploaded to
// CAS once, then we can simply consult the map and return the known
// `GeometryStorageRef`.
//
// The GeometryLibrary stores a reference to the `ServerContext`, and must not
// outlive it, in particular a new CAS GeometryLibrary should be created for
// each RPC.
//
// When called from the grpc context, the `context` argument configures the
// downstream call to CAS. `context` may also be omitted (pass nullptr) if this
// function is called directly from a local binary. In this case, the `metadata`
// argument will configure the call.
std::unique_ptr<GeometryLibrary> GetCasGeometryLibrary(
    std::shared_ptr<intrinsic_proto::content_addressable_storage::v1::
                        ContentAddressableStorageService::StubInterface>
        cas_stub,
    grpc::ServerContext* context, GeometryFingerprintCache& geometry_cache,
    std::multimap<grpc::string_ref, grpc::string_ref> metadata = {});

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::GetCasGeometryLibrary;
}  // namespace intrinsic

#endif  // INTRINSIC_GEOMETRY_STORAGE_CAS_STORAGE_H_
