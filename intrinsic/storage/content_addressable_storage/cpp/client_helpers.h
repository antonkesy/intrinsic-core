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

#ifndef INTRINSIC_STORAGE_CONTENT_ADDRESSABLE_STORAGE_CLIENT_HELPERS_H_
#define INTRINSIC_STORAGE_CONTENT_ADDRESSABLE_STORAGE_CLIENT_HELPERS_H_

#include <cstddef>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "grpcpp/client_context.h"
#include "intrinsic/storage/content_addressable_storage/proto/cas_service.grpc.pb.h"
#include "riegeli/bytes/reader.h"
#include "riegeli/bytes/writer.h"

namespace intrinsic {

// Recommended size of content to be included into one stream request.
//
// Why 1 MiB?
// https://cloud.google.com/blog/products/gcp/optimizing-your-cloud-storage-performance-google-cloud-performance-atlas
// suggests that 1 MB+ is a good chunk size.
inline constexpr std::size_t kContentAddressableStorageDefaultUploadChunkSize =
    1 * 1024 * 1024;

// Retrieves data from the content-addressable storage (CAS) by its object ID.
// Any errors returned from the CAS service are passed through, noticeably
// `::absl::StatusCode::kNotFound` if the object is not present in the CAS or
// `::absl::StatusCode::kInvalidArgument` if the object ID is invalid.
// The `sink` may not be concurrently used by other threads. Data is appended
// to the `sink` in chunks which size is determined by the CAS service.
::absl::Status ContentAddressableStorageGet(
    ::grpc::ClientContext* context,
    ::intrinsic_proto::content_addressable_storage::v1::
        ContentAddressableStorageService::StubInterface* cas_stub,
    std::string_view object_id, ::riegeli::Writer* writer);

// Retrieves data from the content-addressable storage (CAS) by its object ID.
// Any errors returned from the CAS service are passed through, noticeably
// `::absl::StatusCode::kNotFound` if the object is not present in the CAS or
// `::absl::StatusCode::kInvalidArgument` if the object ID is invalid.
::absl::StatusOr<std::string> ContentAddressableStorageGet(
    ::grpc::ClientContext* context,
    ::intrinsic_proto::content_addressable_storage::v1::
        ContentAddressableStorageService::StubInterface* cas_stub,
    std::string_view object_id);

// Uploads data to the content-addressable storage. Returns its object ID on
// success. This function is safe to call if the same data is already in CAS.
// The `source` may not be concurrently used by other threads.
::absl::StatusOr<std::string> ContentAddressableStorageCreate(
    ::grpc::ClientContext* context,
    ::intrinsic_proto::content_addressable_storage::v1::
        ContentAddressableStorageService::StubInterface* cas_stub,
    ::riegeli::Reader* reader,
    std::size_t chunk_size = kContentAddressableStorageDefaultUploadChunkSize);

// Uploads data to the content-addressable storage. Returns its object ID on
// success. This function is safe to call if the same data is already in CAS.
::absl::StatusOr<std::string> ContentAddressableStorageCreate(
    ::grpc::ClientContext* context,
    ::intrinsic_proto::content_addressable_storage::v1::
        ContentAddressableStorageService::StubInterface* cas_stub,
    ::absl::string_view content,
    std::size_t chunk_size = kContentAddressableStorageDefaultUploadChunkSize);

}  // namespace intrinsic

#endif  // INTRINSIC_STORAGE_CONTENT_ADDRESSABLE_STORAGE_CLIENT_HELPERS_H_
