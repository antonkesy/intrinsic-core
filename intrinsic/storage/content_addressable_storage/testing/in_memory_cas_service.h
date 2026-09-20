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

#ifndef INTRINSIC_STORAGE_CONTENT_ADDRESSABLE_STORAGE_TESTING_IN_MEMORY_CAS_SERVICE_H_
#define INTRINSIC_STORAGE_CONTENT_ADDRESSABLE_STORAGE_TESTING_IN_MEMORY_CAS_SERVICE_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "grpcpp/support/sync_stream.h"
#include "intrinsic/storage/content_addressable_storage/proto/cas_service.grpc.pb.h"
#include "intrinsic/storage/content_addressable_storage/proto/cas_service.pb.h"

namespace intrinsic {

// An in-memory CAS service implementation for testing without external process
// dependencies.
class InMemoryCasService : public intrinsic_proto::content_addressable_storage::
                               v1::ContentAddressableStorageService::Service {
 public:
  InMemoryCasService() = default;
  ~InMemoryCasService() override = default;

  ::grpc::Status Create(
      ::grpc::ServerContext* context,
      ::grpc::ServerReader<
          ::intrinsic_proto::content_addressable_storage::v1::CreateRequest>*
          reader,
      ::intrinsic_proto::content_addressable_storage::v1::CreateResponse*
          response) override;

  ::grpc::Status Get(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::content_addressable_storage::v1::GetRequest*
          request,
      ::grpc::ServerWriter<
          ::intrinsic_proto::content_addressable_storage::v1::GetResponse>*
          writer) override;

 private:
  absl::flat_hash_map<std::string, std::string> storage_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_STORAGE_CONTENT_ADDRESSABLE_STORAGE_TESTING_IN_MEMORY_CAS_SERVICE_H_
