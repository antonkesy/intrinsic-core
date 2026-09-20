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

#include "intrinsic/storage/content_addressable_storage/testing/in_memory_cas_service.h"

#include <string>

#include "absl/strings/str_cat.h"

namespace intrinsic {

::grpc::Status InMemoryCasService::Create(
    ::grpc::ServerContext* context,
    ::grpc::ServerReader<
        ::intrinsic_proto::content_addressable_storage::v1::CreateRequest>*
        reader,
    ::intrinsic_proto::content_addressable_storage::v1::CreateResponse*
        response) {
  std::string data;
  ::intrinsic_proto::content_addressable_storage::v1::CreateRequest req;
  while (reader->Read(&req)) {
    data.append(req.checksummed_data().content());
  }
  std::string id = absl::StrCat("intcas://", storage_.size());
  storage_[id] = data;
  response->set_object_id(id);
  return ::grpc::Status::OK;
}

::grpc::Status InMemoryCasService::Get(
    ::grpc::ServerContext* context,
    const ::intrinsic_proto::content_addressable_storage::v1::GetRequest*
        request,
    ::grpc::ServerWriter<
        ::intrinsic_proto::content_addressable_storage::v1::GetResponse>*
        writer) {
  auto it = storage_.find(request->object_id());
  if (it == storage_.end()) {
    return ::grpc::Status(::grpc::StatusCode::NOT_FOUND, "Object not found");
  }
  ::intrinsic_proto::content_addressable_storage::v1::GetResponse response;
  response.mutable_checksummed_data()->set_content(it->second);
  writer->Write(response);
  return ::grpc::Status::OK;
}

}  // namespace intrinsic
