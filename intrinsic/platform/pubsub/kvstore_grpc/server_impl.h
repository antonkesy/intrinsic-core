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

#ifndef INTRINSIC_PLATFORM_PUBSUB_KVSTORE_GRPC_SERVER_IMPL_H_
#define INTRINSIC_PLATFORM_PUBSUB_KVSTORE_GRPC_SERVER_IMPL_H_

#include <optional>

#include "absl/status/status.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/platform/pubsub/kvstore.h"
#include "intrinsic/platform/pubsub/kvstore_grpc/kvstore.grpc.pb.h"
#include "intrinsic/platform/pubsub/kvstore_grpc/kvstore.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"

namespace intrinsic::kvstore {

class KVStoreServerImpl final
    : public intrinsic_proto::kvstore::KVStore::Service {
 public:
  KVStoreServerImpl() = default;

  absl::Status Init();

  grpc::Status Get(grpc::ServerContext* context,
                   const intrinsic_proto::kvstore::GetRequest* request,
                   intrinsic_proto::kvstore::GetResponse* response) override;

  grpc::Status Set(grpc::ServerContext* context,
                   const intrinsic_proto::kvstore::SetRequest* request,
                   intrinsic_proto::kvstore::SetResponse* response) override;

  grpc::Status Delete(
      grpc::ServerContext* context,
      const intrinsic_proto::kvstore::DeleteRequest* request,
      intrinsic_proto::kvstore::DeleteResponse* response) override;

  grpc::Status List(grpc::ServerContext* context,
                    const intrinsic_proto::kvstore::ListRequest* request,
                    intrinsic_proto::kvstore::ListResponse* response) override;

 private:
  PubSub pubsub_;
  std::optional<KeyValueStore> kvstore_;
};

}  // namespace intrinsic::kvstore

#endif  // INTRINSIC_PLATFORM_PUBSUB_KVSTORE_GRPC_SERVER_IMPL_H_
