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

#include "intrinsic/platform/pubsub/kvstore_grpc/server_impl.h"

#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/any.pb.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/platform/pubsub/kvstore.h"
#include "intrinsic/platform/pubsub/kvstore_grpc/kvstore.pb.h"
#include "intrinsic/platform/pubsub/kvstore_grpc/metrics.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros_grpc.h"

namespace intrinsic::kvstore {

absl::Status KVStoreServerImpl::Init() {
  // Wait for the kvstore to be ready.
  while (true) {
    absl::StatusOr<KeyValueStore> kvstore = pubsub_.KeyValueStore();
    absl::Status status = kvstore.status();
    if (kvstore.ok()) {
      status = kvstore->Set("grpc_kvstore_ready", google::protobuf::Any(),
                            /*high_consistency=*/true);
      if (status.ok()) {
        kvstore_ = *std::move(kvstore);
        break;
      }
    }

    LOG(INFO) << "Waiting for kvstore to be ready: " << status;
    absl::SleepFor(absl::Milliseconds(500));
  }
  return absl::OkStatus();
}

grpc::Status KVStoreServerImpl::Get(
    grpc::ServerContext* context,
    const intrinsic_proto::kvstore::GetRequest* request,
    intrinsic_proto::kvstore::GetResponse* response) {
  GetMetricsScope scope;
  LOG(INFO) << "Getting key: " << request->key();
  absl::StatusOr<google::protobuf::Any> ret =
      kvstore_->Get<google::protobuf::Any>(request->key(), kDefaultGetTimeout);
  if (!ret.ok()) {
    return scope.CaptureAndReturnGrpcStatus(ret.status());
  }
  *response->mutable_value() = *std::move(ret);
  return scope.CaptureAndReturnGrpcStatus(absl::OkStatus());
}

grpc::Status KVStoreServerImpl::Set(
    grpc::ServerContext* context,
    const intrinsic_proto::kvstore::SetRequest* request,
    intrinsic_proto::kvstore::SetResponse* response) {
  bool high_consistency =
      request->consistency() ==
      intrinsic_proto::kvstore::SetRequest_Consistency_CONSISTENCY_HIGH;
  SetMetricsScope scope(high_consistency);
  int64_t payload_size = request->value().ByteSizeLong();
  scope.RecordPayloadSize(payload_size);

  LOG(INFO) << "Setting key: " << request->key()
            << (high_consistency ? " with high consistency" : "")
            << ", payload size: " << payload_size << " bytes";

  return scope.CaptureAndReturnGrpcStatus(
      kvstore_->Set(request->key(), request->value(),
                    /*high_consistency=*/true));
}

grpc::Status KVStoreServerImpl::Delete(
    grpc::ServerContext* context,
    const intrinsic_proto::kvstore::DeleteRequest* request,
    intrinsic_proto::kvstore::DeleteResponse* response) {
  DeleteMetricsScope scope;
  return scope.CaptureAndReturnGrpcStatus(kvstore_->Delete(request->key()));
}

grpc::Status KVStoreServerImpl::List(
    grpc::ServerContext* context,
    const intrinsic_proto::kvstore::ListRequest* request,
    intrinsic_proto::kvstore::ListResponse* response) {
  ListMetricsScope scope;
  absl::StatusOr<std::vector<std::string>> keys = kvstore_->ListAllKeys();
  if (!keys.ok()) {
    return scope.CaptureAndReturnGrpcStatus(keys.status());
  }
  response->mutable_keys()->Add(keys->begin(), keys->end());
  return scope.CaptureAndReturnGrpcStatus(absl::OkStatus());
}

}  // namespace intrinsic::kvstore
