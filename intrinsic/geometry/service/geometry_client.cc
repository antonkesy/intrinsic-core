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

#include "intrinsic/geometry/service/geometry_client.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/die_if_null.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/longrunning/operations.pb.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"
#include "grpcpp/support/channel_arguments.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/geometry/processing/pipeline_config.pb.h"
#include "intrinsic/geometry/proto/geometry.pb.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/proto/geometry_service.pb.h"
#include "intrinsic/geometry/proto/geometry_service_types.pb.h"
#include "intrinsic/geometry/proto/geometry_storage_refs.pb.h"
#include "intrinsic/longrunning/cc/operations_proxy.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/stop_token.h"

namespace intrinsic::geo {
namespace {

// Retry policy for the geometry service.
// This makes the client retry on UNAVAILABLE errors, which can happen when:
// - geometry service is not yet ready to serve requests, or
// - gRPC channel is not yet ready, or
// - geometry service is being shut down (e.g., due to platform update).
constexpr char kGeometryServiceRetryPolicy[] = R"(
        {
          "methodConfig": [{
            "name": [{"service": "intrinsic_proto.geometry.GeometryService"}],
            "waitForReady": true,
            "timeout": "300s",
            "retryPolicy": {
                "maxAttempts": 5,
                "initialBackoff": "1s",
                "maxBackoff": "10s",
                "backoffMultiplier": 1.5,
                "retryableStatusCodes": [ "UNAVAILABLE"]
            }
          }]
        })";

::grpc::ChannelArguments MakeGeometryServiceChannelArguments() {
  ::grpc::ChannelArguments channel_args;
  channel_args.SetServiceConfigJSON(kGeometryServiceRetryPolicy);

  // Sets the max message size to unlimited.
  channel_args.SetMaxReceiveMessageSize(-1);
  channel_args.SetMaxSendMessageSize(-1);

  return channel_args;
}

}  // namespace

absl::StatusOr<
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::Stub>>
MakeGeometryServiceStubNonBlocking(absl::string_view geometry_service_address) {
  // Note: We don't use intrinsic::CreateClientChannel because we don't want to
  // block starting this service until the depending service is available.
  auto channel = ::grpc::CreateCustomChannel(
      std::string(geometry_service_address),
      ::grpc::                       // NOLINTNEXTLINE
      InsecureChannelCredentials(),  // NO_LINT(grpc_insecure_credential_linter)
      MakeGeometryServiceChannelArguments());

  auto geometry_stub =
      intrinsic_proto::geometry::GeometryService::NewStub(std::move(channel));
  if (!geometry_stub) {
    return absl::InternalError(
        "Couldn't create stub for the geometry service.");
  }
  return std::move(geometry_stub);
}

absl::StatusOr<
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>>
MakeGeometryServiceStubBlocking(absl::string_view geometry_service_address,
                                const absl::Duration connection_timeout) {
  INTR_ASSIGN_OR_RETURN(
      auto channel,
      intrinsic::connect::CreateClientChannel(
          geometry_service_address, absl::Now() + connection_timeout,
          MakeGeometryServiceChannelArguments()));

  return intrinsic_proto::geometry::GeometryService::NewStub(
      std::move(channel));
}

absl::StatusOr<std::unique_ptr<GeometryClient>> GeometryClient::Create(
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
        stub,
    std::shared_ptr<longrunning::OperationsProxy> operations_proxy) {
  INTR_RET_CHECK_NE(stub, nullptr);
  INTR_RET_CHECK_NE(operations_proxy, nullptr);
  return absl::WrapUnique(
      new GeometryClient(std::move(stub), std::move(operations_proxy)));
}

absl::StatusOr<std::unique_ptr<GeometryClient>> GeometryClient::Create(
    intrinsic_proto::geometry::GeometryService::StubInterface* stub,
    std::shared_ptr<longrunning::OperationsProxy> operations_proxy) {
  INTR_RET_CHECK_NE(stub, nullptr);
  INTR_RET_CHECK_NE(operations_proxy, nullptr);
  return absl::WrapUnique(
      new GeometryClient(stub, std::move(operations_proxy)));
}

GeometryClient::GeometryClient(
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
        stub,
    std::shared_ptr<longrunning::OperationsProxy> operations_proxy)
    : owned_stub_(ABSL_DIE_IF_NULL(std::move(stub))),
      stub_(ABSL_DIE_IF_NULL(owned_stub_).get()),
      operations_proxy_(ABSL_DIE_IF_NULL(std::move(operations_proxy))) {}

GeometryClient::GeometryClient(
    intrinsic_proto::geometry::GeometryService::StubInterface* stub,
    std::shared_ptr<longrunning::OperationsProxy> operations_proxy)
    : stub_(ABSL_DIE_IF_NULL(stub)),
      operations_proxy_(ABSL_DIE_IF_NULL(operations_proxy)) {}

absl::StatusOr<intrinsic_proto::geometry::GeometryWithMetadata>
GeometryClient::GetGeometry(
    grpc::ClientContext& context,
    const intrinsic_proto::geometry::GeometryStorageRefs& storage_refs) {
  intrinsic_proto::geometry::GetGeometryRequest request;
  *request.mutable_geometry_storage_refs_v0() = storage_refs;

  intrinsic_proto::geometry::GeometryWithMetadata result;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(stub_->GetGeometry(&context, request, &result)));

  return result;
}

absl::StatusOr<intrinsic_proto::geometry::GeometryWithMetadata>
GeometryClient::CreateGeometry(grpc::ClientContext& context,
                               intrinsic_proto::geometry::Geometry geometry) {
  intrinsic_proto::geometry::CreateGeometryRequest request;
  *request.mutable_data()->mutable_geometry_v0() = std::move(geometry);
  intrinsic_proto::geometry::GeometryWithMetadata result;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(stub_->CreateGeometry(&context, request, &result)));
  return result;
}

absl::StatusOr<intrinsic_proto::geometry::GeometryProcessingResult>
GeometryClient::ProcessGeometry(
    grpc::ClientContext& context, StopToken cancel_operation,
    const intrinsic_proto::geometry::ProcessGeometryRequest& request) {
  if (cancel_operation.stop_requested()) {
    return absl::CancelledError("Geometry processing cancelled.");
  }

  google::longrunning::Operation operation;
  INTR_RETURN_IF_ERROR(
      ToAbslStatus(stub_->ProcessGeometry(&context, request, &operation)))
      .LogError();

  google::longrunning::GetOperationRequest get_operation_request;
  get_operation_request.set_name(operation.name());
  while (!operation.done() && !operation.has_error()) {
    if (cancel_operation.stop_requested()) {
      google::longrunning::CancelOperationRequest cancel_operation_request;
      cancel_operation_request.set_name(operation.name());
      INTR_RETURN_IF_ERROR(
          operations_proxy_->CancelOperation(cancel_operation_request))
          .LogError();
      return absl::CancelledError("Geometry processing cancelled.");
    }
    absl::SleepFor(absl::Milliseconds(100));
    INTR_ASSIGN_OR_RETURN(
        operation, operations_proxy_->GetOperation(get_operation_request));
  }

  if (operation.has_error()) {
    return absl::InternalError(
        absl::Substitute("Geometry processing failed with error: $0",
                         operation.error().message()));
  }

  intrinsic_proto::geometry::GeometryProcessingResult processing_result;
  if (!operation.response().UnpackTo(&processing_result)) {
    return absl::InternalError(
        "Geometry processing failed when unpacking the processing result.");
  }

  return processing_result;
}

}  // namespace intrinsic::geo
