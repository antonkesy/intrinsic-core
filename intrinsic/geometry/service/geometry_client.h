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

#ifndef INTRINSIC_GEOMETRY_SERVICE_GEOMETRY_CLIENT_H_
#define INTRINSIC_GEOMETRY_SERVICE_GEOMETRY_CLIENT_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "intrinsic/geometry/processing/pipeline_config.pb.h"
#include "intrinsic/geometry/proto/geometry.pb.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/geometry/proto/geometry_service.pb.h"
#include "intrinsic/geometry/proto/geometry_service_types.pb.h"
#include "intrinsic/geometry/proto/v1/geometry.pb.h"
#include "intrinsic/longrunning/cc/operations_proxy.h"
#include "intrinsic/util/thread/stop_token.h"

namespace intrinsic::geo {
// Helper functions to create geometry service stubs.
// It is highly recommended to use these functions instead of creating the stub
// manually. Internally, the channel arguments are configured to retry on
// certain errors, and to set the max message size to unlimited. See
// `MakeGeometryServiceChannelArguments` for details.
//
// Creates a geometry service stub that does not block on channel creation.
absl::StatusOr<
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::Stub>>
MakeGeometryServiceStubNonBlocking(absl::string_view geometry_service_address);

// Creates a geometry service stub that blocks on channel creation.
// Same as above, but the channel creation will be blocked until the channel is
// ready or the connection times out.
absl::StatusOr<
    std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>>
MakeGeometryServiceStubBlocking(absl::string_view geometry_service_address,
                                absl::Duration connection_timeout);

// Client class that talks to GeometryService grpc service and its corresponding
// longrunning operations proxy to create, get and process geometries.
class GeometryClient {
 public:
  // Creates a geometry service client from its service stub and its known proxy
  // to longrunning operations.
  static absl::StatusOr<std::unique_ptr<GeometryClient>> Create(
      std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
          stub,
      std::shared_ptr<longrunning::OperationsProxy> operations_proxy);

  // Creates a geometry service client from a service stub owned externally and
  // its known proxy to longrunning operations.
  static absl::StatusOr<std::unique_ptr<GeometryClient>> Create(
      intrinsic_proto::geometry::GeometryService::StubInterface* stub,
      std::shared_ptr<longrunning::OperationsProxy> operations_proxy);

  // Gets a geometry from its storage refs.
  // Supports cancellation.
  absl::StatusOr<intrinsic_proto::geometry::GeometryWithMetadata> GetGeometry(
      grpc::ClientContext& context,
      const intrinsic_proto::geometry::GeometryStorageRefs& storage_refs);

  // Creates a geometry.
  // Supports cancellation.
  absl::StatusOr<intrinsic_proto::geometry::GeometryWithMetadata>
  CreateGeometry(grpc::ClientContext& context,
                 intrinsic_proto::geometry::Geometry geometry);

  // Processes geometry.
  // Supports cancellation.
  // `cancel_operation` token is checked periodically to see if the processing
  // should be cancelled. This is especially useful in situations where the
  // `ProcessGeometry` call is invoked as part of another longrunning operation.
  // NOTE: explicitly pass `cancel_operation` by value to avoid potential
  // lifetime issues if, say, the original token is no longer valid (which can
  // happen with concurrent LROs).
  absl::StatusOr<intrinsic_proto::geometry::GeometryProcessingResult>
  ProcessGeometry(
      grpc::ClientContext& context, StopToken cancel_operation,
      const intrinsic_proto::geometry::ProcessGeometryRequest& request);

 private:
  GeometryClient(
      std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
          stub,
      std::shared_ptr<longrunning::OperationsProxy> operations_proxy);

  GeometryClient(
      intrinsic_proto::geometry::GeometryService::StubInterface* stub,
      std::shared_ptr<longrunning::OperationsProxy> operations_proxy);

  std::unique_ptr<intrinsic_proto::geometry::GeometryService::StubInterface>
      owned_stub_;
  intrinsic_proto::geometry::GeometryService::StubInterface* stub_;

  std::shared_ptr<longrunning::OperationsProxy> operations_proxy_;
};

}  // namespace intrinsic::geo

namespace intrinsic {
using ::intrinsic::geo::GeometryClient;
}  // namespace intrinsic

#endif  // INTRINSIC_GEOMETRY_SERVICE_GEOMETRY_CLIENT_H_
