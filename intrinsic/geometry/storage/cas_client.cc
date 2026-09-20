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

#include "intrinsic/geometry/storage/cas_client.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"
#include "grpcpp/support/channel_arguments.h"
#include "intrinsic/storage/content_addressable_storage/proto/cas_service.grpc.pb.h"

namespace intrinsic::geo {

using ::intrinsic_proto::content_addressable_storage::v1::
    ContentAddressableStorageService;

// Note: retry policy is based on the one used by the polymorphic data frontend
// and is meant to address flaky connections to on-prem CAS service on startup
// (e.g., b/395559102#comment55).
// https://github.com/intrinsic-ai/intrinsic-core/blob/main/storage/polymorphic_data_frontend/polymorphic_data_frontend_main.go
constexpr char kCasRetryPolicy[] = R"(
        {
          "methodConfig": [{
            "name": [{"service": "intrinsic_proto.content_addressable_storage.v1.ContentAddressableStorageService"}],
            "waitForReady": true,
            "timeout": "300s",
            "retryPolicy": {
                "maxAttempts": 5,
                "initialBackoff": "1s",
                "maxBackoff": "10s",
                "backoffMultiplier": 1.5,
                "retryableStatusCodes": [ "UNAVAILABLE" ]
            }
          }]
        })";

absl::StatusOr<std::unique_ptr<ContentAddressableStorageService::Stub>>
MakeCASServiceStub(absl::string_view cas_service_address) {
  // Note: We don't use intrinsic::CreateClientChannel because we don't want to
  // block starting this service until the depending service is available.
  ::grpc::ChannelArguments channel_args;
  channel_args.SetServiceConfigJSON(kCasRetryPolicy);
  auto channel = ::grpc::CreateCustomChannel(
      std::string(cas_service_address),
      ::grpc::                       // NOLINTNEXTLINE
      InsecureChannelCredentials(),  // NO_LINT(grpc_insecure_credential_linter)
      channel_args);

  auto cas_stub = ContentAddressableStorageService::NewStub(std::move(channel));
  if (!cas_stub) {
    return absl::InternalError("Couldn't create stub for the CAS service.");
  }
  return std::move(cas_stub);
}

}  // namespace intrinsic::geo
