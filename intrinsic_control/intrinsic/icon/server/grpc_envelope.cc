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

#include "intrinsic/icon/server/grpc_envelope.h"

#include <signal.h>

#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/channel_arguments.h"
#include "grpcpp/support/server_interceptor.h"
#include "grpcpp/support/status.h"
#include "grpcpp/support/sync_stream.h"
#include "intrinsic/assets/services/proto/v1/service_state.grpc.pb.h"
#include "intrinsic/assets/services/proto/v1/service_state.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.grpc.pb.h"
#include "intrinsic/hardware/gpio/v1/gpio_service.pb.h"
#include "intrinsic/icon/common/topic_names.h"
#include "intrinsic/icon/proto/rt_tracing_service.grpc.pb.h"
#include "intrinsic/icon/proto/rt_tracing_service.pb.h"
#include "intrinsic/icon/proto/v1/jogging_service.grpc.pb.h"
#include "intrinsic/icon/proto/v1/jogging_service.pb.h"
#include "intrinsic/icon/proto/v1/service.grpc.pb.h"
#include "intrinsic/icon/proto/v1/service.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/icon_api_service.h"
#include "intrinsic/icon/utils/exit_code.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/logging/log_item_builder.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/stats/opencensus.h"
#include "intrinsic/util/grpc/limits.h"
#include "intrinsic/util/status/return.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_macros_grpc.h"
#include "intrinsic/util/thread/stop_token.h"
#include "intrinsic/util/thread/thread.h"

namespace intrinsic::icon {

constexpr absl::Duration kServerRestartMutexTimeout = absl::Seconds(30);

namespace {

constexpr double kErrorStatusPublishPeriodSeconds = 1.0;

}  // namespace

class GrpcEnvelope::WrapperService : public intrinsic::icon::IconApiService {
 public:
  explicit WrapperService(GrpcEnvelope& envelope) : envelope_(envelope) {}

  ::grpc::Status GetActionSignatureByName(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetActionSignatureByNameRequest* request,
      intrinsic_proto::icon::v1::GetActionSignatureByNameResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetActionSignatureByName(context, request, response);
  }

  ::grpc::Status GetConfig(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetConfigRequest* request,
      intrinsic_proto::icon::v1::GetConfigResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetConfig(context, request, response);
  }

  ::grpc::Status GetStatus(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetStatusRequest* request,
      intrinsic_proto::icon::v1::GetStatusResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetStatus(context, request, response);
  }

  ::grpc::Status IsActionCompatible(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::IsActionCompatibleRequest* request,
      intrinsic_proto::icon::v1::IsActionCompatibleResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->IsActionCompatible(context, request, response);
  }

  ::grpc::Status ListActionSignatures(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::ListActionSignaturesRequest* request,
      intrinsic_proto::icon::v1::ListActionSignaturesResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->ListActionSignatures(context, request, response);
  }

  ::grpc::Status ListCompatibleParts(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::ListCompatiblePartsRequest* request,
      intrinsic_proto::icon::v1::ListCompatiblePartsResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->ListCompatibleParts(context, request, response);
  }

  ::grpc::Status ListParts(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::ListPartsRequest* request,
      intrinsic_proto::icon::v1::ListPartsResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->ListParts(context, request, response);
  }

  ::grpc::Status OpenSession(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<intrinsic_proto::icon::v1::OpenSessionResponse,
                                 intrinsic_proto::icon::v1::OpenSessionRequest>*
          stream) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->OpenSession(context, stream);
  }

  ::grpc::Status WatchReactions(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::WatchReactionsRequest* request,
      ::grpc::ServerWriter<intrinsic_proto::icon::v1::WatchReactionsResponse>*
          writer) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->WatchReactions(context, request, writer);
  }

  ::grpc::Status OpenWriteStream(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<
          intrinsic_proto::icon::v1::OpenWriteStreamResponse,
          intrinsic_proto::icon::v1::OpenWriteStreamRequest>* stream) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->OpenWriteStream(context, stream);
  }

  ::grpc::Status GetLatestStreamingOutput(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetLatestStreamingOutputRequest* request,
      intrinsic_proto::icon::v1::GetLatestStreamingOutputResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetLatestStreamingOutput(context, request, response);
  }

  ::grpc::Status GetPlannedTrajectory(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetPlannedTrajectoryRequest* request,
      ::grpc::ServerWriter<
          intrinsic_proto::icon::v1::GetPlannedTrajectoryResponse>* stream)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetPlannedTrajectory(context, request, stream);
  }

  ::grpc::Status Enable(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::EnableRequest* request,
      intrinsic_proto::icon::v1::EnableResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->Enable(context, request, response);
  }

  ::grpc::Status Disable(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::DisableRequest* request,
      intrinsic_proto::icon::v1::DisableResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->Disable(context, request, response);
  }

  ::grpc::Status ClearFaults(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::ClearFaultsRequest* request,
      intrinsic_proto::icon::v1::ClearFaultsResponse* response) override {
    absl::Status error = absl::OkStatus();
    {
      absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
      error = envelope_.IconService().status();
    }
    // If we are latching an error, then attempt to restart the server.
    if (!error.ok()) {
      LOG(INFO) << "Got ClearFault call while in fatal fault, restarting...";
      return ToGrpcStatus(envelope_.RebuildIconImpl());
    }
    // Otherwise, clear ICON faults.
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->ClearFaults(context, request, response);
  }

  ::grpc::Status GetOperationalStatus(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetOperationalStatusRequest* request,
      intrinsic_proto::icon::v1::GetOperationalStatusResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    absl::StatusOr<IconApiService* absl_nonnull> icon_service =
        envelope_.IconService();
    if (!icon_service.ok()) {
      response->mutable_operational_status()->set_state(
          intrinsic_proto::icon::v1::OperationalState::FAULTED);
      response->mutable_operational_status()->set_fault_reason(
          icon_service.status().ToString());
      return ToGrpcStatus(absl::OkStatus());
    }
    return icon_service.value()->GetOperationalStatus(context, request,
                                                      response);
  }

  ::grpc::Status RestartServer(::grpc::ServerContext* context,
                               const google::protobuf::Empty* request,
                               google::protobuf::Empty* response) override {
    LOG(WARNING) << "PUBLIC: Received restart request, will close connections "
                    "and quit when sessions have closed.";
    return ToGrpcStatus(envelope_.RebuildIconImpl());
  }

  ::grpc::Status SetSpeedOverride(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::SetSpeedOverrideRequest* request,
      intrinsic_proto::icon::v1::SetSpeedOverrideResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->SetSpeedOverride(context, request, response);
  }

  ::grpc::Status GetSpeedOverride(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetSpeedOverrideRequest* request,
      intrinsic_proto::icon::v1::GetSpeedOverrideResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetSpeedOverride(context, request, response);
  }

  ::grpc::Status SetLoggingMode(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::SetLoggingModeRequest* request,
      intrinsic_proto::icon::v1::SetLoggingModeResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->SetLoggingMode(context, request, response);
  }

  ::grpc::Status GetLoggingMode(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetLoggingModeRequest* request,
      intrinsic_proto::icon::v1::GetLoggingModeResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetLoggingMode(context, request, response);
  }

  ::grpc::Status GetPartProperties(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetPartPropertiesRequest* request,
      intrinsic_proto::icon::v1::GetPartPropertiesResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetPartProperties(context, request, response);
  }

  ::grpc::Status SetPartProperties(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::SetPartPropertiesRequest* request,
      intrinsic_proto::icon::v1::SetPartPropertiesResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->SetPartProperties(context, request, response);
  }

  ::grpc::Status SetPayload(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::SetPayloadRequest* request,
      intrinsic_proto::icon::v1::SetPayloadResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->SetPayload(context, request, response);
  }

  ::grpc::Status GetPayload(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::GetPayloadRequest* request,
      intrinsic_proto::icon::v1::GetPayloadResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(IconApiService * icon_service,
                               envelope_.IconService());
    return icon_service->GetPayload(context, request, response);
  }

 private:
  GrpcEnvelope& envelope_;
};

intrinsic_proto::services::v1::SelfState::StateCode ConvertToStateCode(
    intrinsic_proto::icon::v1::OperationalState state) {
  switch (state) {
    case intrinsic_proto::icon::v1::DISABLED:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_DISABLED;
    case intrinsic_proto::icon::v1::FAULTED:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR;
    case intrinsic_proto::icon::v1::ENABLED:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_ENABLED;
    case intrinsic_proto::icon::v1::UNKNOWN:
    default:
      return intrinsic_proto::services::v1::SelfState::STATE_CODE_UNSPECIFIED;
  }
}

class GrpcEnvelope::IconHealthService
    : public intrinsic_proto::services::v1::ServiceState::Service {
 public:
  explicit IconHealthService(GrpcEnvelope& envelope) : envelope_(envelope) {}
  ~IconHealthService() override = default;

  ::grpc::Status GetState(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::GetStateRequest* request,
      intrinsic_proto::services::v1::SelfState* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    absl::StatusOr<IconApiService* absl_nonnull> icon_service =
        envelope_.IconService();
    if (!icon_service.ok()) {
      LOG(ERROR) << "Failed to get IconApi service: " << icon_service.status();
      response->set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
      response->mutable_extended_status()->set_title(
          "Realtime control service is not running.");
      response->mutable_extended_status()->mutable_user_report()->set_message(
          icon_service.status().ToString());
      return ToGrpcStatus(absl::OkStatus());
    }
    const intrinsic_proto::icon::v1::GetOperationalStatusRequest
        icon_operational_status_request;
    intrinsic_proto::icon::v1::GetOperationalStatusResponse
        icon_operational_status_response;
    // We know `icon_service` has a value here
    grpc::Status get_operational_status_status =
        icon_service.value()->GetOperationalStatus(
            context, &icon_operational_status_request,
            &icon_operational_status_response);
    if (!get_operational_status_status.ok()) {
      LOG(ERROR) << "Failed to get operational status: "
                 << get_operational_status_status.error_message();
      response->set_state_code(
          intrinsic_proto::services::v1::SelfState::STATE_CODE_ERROR);
      response->mutable_extended_status()->set_title(
          "Requesting operational status failed.");
      response->mutable_extended_status()->mutable_user_report()->set_message(
          get_operational_status_status.error_message());
      return get_operational_status_status;
    }
    response->set_state_code(ConvertToStateCode(
        icon_operational_status_response.operational_status().state()));
    if (!icon_operational_status_response.operational_status()
             .fault_reason()
             .empty()) {
      response->mutable_extended_status()->mutable_user_report()->set_message(
          icon_operational_status_response.operational_status().fault_reason());
    }
    return ToGrpcStatus(absl::OkStatus());
  }

  ::grpc::Status Enable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::EnableRequest* request,
      intrinsic_proto::services::v1::EnableResponse* response) override {
    absl::Status error = absl::OkStatus();
    absl::StatusOr<IconApiService* absl_nonnull> icon_service;
    {
      absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
      icon_service = envelope_.IconService();
      error = envelope_.IconService().status();
    }
    // If we are latching an error, then attempt to restart the server.
    if (!error.ok()) {
      LOG(INFO) << "Got Enable call while in fatal fault, restarting...";
      return ToGrpcStatus(envelope_.RebuildIconImpl());
    }

    // Check if there are faults we need to clear
    const intrinsic_proto::icon::v1::GetOperationalStatusRequest
        icon_operational_status_request;
    intrinsic_proto::icon::v1::GetOperationalStatusResponse
        icon_operational_status_response;
    grpc::Status get_operational_status_status =
        icon_service.value()->GetOperationalStatus(
            context, &icon_operational_status_request,
            &icon_operational_status_response);
    if (!get_operational_status_status.ok()) {
      LOG(ERROR) << "Failed to get operational status: "
                 << get_operational_status_status.error_message();
      return get_operational_status_status;
    }
    if (icon_operational_status_response.operational_status().state() ==
        intrinsic_proto::icon::v1::OperationalState::FAULTED) {
      LOG(INFO) << "ICON is faulted, clearing faults...";

      // Clear ICON faults.
      const intrinsic_proto::icon::v1::ClearFaultsRequest
          icon_clear_faults_request;
      intrinsic_proto::icon::v1::ClearFaultsResponse icon_clear_faults_response;
      absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
      grpc::Status error = icon_service.value()->ClearFaults(
          context, &icon_clear_faults_request, &icon_clear_faults_response);
      if (!error.ok()) {
        LOG(ERROR) << "Failed to clear ICON faults: " << error.error_message();
        return error;
      }

      // Wait (at most until deadline) for ICON to be enabled before returning.
      while (absl::Now() < absl::FromChrono(context->deadline())) {
        get_operational_status_status =
            icon_service.value()->GetOperationalStatus(
                context, &icon_operational_status_request,
                &icon_operational_status_response);
        if (!get_operational_status_status.ok()) {
          LOG(ERROR) << "Failed to get operational status: "
                     << get_operational_status_status.error_message();
          return get_operational_status_status;
        }
        if (icon_operational_status_response.operational_status().state() ==
            intrinsic_proto::icon::v1::OperationalState::ENABLED) {
          return ToGrpcStatus(absl::OkStatus());
        }
        absl::SleepFor(absl::Milliseconds(100));
      }

      return ToGrpcStatus(absl::DeadlineExceededError(
          "The robot controller could not verify it was enabled within the "
          "deadline. This could mean the controller is still processing."));
    }

    return ToGrpcStatus(absl::UnavailableError(
        "Cannot enable directly. The robot controller enables automatically "
        "when no hardware module has an error."));
  }

  ::grpc::Status Disable(
      grpc::ServerContext* context,
      const intrinsic_proto::services::v1::DisableRequest* request,
      intrinsic_proto::services::v1::DisableResponse* response) override {
    return ToGrpcStatus(absl::UnavailableError(
        "Cannot disable directly. The robot controller disables automatically "
        "when an error is detected."));
  }

 private:
  GrpcEnvelope& envelope_;
};

class GrpcEnvelope::GpioWrapperService
    : public intrinsic_proto::gpio::v1::GPIOService::Service {
 public:
  explicit GpioWrapperService(GrpcEnvelope& envelope) : envelope_(envelope) {}

  ::grpc::Status GetSignalDescriptions(
      ::grpc::ServerContext* context,
      const intrinsic_proto::gpio::v1::GetSignalDescriptionsRequest* request,
      intrinsic_proto::gpio::v1::GetSignalDescriptionsResponse* response)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        intrinsic_proto::gpio::v1::GPIOService::Service * gpio_service,
        envelope_.GpioService());
    return gpio_service->GetSignalDescriptions(context, request, response);
  }

  ::grpc::Status ReadSignals(
      ::grpc::ServerContext* context,
      const intrinsic_proto::gpio::v1::ReadSignalsRequest* request,
      intrinsic_proto::gpio::v1::ReadSignalsResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        intrinsic_proto::gpio::v1::GPIOService::Service * gpio_service,
        envelope_.GpioService());
    return gpio_service->ReadSignals(context, request, response);
  }

  ::grpc::Status WaitForValue(
      ::grpc::ServerContext* context,
      const intrinsic_proto::gpio::v1::WaitForValueRequest* request,
      intrinsic_proto::gpio::v1::WaitForValueResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        intrinsic_proto::gpio::v1::GPIOService::Service * gpio_service,
        envelope_.GpioService());
    return gpio_service->WaitForValue(context, request, response);
  }

  ::grpc::Status OpenWriteSession(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<
          intrinsic_proto::gpio::v1::OpenWriteSessionResponse,
          intrinsic_proto::gpio::v1::OpenWriteSessionRequest>* stream)
      override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        intrinsic_proto::gpio::v1::GPIOService::Service * gpio_service,
        envelope_.GpioService());
    return gpio_service->OpenWriteSession(context, stream);
  }

 private:
  GrpcEnvelope& envelope_;
};

class GrpcEnvelope::JoggingWrapperService
    : public intrinsic_proto::icon::v1::JoggingService::Service {
 public:
  explicit JoggingWrapperService(GrpcEnvelope& envelope)
      : envelope_(envelope) {}

  ::grpc::Status GetAvailableParts(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::v1::AvailablePartsRequest* request,
      intrinsic_proto::icon::v1::AvailablePartsResponse* response) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        intrinsic_proto::icon::v1::JoggingService::Service * jogging_service,
        envelope_.JoggingService());
    return jogging_service->GetAvailableParts(context, request, response);
  }

  ::grpc::Status JogRobot(
      ::grpc::ServerContext* context,
      ::grpc::ServerReaderWriter<intrinsic_proto::icon::v1::JoggingResponse,
                                 intrinsic_proto::icon::v1::JoggingRequest>*
          stream) override {
    absl::ReaderMutexLock l(envelope_.icon_impl_mutex_);
    INTR_ASSIGN_OR_RETURN_GRPC(
        intrinsic_proto::icon::v1::JoggingService::Service * jogging_service,
        envelope_.JoggingService());
    return jogging_service->JogRobot(context, stream);
  }

 private:
  GrpcEnvelope& envelope_;
};

class GrpcEnvelope::RealtimeTracingService
    : public intrinsic_proto::icon::RealtimeTracingApi::Service {
 public:
  ::grpc::Status StartTracing(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::StartTracingRequest* request,
      intrinsic_proto::icon::StartTracingResponse* response) override {
    return ToGrpcStatus(absl::UnimplementedError("b/370018091"));
  }

  ::grpc::Status PauseTracing(
      ::grpc::ServerContext* context,
      const intrinsic_proto::icon::PauseTracingRequest* request,
      intrinsic_proto::icon::PauseTracingResponse* response) override {
    return ToGrpcStatus(absl::UnimplementedError("b/370018091"));
  }
};

GrpcEnvelope::GrpcEnvelope(GrpcEnvelope::Config config)
    : config_(std::move(config)),
      publish_error_status_thread_(
          intrinsic::Thread{[this](const intrinsic::StopToken& stop_token) {
            this->PublishErrorStatusLoop(stop_token);
          }}) {
  if (config_.icon_impl_factory) {
    absl::MutexLock l(icon_impl_mutex_);
    icon_impl_ = config_.icon_impl_factory();
    if (!icon_impl_.status().ok()) {
      LOG(ERROR) << "ICON factory returned error: " << icon_impl_.status();
    }
  }
  StartServer();
}

GrpcEnvelope::~GrpcEnvelope() {
  if (!stop_error_status_thread_.HasBeenNotified()) {
    stop_error_status_thread_.Notify();
  }
  if (server_) {
    server_->Shutdown();
  }
  // `icon_impl_` gets destroyed implicitly after this destructor
  // returns.
  // Note: It's important for the order to be correct here:
  // 1. Shut down the gRPC server to prevent any new requests from coming in and
  //    wrap up in-flight ones.
  // 2. Now that there's no more gRPC requests using it, destroy `icon_impl_`.
}

absl::StatusOr<IconApiService* absl_nonnull> GrpcEnvelope::IconService() {
  INTR_RETURN_IF_ERROR(icon_impl_.status());
  return icon_impl_.value()->IconService();
}

absl::StatusOr<intrinsic_proto::gpio::v1::GPIOService::Service* absl_nonnull>
GrpcEnvelope::GpioService() {
  INTR_RETURN_IF_ERROR(icon_impl_.status());
  return icon_impl_.value()->GpioService();
}

absl::StatusOr<intrinsic_proto::icon::v1::JoggingService::Service* absl_nonnull>
GrpcEnvelope::JoggingService() {
  INTR_RETURN_IF_ERROR(icon_impl_.status());
  return icon_impl_.value()->JoggingService();
}

absl::Status GrpcEnvelope::RebuildIconImpl() {
  TryCancelAllStreams();
  // There is no method to wait with a timeout on a locked Mutex, only on a
  // condition *protected by* a Mutex.
  // So, we bring out the big guns: This thread is a "time bomb" that kills the
  // entire process if we don't disarm it (by calling `RequestStop()`) before
  // the deadline.
  intrinsic::Thread kill_if_mutex_is_blocked_thread{
      [](const intrinsic::StopToken& stop_token,
         absl::Time mutex_block_deadline) {
        while (absl::Now() < mutex_block_deadline) {
          if (stop_token.stop_requested()) {
            return;
          }
          absl::SleepFor(absl::Milliseconds(100));
        }
        // If this thread wasn't "disarmed" by requesting a stop, then the
        // MutexLock below is blocking execution. Assume the worst and kill the
        // ICON process.
        std::exit(static_cast<int>(ExitCode::kFatalFaultDuringExec));
      },
      absl::Now() + kServerRestartMutexTimeout};

  // This should block until all current requests have released their locks.
  // Because we called TryCancelAllStreams(), that should happen eventually. If
  // it doesn't, then `kill_if_mutex_is_blocked_thread` stops the server
  // process.
  absl::MutexLock l(icon_impl_mutex_);
  // Got the mutex, time to disarm our time bomb.
  kill_if_mutex_is_blocked_thread.request_stop();
  kill_if_mutex_is_blocked_thread.join();
  // First destroy the old ICON implementation (to make sure old and new don't
  // overlap).
  icon_impl_ = absl::UnavailableError("Restarting ICON service...");
  if (config_.icon_shutdown_for_restart_callback) {
    if (auto status = config_.icon_shutdown_for_restart_callback();
        !status.ok()) {
      LOG(ERROR) << "ICON shutdown callback failed: " << status;
    }
  }
  // Then use the factory to create a new ICON implementation.
  if (!config_.icon_impl_factory) {
    auto error = absl::InternalError(
        "Missing ICON factory, please report this as a bug");
    LOG(ERROR) << error;
    return error;
  }
  icon_impl_ = config_.icon_impl_factory();
  // If the factory returned an error, we want to forward that.
  if (!icon_impl_.status().ok()) {
    LOG(ERROR) << "ICON factory returned error: " << icon_impl_.status();
  }
  return icon_impl_.status();
}

void GrpcEnvelope::PublishErrorStatusLoop(
    const intrinsic::StopToken& stop_token) {
  const std::string& robot_name = config_.icon_name;
  INTR_ASSIGN_OR_RETURN(std::string topic_full_rate,
                        TopicNameForRobotStatus(robot_name),
                        _.LogError().With(ReturnVoid()));
  INTR_ASSIGN_OR_RETURN(std::string topic_decimated,
                        TopicNameForRobotStatusThrottle(robot_name),
                        _.LogError().With(ReturnVoid()));

  // Creating PubSub and Publishers is kind of expensive, so we defer until we
  // actually need to publish.
  std::optional<PubSub> pub_sub;
  std::optional<Publisher> pub_decimated;
  std::optional<Publisher> pub_full_rate;

  intrinsic_proto::icon::RobotStatus robot_status;
  robot_status.mutable_operational_status()->set_state(
      intrinsic_proto::icon::v1::OperationalState::FAULTED);

  // Wait for the stop notification at the top of the loop – this doubles as a
  // sleep statement, which is nice because the loop has two distinct exit
  // points, and sleeping at both of those could be error-prone.
  while (!stop_error_status_thread_.WaitForNotificationWithTimeout(
             absl::Seconds(kErrorStatusPublishPeriodSeconds)) &&
         !stop_token.stop_requested()) {
    // Reset fault reason to the empty string before checking ICON health.
    robot_status.mutable_operational_status()->set_fault_reason("");
    {
      absl::ReaderMutexLock l(icon_impl_mutex_);
      if (!icon_impl_.ok()) {
        robot_status.mutable_operational_status()->set_fault_reason(
            icon_impl_.status().ToString());
      } else if (auto service_status =
                     icon_impl_.value()->IconService().status();
                 !service_status.ok()) {
        robot_status.mutable_operational_status()->set_fault_reason(
            service_status.ToString());
      }
    }
    if (robot_status.operational_status().fault_reason().empty()) {
      continue;
    }
    if (pub_sub == std::nullopt) {
      pub_sub.emplace();
    }
    if (pub_decimated == std::nullopt) {
      INTR_ASSIGN_OR_RETURN(
          pub_decimated,
          pub_sub->CreatePublisher(topic_decimated, TopicConfig()),
          _.LogError().With(ReturnVoid()));
    }
    if (pub_full_rate == std::nullopt) {
      INTR_ASSIGN_OR_RETURN(
          pub_full_rate,
          pub_sub->CreatePublisher(topic_full_rate, TopicConfig()),
          _.LogError().With(ReturnVoid()));
    }
    absl::Time wall_time = absl::Now();

    robot_status.set_wall_clock_timestamp_ns(absl::ToUnixNanos(wall_time));

    intrinsic_proto::data_logger::LogItem log_item =
        intrinsic::data_logger::Builder::From(robot_status)
            .WithEventSource(topic_full_rate)
            .Item();
    if (absl::Status status = pub_full_rate->Publish(log_item, wall_time);
        !status.ok()) {
      LOG_EVERY_N_SEC(ERROR, 1)
          << "Failed to publish init fault RobotStatus on topic ["
          << topic_full_rate << "]. Dropping.";
    }
    // Always also publish to throttled topic (we're publishing very slowly
    // here, anyway)
    log_item.mutable_metadata()->set_event_source(topic_decimated);
    if (absl::Status status = pub_decimated->Publish(log_item, wall_time);
        !status.ok()) {
      LOG_EVERY_N_SEC(ERROR, 1)
          << "Failed to publish init fault RobotStatus on topic ["
          << topic_decimated << "]. Dropping.";
    }
  }
}

void GrpcEnvelope::TryCancelAllStreams() {
  absl::ReaderMutexLock l(icon_impl_mutex_);
  auto icon_service = IconService();
  if (icon_service.ok()) {
    icon_service.value()->TryCancel();
  }
}

void GrpcEnvelope::StartServer() {
  INTRINSIC_ASSERT_NON_REALTIME();
  wrapper_service_ = std::make_unique<WrapperService>(*this);
  gpio_wrapper_service_ = std::make_unique<GpioWrapperService>(*this);

  jogging_wrapper_service_ = std::make_unique<JoggingWrapperService>(*this);
  icon_health_service_ = std::make_unique<IconHealthService>(*this);
  rt_tracing_service_ = std::make_unique<RealtimeTracingService>();

  grpc::ServerBuilder builder;
  if (config_.grpc_address.has_value()) {
    builder.AddListeningPort(
        config_.grpc_address.value(),
        ::grpc::InsecureServerCredentials()  // NOLINT (insecure)
    );
  }
  // Set the max message receive size to 512MB to allow longer trajectories.
  // Please check with the motion team before changing the value (see
  // b/275280379).
  builder.SetMaxReceiveMessageSize((512 * 1024 * 1024));
  builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);
  builder.AddChannelArgument(GRPC_ARG_MAX_METADATA_SIZE,
                             kGrpcRecommendedMaxMetadataSoftLimit);
  builder.AddChannelArgument(GRPC_ARG_ABSOLUTE_MAX_METADATA_SIZE,
                             kGrpcRecommendedMaxMetadataHardLimit);

  builder.RegisterService(icon_health_service_.get());
  builder.RegisterService(wrapper_service_.get());
  builder.RegisterService(gpio_wrapper_service_.get());
  builder.RegisterService(jogging_wrapper_service_.get());
  if (config_.auto_config_service != nullptr) {
    builder.RegisterService(config_.auto_config_service.get());
  }
  builder.RegisterService(rt_tracing_service_.get());

  // OpenCensusInterceptor creates metrics that can be used to detect the
  // first RPC with a given status, unlike gRPC's built-in metrics.
  std::vector<
      std::unique_ptr<grpc::experimental::ServerInterceptorFactoryInterface>>
      creators;
  creators.push_back(std::make_unique<OpenCensusInterceptorFactory>(
      intrinsic_proto::icon::v1::IconApi::service_full_name()));
  builder.experimental().SetInterceptorCreators(std::move(creators));

  std::string server_description = "In-Process ICON Server";
  if (config_.grpc_address) {
    server_description =
        absl::StrCat("ICON Server listening on ", *config_.grpc_address);
  }
  server_ = builder.BuildAndStart();
  if (!server_) {
    LOG(FATAL) << "Failed to start " << server_description;
  }
  LOG(INFO) << "Started " << server_description;
}

std::shared_ptr<grpc::Channel> GrpcEnvelope::InProcChannel(
    const grpc::ChannelArguments& channel_args) {
  if (!server_) {
    return nullptr;
  }
  return server_->InProcessChannel(channel_args);
}

void GrpcEnvelope::Wait() {
  if (server_ == nullptr) {
    return;
  }
  server_->Wait();
}

}  // namespace intrinsic::icon
