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

#include "intrinsic/scene/cad/grpc_cad_importer.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "grpcpp/client_context.h"
#include "grpcpp/support/channel_arguments.h"
#include "grpcpp/support/sync_stream.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/geometry/proto/geometry_storage_refs.pb.h"
#include "intrinsic/scene/cad/cad_import_service.grpc.pb.h"
#include "intrinsic/scene/cad/cad_import_service.pb.h"
#include "intrinsic/scene/cad/cad_importer.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/proto/geometry_component.pb.h"
#include "intrinsic/world/proto/world_entity.pb.h"

namespace intrinsic {
namespace world {

GRPCCADImporter::GRPCCADImporter(
    std::unique_ptr<
        intrinsic_proto::scene::cad::CADImportService::StubInterface>
        stub)
    : stub_(std::move(stub)) {}

absl::StatusOr<intrinsic_proto::scene_object::v1::ImportedScene>
GRPCCADImporter::ImportCADAsScene(
    absl::string_view cad_file_data,
    const intrinsic_proto::scene::cad::CADImportConfig& cad_import_config) {
  grpc::ClientContext context;
  intrinsic_proto::scene::cad::CADImportRequest request;
  *request.mutable_import_file_data()->mutable_data() =
      std::string(cad_file_data);
  std::string file_name =
      absl::StrCat("cad_import_", absl::ToUnixNanos(absl::Now()), ".step");
  *request.mutable_import_file_data()->mutable_file_name() = file_name;

  LOG(INFO) << "Sending CAD import to ImportedScene request for file: "
            << file_name;
  request.set_target_format(
      intrinsic_proto::scene::cad::CADImportTargetFormat::IMPORTED_SCENE);
  *request.mutable_config() = cad_import_config;
  auto client_reader = stub_->ImportCAD(&context, request);

  // Expect a single imported_scene response message. Returns error for all
  // other cases.
  intrinsic_proto::scene::cad::CADImportResponse response;
  if (!client_reader->Read(&response)) {
    INTR_RETURN_IF_ERROR(ToAbslStatus(client_reader->Finish()));

    return absl::InternalError(
        "CAD Import request did not write back a valid imported_scene and any "
        "error message.");
  }
  if (!response.has_imported_scene()) {
    return absl::InternalError(
        absl::Substitute("Import CAD as scene expect response containing only "
                         "imported_scene. $0 received instead.",
                         response));
  }

  if (intrinsic_proto::scene::cad::CADImportResponse second_response;
      client_reader->Read(&response)) {
    return absl::InternalError(absl::Substitute(
        "Import CAD as scene expect single response containing only "
        "imported_scene. A second response $0 received.",
        second_response));
  }

  return response.imported_scene();
}

absl::StatusOr<std::unique_ptr<CADImporter>> CreateGRPCCADImporter(
    absl::string_view cad_import_service_address) {
  grpc::ChannelArguments channel_args = connect::DefaultGrpcChannelArgs();
  // Default send and receive message size is 4MB for gRPC, which is too small
  // for potentially large geometries streamed across CAD import services.
  channel_args.SetMaxReceiveMessageSize(-1);
  channel_args.SetMaxSendMessageSize(-1);
  INTR_ASSIGN_OR_RETURN(auto channel,
                        connect::CreateClientChannel(
                            cad_import_service_address,
                            absl::Now() + absl::Seconds(10), channel_args));
  auto stub = intrinsic_proto::scene::cad::CADImportService::NewStub(
      std::move(channel));
  return std::make_unique<GRPCCADImporter>(std::move(stub));
}

}  // namespace world
}  // namespace intrinsic
