#include "third_party/abb_hardware_module/abb_hwm/egm_server.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "third_party/abb_egm/egm.pb.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_serialization.h"
#include "third_party/abb_hardware_module/abb_hwm/udp_server.h"

namespace abb_hardware_module {

intrinsic::icon::RealtimeStatusOr<egm_data::EgmRobot>
EgmServer::GetDataFromRobot() {
  egm_data::EgmRobot* egm_robot = nullptr;
  from_robot_.GetActiveBuffer(&egm_robot);
  if (egm_robot == nullptr || !egm_robot->header.has_value()) {
    return intrinsic::icon::RealtimeStatus(absl::StatusCode::kUnavailable,
                                           "No EgmRobot message available.");
  }
  // Since the `RealtimeStatusOr` contains Eigen types and std::optional,
  // `IsCopyable` evaluates to false and the constructor of `RealtimeStatusOr`
  // requires an rvalue type. As `GetDataFromRobot` is expected to return valid
  // data when called repeatedly without an update from the robot, we can't move
  // it and have to create an explicit copy of the stack-allocated data
  // structure.
  egm_data::EgmRobot explicit_copy{*egm_robot};
  return std::move(explicit_copy);
}

void EgmServer::SetDataToRobot(const egm_data::EgmSensor& egm_sensor) {
  egm_data::EgmSensor* free_buffer = to_robot_.GetFreeBuffer();
  *free_buffer = egm_sensor;
  to_robot_.CommitFreeBuffer();
}

std::string EgmServer::HandleEgmMessage(const std::string& message) {
  abb::egm::EgmRobot egm_robot_proto;
  egm_robot_proto.ParseFromString(message);
  egm_data::EgmRobot* egm_robot = from_robot_.GetFreeBuffer();
  FromProto(egm_robot_proto, egm_robot);
  from_robot_.CommitFreeBuffer();

  egm_data::EgmSensor* egm_sensor = nullptr;
  to_robot_.GetActiveBuffer(&egm_sensor);
  if (egm_sensor != nullptr && egm_sensor->header.has_value()) {
    abb::egm::EgmSensor egm_sensor_proto;
    ToProto(*egm_sensor, &egm_sensor_proto);
    return egm_sensor_proto.SerializeAsString();
  }
  return "";
}

absl::Status EgmServer::Init() {
  LOG(INFO) << "Initializing EGM Server..";
  INTR_RETURN_IF_ERROR(udp_server_->Init([this](const std::string& message) {
    return HandleEgmMessage(message);
  }));
  LOG(INFO) << "Starting EGM Communication Thread..";
  INTR_ASSIGN_OR_RETURN(abb_communication_thread_,
                        CreateRealtimeCapableThread(
                            thread_options_, [this]() { udp_server_->run(); }));
  LOG(INFO) << "EGM Server Initialized.";
  return absl::OkStatus();
}

void EgmServer::Stop() {
  udp_server_->Stop();
  abb_communication_thread_.join();
}

absl::StatusOr<std::unique_ptr<EgmServer>> CreateEgmServer(
    const std::string& allowed_client_ip, int port) {
  std::unique_ptr<UDPServer> udp_server =
      std::make_unique<UDPServer>(allowed_client_ip, port);
  std::unique_ptr<EgmServer> egm_server =
      std::make_unique<EgmServer>(std::move(udp_server));
  INTR_RETURN_IF_ERROR(egm_server->Init()).LogError().SetPrepend()
      << "Failed to initialize the EGM server: ";
  return egm_server;
}

}  // namespace abb_hardware_module
