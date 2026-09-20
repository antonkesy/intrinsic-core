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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_eki_system_status.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/algorithm/container.h"
#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kss_messages.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/tcp_server_client.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser.h"
#include "intrinsic/math/units.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/invalid_until_set.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::kuka {

// A list of hand-picked messages that are worth reporting via Flowstate.
// The order of the list represents the priority of each message since only the
// first few will be displayed. A lower index in this array means a higher
// priority.
int32_t kss_message_allow_priority_list[] = {
    // Messages that can happen in normal execution and are of high interest.
    15040,  // Ackn.: Maximum global axis velocity exceeded
    15041,  // Ackn.: Maximum safe reduced Cartesian velocity exceeded
    15039,  // Ackn.: Maximum global Cartesian velocity exceeded
    1074,   // Command motor torque {Axis number}
    1133,   // Maximum gear torque, axis {Axis number}
    15037,  // Cell area exceeded
    117,    // Collision detection axis {Axis number}
    15127,  // Ackn.: Stop because workspace exceeded
    15048,  // Ackn.: Mastering test time interval expired
    27004,  // Brake test required
    // 15047,  // Mastering test required (internal) <-- Do not report: Quite
    // interesting message, but clearing this KUKA fault is optional and leads
    // to false-positives when clearing faults via ICON.

    // Messages that are unlikely and of lower interest, but might still be
    // worth showing.
    26217,  // Ackn. brake cool off time ({Drive}).
    114,    // Workspace no. {Workspace number} violated.
    13068,  // <{Bus instance}> EtherCAT device {Device name} is not connected
            // to bus.
    15034,  // Ackn.: More than one tool activated in the safety controller
    2048,   // Server time limit reached
    1347,   // Robot not mastered
    10046,  // Timeout establishing connection between PLC and {Name}
    112,    // Invalid $TOOL: workspace monitoring not possible
    205,    // Software limit switch {Motion direction} {Axis number}
    3193,   // Ackn. Robot stopped by submit
    29004,  // Internal RSI error
    15135,  // Ackn.: Safety stop before leaving cell area.
    26016,  // Ackn. Position controller in limit ({Drive}).
    14001,  // Ackn. Drive bus power-down
    2858,   // Ackn. Stop due to field bus error
    1362,   // STOP due to operating mode change
    26007,  // Permitted actual velocity exceeded ({Drive}).
    31,     // No connection to RDC possible (bus error, etc.)
    29009,  // Configuration file of the RSI I/Os is invalid.
    // 29005,  // RSI cannot set any outputs due to operator protection <-- Do
    // not report: Interesting, but leads to false positives
    404,    // Safety stop
    15079,  // Monitoring space no. {Number of monitoring space} violated
    29000,  // {Type} Permissible overall correction exceeded: RSI is stopped
    // 200,    // Drives not ready
    29008,  // RSI I/Os configuration file not found.
    15016,  // Ackn.: Stop due to standstill monitoring violation
    26111,  // Warning: Device temperature is too high ({Device type})
            // ({Number}).
    13015,  // <{Bus ID}> Ethercat bus scan error. Device: {Incorrect device}
            // [{Additional info}]
    1211,  // STOP due to software limit switch {Motion direction} {Axis number}
    26173,  // Maximum permissible force exceeded ({Drive}).
    15036,  // Ackn.: No tool activated in safety controller
    12017,  // Operator safety not acknowledged
    13021,  // <{Bus ID}> Ethercat network error. {Details} [{Details}]
    29013,  // RSI cannot communicate with field bus: {Field bus instance}
    3201,   // Following error {Axis number} in torque mode exceeded
    15042,  // Ackn.: Safe reduced axis velocity exceeded
};

// Max allowed length of the EKI xml.
constexpr size_t kMaxEkiXmlLength = 4096;
// Connect timeout for the EKI TCP connection.
constexpr absl::Duration kConnectTimeout = absl::Seconds(2);
// Factor used to multiply the target cycle time with to get the timeout for EKI
// calls.
constexpr double kEkiTimeoutFactor = 10.0;
// Minimum protocol version for the set payload command.
const Version kMinimumSetPayloadProtocolVersion({1, 1, 0});

namespace {
// Orders `element_list` by priorities given by `prio_list`.
// The elements in `prio_list` are given by descending priority, i.e., the first
// element has the highest priority.
// The elements in `element_list` are the elements to order. All elements in
// `element_list` must be contained in `prio_list`, otherwise the element will
// be missing.
//
//  Returns the ordered elements as a vector.
std::vector<int32_t> OrderByPrio(absl::Span<const int32_t> prio_list,
                                 const std::set<int32_t>& elements) {
  // Use std::map here to have ordering of elements.
  std::map<int32_t, int32_t> prio_element_map;

  for (size_t i = 0; i < prio_list.size(); i++) {
    // Check if the current element is part of the query. We iterate over
    // prio_list since elements is a set and find() is more efficient.
    if (absl::c_find(elements, prio_list[i]) != elements.end()) {
      prio_element_map[i] = prio_list[i];
    }
  }
  std::vector<int32_t> ordered_elements;
  ordered_elements.reserve(prio_element_map.size());
  for (auto& [prio, element] : prio_element_map) {
    // This works because prio_element_map stores the keys in order using the
    // comparison function.
    ordered_elements.push_back(element);
  }
  return ordered_elements;
}

}  // namespace

KukaEkiSystemStatus::KukaEkiSystemStatus(
    const KukaEkiClientParams& params,
    std::unique_ptr<icon::TcpClient> tcp_client)
    : update_frequency_(params.update_frequency),
      kss_messages_map_(CreateKssMessageMap()) {
  if (tcp_client != nullptr) {
    tcp_client_ = std::move(tcp_client);
  } else {
    tcp_client_ = std::make_unique<icon::TcpClient>(
        params.eki_address, params.eki_port, "EKI_Status_Connection");
  }
}

absl::StatusOr<std::unique_ptr<KukaEkiSystemStatus>>
KukaEkiSystemStatus::Create(const KukaEkiClientParams& params,
                            std::unique_ptr<icon::TcpClient> tcp_client) {
  std::unique_ptr<KukaEkiSystemStatus> result(
      new KukaEkiSystemStatus(params, std::move(tcp_client)));

  INTR_RETURN_IF_ERROR(result->Init());
  return result;
}

absl::Status KukaEkiSystemStatus::Init() {
  INTR_RETURN_IF_ERROR(tcp_client_->Connect(kConnectTimeout));
  LOG(INFO) << "Connected to EKI, using update frequency of "
            << update_frequency_ << " Hz";
  // The EKI communication is a non-realtime communication. Therefore, this
  // thread does not need any realtime scheduling options.
  intrinsic::ThreadOptions thread_options;
  thread_options.SetName("kuka_eki_status");

  INTR_ASSIGN_OR_RETURN(communication_thread_,
                        CreateRealtimeCapableThread(thread_options, [this] {
                          EkiCommunicationThreadJob();
                        }));
  return absl::OkStatus();
}

KukaEkiSystemStatus::~KukaEkiSystemStatus() {
  shutdown_requested_ = true;
  if (communication_thread_.joinable()) {
    communication_thread_.join();
  }
}

void KukaEkiSystemStatus::EkiCommunicationThreadJob() {
  absl::Duration target_cycle_duration = absl::Seconds(1.0 / update_frequency_);
  absl::Duration reconnect_interval = absl::Seconds(0.5);

  while (!this->shutdown_requested_) {
    auto start = absl::Now();
    // This cleanup lambda makes sures that we keep the desired cycle
    // time (also for all `continue` calls), if possible.
    auto cleanup_sleep = absl::MakeCleanup([target_cycle_duration, start] {
      absl::Duration elapsed_time = absl::Now() - start;
      absl::Duration time_remaining = target_cycle_duration - elapsed_time;
      absl::SleepFor(time_remaining);
    });

    is_connected_ = HandleEKIConnection(reconnect_interval);
    if (!*is_connected_) {
      continue;
    }

    if (!SendEkiRequest()) {
      continue;
    }

    if (auto status =
            ReceiveEkiTelemetry(target_cycle_duration * kEkiTimeoutFactor);
        !status.ok()) {
      if (status.code() == absl::StatusCode::kDeadlineExceeded) {
        is_connected_ = false;
      } else {
        LOG_EVERY_N_SEC(ERROR, 10)
            << "Failed to receive EKI telemetry: " << status;
      }
    }
  }
}

absl::StatusOr<KukaErrorFlagsMask> KukaEkiSystemStatus::ActiveErrorFlags()
    const {
  absl::MutexLock lock(mutex_);
  if (!eki_telemetry_) {
    return KukaErrorFlagsMask::kNone;
  }
  KukaErrorFlagsMask result = KukaErrorFlagsMask::kNone;
  if (eki_telemetry_->emergency_stop_active) {
    result |= KukaErrorFlagsMask::kEmergencyStopActive;
  }
  if (eki_telemetry_->internal_emergency_stop_active) {
    result |= KukaErrorFlagsMask::kInternalEmergencyStopActive;
  }
  // Only report in execution modes that the operator safety is open since it
  // is never reported as closed in T1/T2 by KUKA.
  if (!eki_telemetry_->operator_safety_closed &&
      (eki_telemetry_->op_mode == OpMode::kAutomatic ||
       eki_telemetry_->op_mode == OpMode::kExternal)) {
    result |= KukaErrorFlagsMask::kOperatorSafetyOpen;
  }
  auto check_software_version =
      [&](absl::string_view actual_software_version_string) -> absl::Status {
    INTR_ASSIGN_OR_RETURN(
        auto actual_software_version,
        Version::CreateFromString(actual_software_version_string));

    INTR_RETURN_IF_ERROR(Version::IsInVersionRange(
        "software", kMinimumRsiHwmSoftwareVersion,
        kFirstIncompatibleRsiHwmSoftwareVersion, actual_software_version));

    return absl::OkStatus();
  };
  auto check_protocol_version =
      [&](absl::string_view actual_protocol_version_string) -> absl::Status {
    INTR_ASSIGN_OR_RETURN(
        auto actual_protocol_version,
        Version::CreateFromString(actual_protocol_version_string));
    INTR_RETURN_IF_ERROR(Version::IsInVersionRange(
        "protocol", kMinimumRsiHwmProtocolVersion,
        kFirstIncompatibleRsiHwmProtocolVersion, actual_protocol_version));

    return absl::OkStatus();
  };
  if (auto status = check_software_version(eki_telemetry_->software_version);
      !status.ok()) {
    LOG_EVERY_N_SEC(WARNING, 30) << status;
    result |= KukaErrorFlagsMask::kVersionMismatch;
  }
  if (auto status = check_protocol_version(eki_telemetry_->protocol_version);
      !status.ok()) {
    LOG_EVERY_N_SEC(WARNING, 30) << status;
    result |= KukaErrorFlagsMask::kVersionMismatch;
  }

  return result;
}

bool KukaEkiSystemStatus::HandleEKIConnection(
    absl::Duration& reconnect_interval) {
  if (!tcp_client_->IsConnected()) {
    absl::SleepFor(reconnect_interval);
    if (auto status = tcp_client_->Connect(kConnectTimeout); !status.ok()) {
      LOG(ERROR) << status;
      if (reconnect_interval < absl::Seconds(10)) {
        reconnect_interval *= 2;
      }
    } else {
      reconnect_interval = absl::Seconds(0.5);
    }
    return false;
  }

  return true;
}

absl::StatusOr<std::string> KukaEkiSystemStatus::AssembleEkiXml(
    const EkiCommand& eki_command) const {
  RealtimeXmlGenerator generator(10, 10);
  INTRINSIC_RT_ASSIGN_OR_RETURN(RtXmlElement * root,
                                generator.AddRootElement("Intrinsic"));
  INTRINSIC_RT_RETURN_IF_ERROR(root->AddElement("Update").status());
  if (eki_command.payload.has_value()) {
    INTR_ASSIGN_OR_RETURN(const KukaPayload kuka_payload,
                          FromRobotPayload(eki_command.payload.value()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto payload, root->AddElement("Payload"));
    INTRINSIC_RT_RETURN_IF_ERROR(payload->AddElement("Apply").status());
    INTRINSIC_RT_RETURN_IF_ERROR(
        payload->AddAttributeWithValue("Mass", kuka_payload.mass));
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto com, payload->AddElement("CM"));
    INTRINSIC_RT_RETURN_IF_ERROR(
        com->AddAttributeWithValue("X", kuka_payload.center_of_gravity_x));
    INTRINSIC_RT_RETURN_IF_ERROR(
        com->AddAttributeWithValue("Y", kuka_payload.center_of_gravity_y));
    INTRINSIC_RT_RETURN_IF_ERROR(
        com->AddAttributeWithValue("Z", kuka_payload.center_of_gravity_z));
    INTRINSIC_RT_RETURN_IF_ERROR(
        com->AddAttributeWithValue("A", kuka_payload.center_of_gravity_a));
    INTRINSIC_RT_RETURN_IF_ERROR(
        com->AddAttributeWithValue("B", kuka_payload.center_of_gravity_b));
    INTRINSIC_RT_RETURN_IF_ERROR(
        com->AddAttributeWithValue("C", kuka_payload.center_of_gravity_c));

    INTRINSIC_RT_ASSIGN_OR_RETURN(auto inertia, payload->AddElement("J"));
    INTRINSIC_RT_RETURN_IF_ERROR(
        inertia->AddAttributeWithValue("X", kuka_payload.inertia_xx));
    INTRINSIC_RT_RETURN_IF_ERROR(
        inertia->AddAttributeWithValue("Y", kuka_payload.inertia_yy));
    INTRINSIC_RT_RETURN_IF_ERROR(
        inertia->AddAttributeWithValue("Z", kuka_payload.inertia_zz));
    LOG(INFO) << "Setting payload to:\nmass: " << ToString(kuka_payload);
  }

  std::string output;
  output.resize(kMaxEkiXmlLength);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      size_t size, generator.GenerateXMLString(
                       absl::Span<char>(output.data(), output.size())));
  output.resize(size);
  return output;
}
bool KukaEkiSystemStatus::SendEkiRequest() {
  absl::MutexLock lock(mutex_);
  auto xml_command = AssembleEkiXml(eki_command_);
  // Reset so that we don't send the same command again.
  eki_command_ = {};
  if (!xml_command.ok()) {
    LOG_EVERY_N_SEC(ERROR, 5)
        << "Failed to assemble EKI XML: " << xml_command.status();
    return false;
  }
  if (icon::RealtimeStatus status = tcp_client_->Send(
          absl::Span<const uint8_t>(
              reinterpret_cast<const uint8_t*>(xml_command->data()),
              xml_command->size()),
          MSG_NOSIGNAL  // Avoid getting sig_nopipe
      );
      !status.ok()) {
    LOG_EVERY_N_SEC(ERROR, 5) << "Failed to send to EKI: " << status;
    tcp_client_->Disconnect();
    return false;
  }

  return true;
}

absl::Status KukaEkiSystemStatus::ReceiveEkiTelemetry(absl::Duration timeout) {
  FixedVector<uint8_t, kMaxEkiXmlLength> buffer;

  auto received = tcp_client_->ReceiveContainer(timeout, false, 0, buffer);
  if (!received.ok()) {
    tcp_client_->Disconnect();
    return received;
  } else {
    absl::string_view xml = absl::string_view(
        reinterpret_cast<const char*>(buffer.data()), buffer.size());
    auto telemetry = ParseEkiXml(xml);
    if (telemetry.ok()) {
      LOG_FIRST_N(INFO, 1) << "Received first EKI telemetry of robot model '"
                           << telemetry->robot_model << "'";

      *position_.GetFreeBuffer() = telemetry->position;
      position_.CommitFreeBuffer();
      absl::MutexLock lock(mutex_);
      eki_telemetry_ = *telemetry;
      new_data_arrived_.Signal();
    } else {
      return absl::UnknownError(absl::StrCat(
          "Failed to parse telemetry: ", telemetry.status(), "\nXML:\n", xml));
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<KukaEkiSystemStatus::EkiTelemetry>
KukaEkiSystemStatus::ParseEkiXml(absl::string_view xml) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto root, RealtimeXmlParserElement::ParseDoc(xml, "Robot"));
  EkiTelemetry result;
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto cycle, root.FirstElement("Cycle"));
  INTRINSIC_RT_ASSIGN_OR_RETURN(result.cycle, cycle.TextAsInteger<size_t>());

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto software_version,
                                root.FirstElement("SoftwareVersion"));
  result.software_version = software_version.Text();

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto protocol_version,
                                root.FirstElement("ProtocolVersion"));
  result.protocol_version = protocol_version.Text();

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto robot_model,
                                root.FirstElement("RobotModel"));
  result.robot_model = robot_model.Text();

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto serial_no, root.FirstElement("SerialNo"));
  INTRINSIC_RT_ASSIGN_OR_RETURN(result.serial_no,
                                serial_no.TextAsInteger<uint32_t>());

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto position, root.FirstElement("LastPos"));
  for (size_t i = 0; i < result.position.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        result.position[i],
        position.AttributeAsDouble(absl::StrCat("A", i + 1)));
    result.position[i] = DegToRad(result.position[i]);
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto op_mode, root.FirstElement("OpMode"));
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto op_mode_value,
                                op_mode.TextAsInteger<int32_t>());
  result.op_mode = static_cast<OpMode>(op_mode_value);

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto emergency_stop_active,
                                root.FirstElement("EmergencyStop"));
  result.emergency_stop_active = emergency_stop_active.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto internal_emergency_stop_active,
                                root.FirstElement("EmergencyStopInternal"));
  result.internal_emergency_stop_active =
      internal_emergency_stop_active.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto operator_safety_closed,
                                root.FirstElement("OperatorSafetyClosed"));
  result.operator_safety_closed = operator_safety_closed.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto rsi_running,
                                root.FirstElement("RSIActive"));
  result.rsi_active = rsi_running.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto messages_present,
                                root.FirstElement("MessagesPresent"));
  result.messages_present = messages_present.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto program_active,
                                root.FirstElement("ProgramActive"));
  result.program_active = program_active.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto program_selected,
                                root.FirstElement("ProgramSelected"));
  result.program_selected = program_selected.Text() == "1";

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto message_numbers,
                                root.FirstElement("Messages"));
  auto split = absl::StrSplit(message_numbers.Text(), ';', absl::SkipEmpty());
  for (auto number_string : split) {
    int32_t message_number;
    if (absl::SimpleAtoi(number_string, &message_number)) {
      result.message_numbers.insert(message_number);
    } else {
      return absl::InvalidArgumentError(
          absl::StrCat("Failed to parse message number: ", number_string));
    }
  }

  auto payload = root.FirstElement("Payload");
  // Payload is optional to be backwards compatible.
  if (!payload.ok() && payload.status().code() != absl::StatusCode::kNotFound) {
    // If its another error than NotFound, return it.
    return payload.status();
  } else if (payload.ok()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        result.max_payload_mass,
        payload.value().AttributeAsDouble("MaxPayloadMass", true));
    KukaPayload kuka_payload;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        kuka_payload.mass, payload.value().AttributeAsDouble("Mass", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto com, payload.value().FirstElement("CM"));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.center_of_gravity_x,
                                  com.AttributeAsDouble("X", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.center_of_gravity_y,
                                  com.AttributeAsDouble("Y", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.center_of_gravity_z,
                                  com.AttributeAsDouble("Z", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.center_of_gravity_a,
                                  com.AttributeAsDouble("A", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.center_of_gravity_b,
                                  com.AttributeAsDouble("B", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.center_of_gravity_c,
                                  com.AttributeAsDouble("C", true));

    INTRINSIC_RT_ASSIGN_OR_RETURN(auto inertia,
                                  payload.value().FirstElement("J"));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.inertia_xx,
                                  inertia.AttributeAsDouble("X", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.inertia_yy,
                                  inertia.AttributeAsDouble("Y", true));
    INTRINSIC_RT_ASSIGN_OR_RETURN(kuka_payload.inertia_zz,
                                  inertia.AttributeAsDouble("Z", true));
    result.payload = kuka_payload;
  }
  return result;
}

icon::RealtimeStatusOr<eigenmath::Vectord<kKukaNumJoints>>
KukaEkiSystemStatus::CurrentPosition() const {
  InvalidUntilSet<eigenmath::Vectord<kKukaNumJoints>>* position = nullptr;
  position_.GetActiveBuffer(&position);
  if (position && position->has_value()) {
    return eigenmath::Vectord<kKukaNumJoints>(**position);

  } else {
    return icon::UnavailableError("No position has been reported yet.");
  }
}

absl::StatusOr<OpMode> KukaEkiSystemStatus::CurrentOpMode() const {
  absl::MutexLock lock(mutex_);
  if (!eki_telemetry_) {
    return absl::FailedPreconditionError("EKI telemetry was not received yet");
  }
  if (eki_telemetry_->op_mode != OpMode::kNone) {
    return eki_telemetry_->op_mode;
  }
  return absl::UnavailableError("");
}

std::vector<std::string> KukaEkiSystemStatus::ErrorMessages() const {
  auto kss_message_to_string = [](KssMessage message) {
    return absl::StrCat("KSS", message.message_number, ": ", message.message);
  };
  std::vector<std::string> result;
  if (is_connected_.has_value() && !*is_connected_) {
    // The message will give the user a concrete hint that the connection is
    // not working and to which IP:Port the connection was attempted.
    result.push_back(absl::StrCat("No connection to KUKA EKI at ",
                                  tcp_client_->GetHost(), ":",
                                  tcp_client_->GetPort()));
  }
  absl::MutexLock lock(mutex_);
  // Only report faults when RSI/KRL program is not active.
  if (eki_telemetry_.has_value() &&
      (!eki_telemetry_->rsi_active || !eki_telemetry_->program_active)) {
    for (const int32_t number :
         OrderByPrio(absl::MakeSpan(kss_message_allow_priority_list),
                     eki_telemetry_->message_numbers)) {
      // Look first in CrossMeld, the category for system messages. In theory,
      // those messages numbers are ambiguous.
      constexpr absl::string_view kss_system_message_category = "CrossMeld";
      auto it = kss_messages_map_.find(kss_system_message_category);
      if (it != kss_messages_map_.end()) {
        auto cross_meld_it = it->second.find(number);
        if (cross_meld_it != it->second.end()) {
          result.push_back(kss_message_to_string(cross_meld_it->second));
          continue;
        }
      } else {
        // Other categories are currently not supported since the reporting on
        // the KRC side cannot distinguish between them.
      }
    }
  }
  return result;
}

bool KukaEkiSystemStatus::WaitForNewData(absl::Duration timeout) const {
  const absl::Time deadline = absl::Now() + timeout;
  // We wait for 2 cycles to make sure that the command was executed. In case
  // this happens before the telemetry is received, we set this directly to 2.
  size_t min_expected_cycle_number = 2;
  absl::MutexLock lock(mutex_);
  if (eki_telemetry_.has_value()) {
    // Increase by the current cycle number to make sure that we wait for as
    // much cycles as specified above.
    min_expected_cycle_number += eki_telemetry_->cycle;
  }

  while ((!eki_telemetry_.has_value() ||
          eki_telemetry_->cycle < min_expected_cycle_number) &&
         absl::Now() < deadline) {
    new_data_arrived_.WaitWithDeadline(&mutex_, deadline);
  }
  return eki_telemetry_.has_value() &&
         eki_telemetry_->cycle >= min_expected_cycle_number;
}

// LINT.IfChange(ResetFaultStorage)
void KukaEkiSystemStatus::ResetFaultStorage() {
  absl::MutexLock lock(mutex_);
  if (eki_telemetry_.has_value()) {
    LOG(INFO) << "Resetting faults of EKI telemetry";
    // Set all telemetry data to a state where they do not trigger a fault.
    eki_telemetry_->emergency_stop_active = false;
    eki_telemetry_->internal_emergency_stop_active = false;
    eki_telemetry_->messages_present = false;
    eki_telemetry_->rsi_active = true;
    eki_telemetry_->program_active = true;
    eki_telemetry_->messages_present = false;
    eki_telemetry_->program_selected = true;
    eki_telemetry_->message_numbers.clear();
  }
}
// LINT.ThenChange()

KukaEkiSystemStatus::KssMessageSeverity
KukaEkiSystemStatus::StringToKssMessageSeverity(absl::string_view str) {
  auto lower_case_str = absl::AsciiStrToLower(str);
  if (lower_case_str == "info") {
    return KssMessageSeverity::kInfo;
  } else if (lower_case_str == "state") {
    return KssMessageSeverity::kState;
  } else if (lower_case_str == "acknowledgment") {
    return KssMessageSeverity::kAcknowledgment;
  } else if (lower_case_str == "error") {
    return KssMessageSeverity::kError;
  } else if (lower_case_str == "infooracknowledgment") {
    return KssMessageSeverity::kInfoOrAcknowledgment;
  }
  return KssMessageSeverity::kUnknown;
}

absl::flat_hash_map<
    std::string, absl::flat_hash_map<int32_t, KukaEkiSystemStatus::KssMessage>>
KukaEkiSystemStatus::CreateKssMessageMap() {
  absl::flat_hash_map<std::string, absl::flat_hash_map<int32_t, KssMessage>>
      kss_messages_map;
  for (auto& element : kss_message_data) {
    int32_t message_number;
    if (absl::SimpleAtoi(element[1], &message_number)) {
      absl::Span<int32_t> kss_message_allow_list_span =
          absl::MakeSpan(kss_message_allow_priority_list);
      if (absl::c_find(kss_message_allow_list_span, message_number) !=
          kss_message_allow_list_span.end()) {
        kss_messages_map[element[0]][message_number] =
            KssMessage{.message = std::string(element[2]),
                       .message_number = message_number,
                       .severity = StringToKssMessageSeverity(element[3])};
      }
    } else {
      // If this fails, the generated file contains an error. This should only
      // be possible to happen, when the generated file is updated (which
      // is done manually) and we should fail hard then.
      QCHECK(false) << "Failed to parse KUKA KSS message number: "
                    << element[1];
    }
  }
  return kss_messages_map;
}

absl::Status KukaEkiSystemStatus::SetPayload(const RobotPayloadBase& payload) {
  {
    absl::MutexLock lock(mutex_);
    // We check that the payload is not completely out of the range of the
    // current robot, e.g. if the user used the wrong unit or robot model.
    constexpr double kMaxPayloadMassFactor = 2.0;
    if (eki_telemetry_.has_value() &&
        eki_telemetry_->max_payload_mass.has_value()) {
      const double max_payload_sanity_check_threshold =
          *eki_telemetry_->max_payload_mass * kMaxPayloadMassFactor;
      if (payload.mass() > max_payload_sanity_check_threshold) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Desired payload mass of ", payload.mass(),
            " kg is a lot larger than the maximum payload mass of ",
            *eki_telemetry_->max_payload_mass,
            " kg of the robot (Error threshold: ",
            max_payload_sanity_check_threshold, ")."));
      }
    } else if (!eki_telemetry_.has_value()) {
      return absl::FailedPreconditionError(
          "No EKI telemetry available to check max payload mass. Please wait "
          "until the first EKI packet arrived.");
    } else {
      INTR_ASSIGN_OR_RETURN(
          auto protocol_version,
          Version::CreateFromString(eki_telemetry_->protocol_version));
      if (protocol_version < kMinimumSetPayloadProtocolVersion) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Intrinsic option package too old for setting payload, "
            "update to min. ",
            kMinimumSetPayloadProtocolVersion.ToString(),
            ". Current: ", eki_telemetry_->protocol_version));
      }
    }
    eki_command_.payload = payload;
  }
  // A generous, but not annoying timeout for the payload to be applied.
  auto payloadSettingTimeout = absl::Seconds(5);
  if (!WaitForNewData(payloadSettingTimeout)) {
    return absl::DeadlineExceededError(
        absl::StrCat("No new data arrived from EKI in ", payloadSettingTimeout,
                     " after setting payload."));
  }

  return absl::OkStatus();
}
}  // namespace intrinsic::kuka
