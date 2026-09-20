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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_UTIL_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_UTIL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <variant>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/bitmask_enums.h"  // IWYU pragma: keep
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/world/robot_payload/robot_payload.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::kuka {

constexpr static int kKukaNumJoints = 6;
constexpr static int kKukaMaxDigitalSignals = 64;

// Maximum size of RSI XML telemetry.
constexpr int kTelemetryBufferSize = 4096;
// Maximum length of the RSI XML command.
constexpr int kCommandBufferSize = 4096;
constexpr absl::Duration kOPCUADefaultTimeout = absl::Seconds(15);
constexpr const absl::Duration kDefaultRSITimeoutDelay = absl::Milliseconds(40);
// A valid, relatively centered pose of a KUKA 6-axis robot.
constexpr double kKukaCannonPose[kKukaNumJoints] = {0.0, -1.57, 1.571,
                                                    0.0, 0.0,   0.0};

// Describes the Operation modes of a KUKA robot. See the KUKA manual for more
// information.
enum class OpMode : uint8_t {
  kNone = 0,
  kT1 = 1,
  kT2 = 2,
  kAutomatic = 3,
  kExternal = 4
};
constexpr size_t OpModeCount = 4;

// Enum to specify as a bitmask which error flags are active. Multiple flags can
// easily be combined using operators such as `|` and `&`. See
// intrinsic/icon/utils/bitmask_enums.h for more documentation.
//
// Example:
//   KukaErrorFlagsMask flags = KukaErrorFlagsMask::kMessagesPresent |
//       KukaErrorFlagsMask::kEmergencyStopActiveOrWrongOpMode;
//
enum class KukaErrorFlagsMask {
  kNone = 0,
  // PLC mode cannot distinguish between emergency stop and
  // wrong operation mode.
  kEmergencyStopActiveOrWrongOpMode = BitMask(0),
  // The (external) emergency stop is active, e.g. of the safety cell.
  kEmergencyStopActive = BitMask(1),
  // The internal emergency stop is active, i.e. of the teach pendant.
  kInternalEmergencyStopActive = BitMask(2),
  // There are messages present on the teach pendant, which need to be cleared.
  kMessagesPresent = BitMask(4),
  // The software or protocol version of the Intrinsic option package on the KRC
  // does not match the local version.
  kVersionMismatch = BitMask(5),
  // The operator safety is open, e.g. the door of a safety cell.
  kOperatorSafetyOpen = BitMask(6),
};
std::true_type EnableBitmaskEnum(KukaErrorFlagsMask);

inline absl::string_view KukaErrorFlagsMaskToString(KukaErrorFlagsMask flags) {
  switch (flags) {
    case KukaErrorFlagsMask::kNone:
      return "None";
    case KukaErrorFlagsMask::kEmergencyStopActiveOrWrongOpMode:
      return "EmergencyStopActiveOrWrongOpMode";
    case KukaErrorFlagsMask::kMessagesPresent:
      return "MessagesPresent";
    case KukaErrorFlagsMask::kEmergencyStopActive:
      return "EmergencyStopActive";
    case KukaErrorFlagsMask::kInternalEmergencyStopActive:
      return "InternalEmergencyStopActive";
    case KukaErrorFlagsMask::kVersionMismatch:
      return "VersionMismatch";
    case KukaErrorFlagsMask::kOperatorSafetyOpen:
      return "OperatorSafetyOpen";
    default:
      return "Unknown";
  }
}

inline absl::string_view OpModeToString(OpMode op_mode) {
  switch (op_mode) {
    case OpMode::kNone:
      return "None";
    case OpMode::kT1:
      return "T1";
    case OpMode::kT2:
      return "T2";
    case OpMode::kAutomatic:
      return "Automatic";
    case OpMode::kExternal:
      return "External";
    default:
      return "Unknown";
  }
}

// Kuka RSI works by sending telemetry to a sensor, and then expecting the
// sensor to respond with corrections.  This is the telemetry portion of that
// exchange.
struct RsiTelemetry {
  // The position of each joint, in order, in radians. Initialized to a valid
  // pose for a KUKA 6-axis robot.
  std::array<double, kKukaNumJoints> position = std::to_array(kKukaCannonPose);
  // The effort being applied at each joint, in N.m.
  std::array<double, kKukaNumJoints> torque;
  // Digital inputs received from the robot.
  FixedVector<bool, kKukaMaxDigitalSignals> digital_input;
  // Digital outputs received from the robot (actual state of the command or
  // changed by other sources).
  FixedVector<bool, kKukaMaxDigitalSignals> digital_output;
  // Interpolator counter from the KRC (u64 milliseconds since KRC last booted)
  uint64_t interpolator_counter_ms = 0;
  // Delayed packets reported by the KRC
  uint64_t delayed_packets = 0;
  // Current operation mode.
  OpMode op_mode = OpMode::kNone;
  // Whether the RSI correction is active since it is possible that RSI is
  // active, but the robot cannot be controlled.
  bool rsi_correction_active = false;

  RsiTelemetry() { torque.fill(0.0); }

  // Returns false if this is not a valid Telemetry object (e.g.
  // un-initialized).
  bool IsValid() {
    // The interpolator counter can never realistically be 0 (it would mean
    // that the RSI packet was received at the same exact time the KRC last
    // booted).
    return interpolator_counter_ms != 0;
  }
};

// Kuka RSI works by sending telemetry to a client (i.e this framework ), and
// expecting the client to respond with corrections.  This is the corrections
// portion of that exchange.
struct RSICommand {
  // The position of each robot joint (assumes all rotary), in radians.
  std::array<double, kKukaNumJoints> position{};
  // Digital outputs to send to the robot.
  FixedVector<bool, kKukaMaxDigitalSignals> digital_output{};
  // If this is set to true the KRL script running on the robot will exit
  // gracefully.
  bool exit_rsi{false};
};

// Parameters for the class "KukaPlcClient".
struct KukaPlcClientParams {
  // The OPC-UA address of the server, e.g. "opc.tcp://172.16.14.1:4840"
  std::string opcua_address;
  // Node of the PLC in the OPC-UA server.
  std::string opcua_node_name;
  // Timeout for the opc-ua connection.
  absl::Duration opcua_timeout = kOPCUADefaultTimeout;
};

// Parameters for the class "KukaEkiSystemControl" and "KukaEkiSystemStatus".
struct KukaEkiClientParams {
  // The IP address of the KLI interface on the KUKA KRC.
  std::string eki_address;
  // Port on which the EKI server is listening.
  uint16_t eki_port = 0;
  // Frequency at which the RSI HWM will send requests to the EKI on KRC side.
  double update_frequency = 0;
};

struct KukaConfig {
  static absl::StatusOr<KukaConfig> FromProto(
      const intrinsic_proto::icon::KukaRsiModule& proto_config)
      INTRINSIC_NON_REALTIME_ONLY;
  // The ip of the local NIC (i.e. the machine where this class is running on)
  // to listen on for RSI UDP packages.
  std::string local_rsi_host;
  // The port to listen on for RSI packages.
  uint32_t local_rsi_port;
  // Timeout until the RSI telemetry updates are considered late and the
  // HWM will disable itself.
  absl::Duration rsi_timeout = kDefaultRSITimeoutDelay;
  // Parameters for the control client. If monostate is set, a NoOp control
  // client will be created.
  std::variant<std::monostate, KukaPlcClientParams, KukaEkiClientParams>
      control_client_params;
  // Parameters for the status client. If monostate is set, a NoOp status client
  // will be created.
  std::variant<std::monostate, KukaEkiClientParams> status_client_params;
  // Number of digital inputs expected from the KUKA RSI.
  int32_t num_digital_inputs;
  // Number of digital outputs expected from ICON that will be forwarded to KUKA
  // RSI. The KUKA RSI needs to send the actual values of the digital outputs
  // back as inputs.
  int32_t num_digital_outputs;
};

// Class to generate and parse RSI XML data with real-time guarantees. The
// constructor pre-allocates some XML element/attribute strings (still on the
// stack, but quite a few). Therefore, it is advisable to only instantiates this
// class once and keep it around.
class RSIXML {
 public:
  RSIXML() INTRINSIC_CHECK_REALTIME_SAFE;

  // Extract from the given XML `buffer` the RSI telemetry data.
  icon::RealtimeStatus ParseTelemetryRT(absl::string_view buffer,
                                        RsiTelemetry& telemetry)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Creates the RSI XML string using `generator` and the content from `command`
  // and insert the XML string into `buffer`.
  //
  // Returns ResourceExhaustedError when `buffer` is not large enough to fit the
  // XML string.
  icon::RealtimeStatus AssembleCommandRT(
      const RSICommand& command, uint64_t ipoc,
      FixedVector<char, kCommandBufferSize>& buffer)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Creates `NumberOfStrings` strings with the given `prefix` and incrementing
  // numbers starting from 1. Clears `strings` and the resulting strings are
  // stored in `strings`. For example, if `prefix` is "DI" and `NumberOfStrings`
  // is 3, the strings "DI1", "DI2", "DI3" will be stored in `strings`.
  template <size_t StringSize, size_t NumberOfStrings>
  static void CreateXMLAttributeStrings(
      absl::string_view prefix,
      FixedVector<icon::FixedString<StringSize>, NumberOfStrings>& strings) {
    strings.clear();
    for (size_t i = 0; i < NumberOfStrings; i++) {
      strings.push_back(icon::FixedString<StringSize>(
          icon::RealtimeStatus::StrCat(prefix, i + 1)));
    }
  }

 private:
  RealtimeXmlGenerator generator_ = RealtimeXmlGenerator(10, 100);
  FixedVector<icon::FixedString<5>, kKukaMaxDigitalSignals> digital_outputs_;
  FixedVector<icon::FixedString<5>, kKukaMaxDigitalSignals> digital_inputs_;
  FixedVector<icon::FixedString<3>, kKukaNumJoints> joint_axis_names_;
};

// Payload of a KUKA robot in the format as received from the KUKA KRC.
struct KukaPayload {
  // Mass of the payload in kg.
  double mass = 0.0;
  // Center of gravity of the payload in m.
  double center_of_gravity_x = 0.0;
  double center_of_gravity_y = 0.0;
  double center_of_gravity_z = 0.0;
  // Rotation of inertia matrix at center of gravity. KUKA is using ZY'X''
  // order for ABC Euler angles (intrinsic Tait–Bryan to be more precise) (i.e.
  // A=Z, B=Y, C=X). Values in degree.
  double center_of_gravity_a = 0.0;
  double center_of_gravity_b = 0.0;
  double center_of_gravity_c = 0.0;
  // Inertia values at center of gravity. Values in kgm^2.
  double inertia_xx = 0.0;
  double inertia_yy = 0.0;
  double inertia_zz = 0.0;

  absl::Status IsApprox(const KukaPayload& other, double tolerance = 1e-5);
};

std::string ToString(const KukaPayload& payload);

// Converts an Intrinsic RobotPayload type to a KukaPayload type. Transforms the
// inertial matrix to principal moments and adds the rotation of the inertial
// matrix to the center of gravity.
absl::StatusOr<KukaPayload> FromRobotPayload(const RobotPayloadBase& payload);
// Converts a KukaPayload type to an Intrinsic RobotPayload type and checks that
// the payload is valid (e.g. inertial matrix is positive definite).
absl::StatusOr<RobotPayload> ToRobotPayload(const KukaPayload& payload);

class Version {
 public:
  using VersionTuple = std::tuple<size_t, size_t, size_t>;

  explicit Version(const VersionTuple& version) : version_(version) {}

  size_t Major() const { return std::get<0>(version_); }
  size_t Minor() const { return std::get<1>(version_); }
  size_t Patch() const { return std::get<2>(version_); }

  // Creates a Version object from a firmware string in the form of `1.2.3`.
  static absl::StatusOr<Version> CreateFromString(absl::string_view firmware);

  // Prints the Version object in the form `V1.2.3.4`.
  std::string ToString() const;

  // Returns OK when `expected_version` and  `actual_version` are of equal minor
  // and major versions. The expected format is `x.y.z`, where `x` is major, `y`
  // is minor and `z` is the patch version. `version_type` is used to provide a
  // better description of the version in return error messages, e.g. "protocol"
  // or "software". Different patch versions are treated to be an equal version,
  // but a log message will be printed to warn that the version is not exactly
  // the same.
  //
  // Examples for equality:
  // - 1.2.3 vs 1.2.3
  // - 1.2.4 vs 1.2.3
  // Examples for inequality:
  // - 1.2.3 vs 1.1.3
  // - 1.2.3 vs 2.2.3
  //
  // Usage example:
  // ```
  // if(auto status = IsCompatibleVersion("protocol", "1.2.3", "1.2.3");
  // !status.ok()) {
  //   LOG(ERROR) << status;
  // }
  // ```
  static absl::Status IsCompatibleVersion(absl::string_view version_type,
                                          const Version& expected_version,
                                          const Version& actual_version);

  // Returns OK if the given `actual_version` is at least the `minimum_version`
  // and less than the `first_incompatible_version`. The expected format is
  // `x.y.z`, where `x` is major, `y` is minor and `z` is the patch version.
  // `version_type` is used to provide a better description of the version in
  // the log messages.
  static absl::Status IsInVersionRange(
      absl::string_view version_type, const Version& minimum_version,
      const Version& first_incompatible_version, const Version& actual_version);

  friend bool operator<(const Version& lhs, const Version& rhs) {
    return lhs.version_ < rhs.version_;
  }
  friend bool operator>(const Version& lhs, const Version& rhs) {
    return rhs < lhs;
  }
  friend bool operator<=(const Version& lhs, const Version& rhs) {
    return !(lhs > rhs);
  }
  friend bool operator>=(const Version& lhs, const Version& rhs) {
    return !(lhs < rhs);
  }
  friend bool operator==(const Version& lhs, const Version& rhs) {
    return lhs.version_ == rhs.version_;
  }
  friend bool operator!=(const Version& lhs, const Version& rhs) {
    return !(lhs == rhs);
  }
  // Pretty print for logging and check failures.
  friend std::ostream& operator<<(std::ostream& os, const Version& v) {
    return os << v.ToString();
  }

 private:
  VersionTuple version_;
};

}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_RSI_UTIL_H_
