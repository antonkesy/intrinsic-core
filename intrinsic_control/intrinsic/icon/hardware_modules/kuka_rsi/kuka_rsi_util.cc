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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/icon/utils/xml/realtime_xml_generator.h"
#include "intrinsic/icon/utils/xml/realtime_xml_parser.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/inertia_utils.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/units.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/proto_time.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/robot_payload/robot_payload.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::kuka {

namespace {
absl::StatusOr<KukaEkiClientParams> FromProto(
    const intrinsic_proto::icon::KukaEkiClientParams& params)
    INTRINSIC_NON_REALTIME_ONLY {
  if (params.eki_address().empty()) {
    return absl::InvalidArgumentError("EKI host address is empty");
  }
  if (params.eki_port() == 0 || params.eki_port() > 65535) {
    return absl::OutOfRangeError(
        absl::StrCat("EKI port ", params.eki_port(),
                     " is invalid, choose a value in the range of [1, 65535]"));
  }
  if (params.update_frequency() <= 0 || params.update_frequency() > 15) {
    return absl::OutOfRangeError(
        absl::StrCat("EKI update frequency of ", params.update_frequency(),
                     " is invalid, choose a value in the range of (0, 15]"));
  }
  return KukaEkiClientParams{
      .eki_address = params.eki_address(),
      .eki_port = static_cast<uint16_t>(params.eki_port()),
      .update_frequency = params.update_frequency()};
}

}  // namespace

absl::StatusOr<KukaConfig> KukaConfig::FromProto(
    const intrinsic_proto::icon::KukaRsiModule& proto_config)
    INTRINSIC_NON_REALTIME_ONLY {
  if (!proto_config.has_local_rsi_host()) {
    return absl::InvalidArgumentError(
        "'local_rsi_host' is missing from the KUKA RSI configuration");
  }
  if (!proto_config.has_local_rsi_port()) {
    return absl::InvalidArgumentError(
        "'local_rsi_port' is missing from the KUKA RSI configuration");
  }
  std::variant<std::monostate, KukaPlcClientParams, KukaEkiClientParams>
      control_client_params;
  if (proto_config.has_plc_client_params()) {
    absl::Duration timeout = kOPCUADefaultTimeout;
    if (proto_config.plc_client_params().has_plc_opcua_timeout()) {
      INTR_ASSIGN_OR_RETURN(
          timeout,
          ToAbslDuration(proto_config.plc_client_params().plc_opcua_timeout()));
    }
    control_client_params = KukaPlcClientParams{
        .opcua_address = proto_config.plc_client_params().plc_opcua_address(),
        .opcua_node_name =
            proto_config.plc_client_params().plc_opcua_node_name(),
        .opcua_timeout = timeout};
  } else if (proto_config.has_eki_control_client_params()) {
    INTR_ASSIGN_OR_RETURN(
        control_client_params,
        kuka::FromProto(proto_config.eki_control_client_params()));
  }

  std::variant<std::monostate, KukaEkiClientParams> status_client_params;
  if (proto_config.has_eki_status_client_params()) {
    INTR_ASSIGN_OR_RETURN(
        status_client_params,
        kuka::FromProto(proto_config.eki_status_client_params()));
  }
  std::optional<absl::Duration> rsi_timeout;
  if (proto_config.has_rsi_timeout()) {
    INTR_ASSIGN_OR_RETURN(rsi_timeout,
                          ToAbslDuration(proto_config.rsi_timeout()));
  }
  if (proto_config.digital_output_names_size() > kKukaMaxDigitalSignals) {
    return absl::OutOfRangeError(
        absl::StrCat(proto_config.digital_output_names_size(),
                     " digital outputs are configured for RSI, but maximally ",
                     kKukaMaxDigitalSignals, " are allowed"));
  }
  if (proto_config.digital_input_names_size() > kKukaMaxDigitalSignals) {
    return absl::OutOfRangeError(
        absl::StrCat(proto_config.digital_input_names_size(),
                     " digital inputs are configured for RSI, but maximally ",
                     kKukaMaxDigitalSignals, " are allowed"));
  }
  return KukaConfig{
      .local_rsi_host = proto_config.local_rsi_host(),
      .local_rsi_port = proto_config.local_rsi_port(),
      .rsi_timeout =
          rsi_timeout.has_value() ? *rsi_timeout : kDefaultRSITimeoutDelay,
      .control_client_params = control_client_params,
      .status_client_params = status_client_params,
      .num_digital_inputs = proto_config.digital_input_names_size(),
      .num_digital_outputs = proto_config.digital_output_names_size(),
  };
}

RSIXML::RSIXML() {
  CreateXMLAttributeStrings("DO", digital_outputs_);
  CreateXMLAttributeStrings("DI", digital_inputs_);
  CreateXMLAttributeStrings("A", joint_axis_names_);
}

icon::RealtimeStatus RSIXML::ParseTelemetryRT(absl::string_view buffer,
                                              RsiTelemetry& telemetry) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const RealtimeXmlParserElement root_element,
      RealtimeXmlParserElement ::ParseDoc(
          absl::string_view(buffer.data(), buffer.size()), "Rob"));
  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(RealtimeXmlParserElement position_element,
                                  root_element.FirstElement("Pos"));
    for (int i = 0; i < telemetry.position.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          telemetry.position[i],
          position_element.AttributeAsDouble(joint_axis_names_.at(i)));
      telemetry.position[i] = DegToRad(telemetry.position[i]);
    }
  }

  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(RealtimeXmlParserElement torque_element,
                                  root_element.FirstElement("Trq"));
    for (int i = 0; i < telemetry.torque.size(); i++) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          telemetry.torque[i],
          torque_element.AttributeAsDouble(joint_axis_names_.at(i)));
    }
  }

  {
    icon::RealtimeStatusOr<RealtimeXmlParserElement> digital_in_element =
        root_element.FirstElement("DigIn");
    telemetry.digital_input.clear();
    if (digital_in_element.ok()) {
      for (int j = 0; j < kKukaMaxDigitalSignals; j++) {
        icon::RealtimeStatusOr<int64_t> value =
            digital_in_element.value().AttributeAsInteger<int64_t>(
                digital_inputs_.at(j), false);

        if (icon::IsNotFound(value.status())) {
          break;
        }

        if (!value.ok()) {
          return value.status();  // Return on other errors.
        }

        if (telemetry.digital_input.size() ==
            telemetry.digital_input.capacity()) {
          return icon::ResourceExhaustedError(icon::RealtimeStatus::StrCat(
              "Received more digital inputs than are reserved (reserved: ",
              telemetry.digital_input.capacity(), ")"));
        }
        telemetry.digital_input.push_back(value.value() == 1);
      }
    } else if (!icon::IsNotFound(digital_in_element.status())) {
      return digital_in_element.status();
    }
  }

  {
    icon::RealtimeStatusOr<RealtimeXmlParserElement> digital_out_element =
        root_element.FirstElement("DigOut");
    telemetry.digital_output.clear();
    if (digital_out_element.ok()) {
      for (int j = 0; j < kKukaMaxDigitalSignals; j++) {
        icon::RealtimeStatusOr<int64_t> value =
            digital_out_element.value().AttributeAsInteger<int64_t>(
                digital_outputs_.at(j), false);

        if (icon::IsNotFound(value.status())) {
          break;
        }

        if (!value.ok()) {
          return value.status();  // Return on other errors.
        }

        if (telemetry.digital_output.size() ==
            telemetry.digital_output.capacity()) {
          return icon::ResourceExhaustedError(icon::RealtimeStatus::StrCat(
              "Received more digital outputs than are reserved (reserved: ",
              telemetry.digital_output.capacity(), ")"));
        }
        telemetry.digital_output.push_back(value.value() == 1);
      }
    } else if (!icon::IsNotFound(digital_out_element.status())) {
      return digital_out_element.status();
    }
  }

  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(RealtimeXmlParserElement mode_op_element,
                                  root_element.FirstElement("ModeOp"));
    INTRINSIC_RT_ASSIGN_OR_RETURN(int64_t op_mode_int,
                                  mode_op_element.TextAsInteger<int64_t>());
    telemetry.op_mode = static_cast<OpMode>(op_mode_int);
  }

  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(RealtimeXmlParserElement ipoc_element,
                                  root_element.FirstElement("IPOC"));
    INTRINSIC_RT_ASSIGN_OR_RETURN(telemetry.interpolator_counter_ms,
                                  ipoc_element.TextAsInteger<uint64_t>());
  }

  {
    auto rsi_correction_active_element =
        root_element.FirstElement("RsiCorrectionActive", false);
    if (rsi_correction_active_element.ok()) {
      telemetry.rsi_correction_active =
          rsi_correction_active_element.value().Text() == "1";
    } else {
      // If not available, assume it is running to avoid making the system
      // completely unusable.
      telemetry.rsi_correction_active = true;
    }
  }

  {
    INTRINSIC_RT_ASSIGN_OR_RETURN(RealtimeXmlParserElement delay_element,
                                  root_element.FirstElement("Delay"));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        telemetry.delayed_packets,
        delay_element.AttributeAsInteger<uint64_t>("D"));
  }

  return icon::OkStatus();
}

icon::RealtimeStatus RSIXML::AssembleCommandRT(
    const RSICommand& command, const uint64_t ipoc,
    FixedVector<char, kCommandBufferSize>& buffer) {
  generator_.Clear();

  INTRINSIC_RT_ASSIGN_OR_RETURN(RtXmlElement * root_node,
                                generator_.AddRootElement("Sen"));
  INTRINSIC_RT_RETURN_IF_ERROR(
      root_node->AddAttributeWithValue("Type", "Intrinsic"));
  INTRINSIC_RT_ASSIGN_OR_RETURN(RtXmlElement * ak_node,
                                root_node->AddElement("AK"));
  for (int i = 0; i < command.position.size(); i++) {
    INTRINSIC_RT_RETURN_IF_ERROR(ak_node->AddAttributeWithValue(
        joint_axis_names_.at(i), RadToDeg(command.position[i])));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(RtXmlElement * dio_node,
                                root_node->AddElement("DigOut"));
  for (int i = 0; i < command.digital_output.size(); i++) {
    INTRINSIC_RT_RETURN_IF_ERROR(dio_node->AddAttributeWithValue(
        digital_outputs_.at(i), command.digital_output[i] ? "1" : "0"));
  }

  INTRINSIC_RT_RETURN_IF_ERROR(
      root_node->AddElementWithText("STOP", command.exit_rsi ? "1" : "0")
          .status());

  INTRINSIC_RT_RETURN_IF_ERROR(
      root_node->AddElementWithText("IPOC", ipoc).status());
  buffer.resize(buffer.capacity());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      size_t size, generator_.GenerateXMLString(
                       absl::Span<char>(buffer.data(), buffer.capacity())));
  buffer.resize(size);
  return icon::OkStatus();
}

absl::StatusOr<KukaPayload> FromRobotPayload(const RobotPayloadBase& payload) {
  KukaPayload kuka_payload;
  kuka_payload.mass = payload.mass();
  constexpr double m2mm = 1e3;
  eigenmath::Vector3d cog = payload.tip_t_cog().translation() * m2mm;
  kuka_payload.center_of_gravity_x = cog.x();
  kuka_payload.center_of_gravity_y = cog.y();
  kuka_payload.center_of_gravity_z = cog.z();

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto principal_inertia_moments,
      TransformToPrincipalInertiaMoments(payload.inertia()));
  const eigenmath::Matrix3d flange_r_principal_inertia =
      payload.tip_t_cog().rotationMatrix() * principal_inertia_moments.rotation;
  const eigenmath::Vector3d euler_angles =
      flange_r_principal_inertia
          .canonicalEulerAngles(2, 1, 0)  // Z'YX'' intrinsic Tait-Bryan
          .reverse();  // reverse so that we get the angles in order XZY and not
                       // ZYX.

  constexpr double rad2deg = 180.0 / M_PI;
  kuka_payload.center_of_gravity_a = euler_angles.z() * rad2deg;
  kuka_payload.center_of_gravity_b = euler_angles.y() * rad2deg;
  kuka_payload.center_of_gravity_c = euler_angles.x() * rad2deg;

  kuka_payload.inertia_xx = principal_inertia_moments.moments.x();
  kuka_payload.inertia_yy = principal_inertia_moments.moments.y();
  kuka_payload.inertia_zz = principal_inertia_moments.moments.z();
  return kuka_payload;
}

absl::StatusOr<RobotPayload> ToRobotPayload(const KukaPayload& payload) {
  constexpr double mm2m = 1e-3;
  Eigen::Vector3d cog(payload.center_of_gravity_x * mm2m,
                      payload.center_of_gravity_y * mm2m,
                      payload.center_of_gravity_z * mm2m);
  constexpr double deg2rad = M_PI / 180.0;
  Eigen::Matrix3d rotation_matrix =
      (Eigen::AngleAxisd(payload.center_of_gravity_a * deg2rad,
                         Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(payload.center_of_gravity_b * deg2rad,
                         Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(payload.center_of_gravity_c * deg2rad,
                         Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  eigenmath::Vector3d inertia(payload.inertia_xx, payload.inertia_yy,
                              payload.inertia_zz);
  return RobotPayload::Create(payload.mass, Pose3d(rotation_matrix, cog),
                              inertia.asDiagonal());
}

absl::Status KukaPayload::IsApprox(const KukaPayload& other, double tolerance) {
  if (!intrinsic::AlmostEquals(mass, other.mass, tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload mass does not match: received ", other.mass, " vs ",
        mass));
  }

  if (!intrinsic::AlmostEquals(center_of_gravity_x, other.center_of_gravity_x,
                               tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload center of gravity x does not match: received ",
        other.center_of_gravity_x, " vs ", center_of_gravity_x));
  }
  if (!intrinsic::AlmostEquals(center_of_gravity_y, other.center_of_gravity_y,
                               tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload center of gravity y does not match: received ",
        other.center_of_gravity_y, " vs ", center_of_gravity_y));
  }
  if (!intrinsic::AlmostEquals(center_of_gravity_z, other.center_of_gravity_z,
                               tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload center of gravity z does not match: received ",
        other.center_of_gravity_z, " vs ", center_of_gravity_z));
  }

  if (!intrinsic::AlmostEquals(center_of_gravity_a, other.center_of_gravity_a,
                               tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload center of gravity a does not match: received ",
        other.center_of_gravity_a, " vs ", center_of_gravity_a));
  }
  if (!intrinsic::AlmostEquals(center_of_gravity_b, other.center_of_gravity_b,
                               tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload center of gravity b does not match: received ",
        other.center_of_gravity_b, " vs ", center_of_gravity_b));
  }
  if (!intrinsic::AlmostEquals(center_of_gravity_c, other.center_of_gravity_c,
                               tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload center of gravity c does not match: received ",
        other.center_of_gravity_c, " vs ", center_of_gravity_c));
  }

  if (!intrinsic::AlmostEquals(inertia_xx, other.inertia_xx, tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload inertia_xx does not match: received ",
        other.inertia_xx, " vs ", inertia_xx));
  }
  if (!intrinsic::AlmostEquals(inertia_yy, other.inertia_yy, tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload inertia_yy does not match: received ",
        other.inertia_yy, " vs ", inertia_yy));
  }
  if (!intrinsic::AlmostEquals(inertia_zz, other.inertia_zz, tolerance)) {
    return absl::UnknownError(icon::RealtimeStatus::StrCat(
        "Received payload inertia_zz does not match: received ",
        other.inertia_zz, " vs ", inertia_zz));
  }
  return absl::OkStatus();
}

std::string ToString(const KukaPayload& payload) {
  return absl::StrCat(
      "mass: ", payload.mass, "\ncog (XYZ): ", payload.center_of_gravity_x, " ",
      payload.center_of_gravity_y, " ", payload.center_of_gravity_z,
      "\ncog rotation (ABC):", payload.center_of_gravity_a, " ",
      payload.center_of_gravity_b, " ", payload.center_of_gravity_c,
      "\ninertia (XX,YY,ZZ): ", payload.inertia_xx, " ", payload.inertia_yy,
      " ", payload.inertia_zz);
}

absl::StatusOr<Version> Version::CreateFromString(absl::string_view firmware) {
  VersionTuple parsed;

  std::vector<absl::string_view> v = absl::StrSplit(firmware, '.');
  if (v.size() != 3) {
    return absl::InvalidArgumentError(absl::StrCat(
        "'", firmware, "'", " is not in the expected format '1.2.3'."));
  }
  // Parse ignoring the first char.
  if (absl::string_view tmp = v[0];
      !absl::SimpleAtoi(tmp, &std::get<0>(parsed))) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to parse ", "'", tmp, "'", " as major version number token."));
  }
  if (absl::string_view tmp = v[1];
      !absl::SimpleAtoi(tmp, &std::get<1>(parsed))) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to parse ", "'", tmp, "'", " as minor version number token."));
  }
  if (absl::string_view tmp = v[2];
      !absl::SimpleAtoi(tmp, &std::get<2>(parsed))) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Failed to parse ", "'", tmp, "'", " as patch version number token."));
  }
  return Version(parsed);
}

std::string Version::ToString() const {
  return absl::StrCat(std::get<0>(version_), ".", std::get<1>(version_), ".",
                      std::get<2>(version_));
}

absl::Status Version::IsCompatibleVersion(absl::string_view version_type,
                                          const Version& expected_version,
                                          const Version& actual_version) {
  if (expected_version == actual_version) {
    return absl::OkStatus();
  }
  if (expected_version.Major() != actual_version.Major() ||
      expected_version.Minor() != actual_version.Minor()) {
    return absl::OutOfRangeError(
        absl::StrCat("Wrong Intrinsic option package ", version_type,
                     " version on KUKA controller: ", actual_version.ToString(),
                     " expected: ", expected_version.ToString()));
  }
  LOG_FIRST_N(WARNING, 1) << "Different Intrinsic option package "
                          << version_type
                          << " patch on KUKA controller: " << actual_version
                          << " expected: " << expected_version
                          << ". This should be OK.";
  return absl::OkStatus();
}

absl::Status Version::IsInVersionRange(
    absl::string_view version_type, const Version& minimum_version,
    const Version& first_incompatible_version, const Version& actual_version) {
  if (IsCompatibleVersion(version_type, minimum_version,
                          first_incompatible_version)
          .ok()) {
    return absl::InvalidArgumentError(
        "Minimum version and first incompatible version must not be the "
        "same");
  }

  if (actual_version < minimum_version) {
    return absl::OutOfRangeError(
        absl::StrCat("Intrinsic option package ", version_type,
                     " older than version range [", minimum_version.ToString(),
                     ",", first_incompatible_version.ToString(),
                     "): ", actual_version.ToString()));
  }

  if (actual_version >= first_incompatible_version) {
    return absl::OutOfRangeError(
        absl::StrCat("Intrinsic option package ", version_type,
                     " newer than version range [", minimum_version.ToString(),
                     ",", first_incompatible_version.ToString(),
                     "): ", actual_version.ToString()));
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::kuka
