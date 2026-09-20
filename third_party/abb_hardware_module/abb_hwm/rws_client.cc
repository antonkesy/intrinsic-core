#include "third_party/abb_hardware_module/abb_hwm/rws_client.h"

#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "include/curl/curl.h"
#include "include/curl/easy.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/thread_options.h"
#include "intrinsic/util/thread/thread_utils.h"
#include "third_party/abb_hardware_module/abb_hwm/abb_elog.h"

namespace {

constexpr int kConnectionCheckIntervalSeconds = 1;
constexpr intrinsic::ThreadOptions kConnectionCheckThreadOptions() {
  return intrinsic::ThreadOptions()
      .SetName("ABB_RWS_ConnectionCheck")
      .SetNormalPriorityAndScheduler();
}

constexpr char kAcceptJsonHeader[] = "accept: application/hal+json;v=2.0";
constexpr char kContentTypeHeader[] =
    "Content-Type: application/x-www-form-urlencoded;v=2.0";
constexpr char kAcceptXmlHeader[] = "accept: application/xhtml+xml;v=2.0";

constexpr absl::string_view kStartExecutionDataParameters =
    "regain=continue&execmode=continue&cycle=asis&condition=none&stopatbp="
    "enabled&alltaskbytsp=true";
constexpr absl::string_view kStartExecutionUrl = "/rw/rapid/execution/start";

constexpr absl::string_view kStopExecutionDataParameters =
    "stopmode=stop&usetsp=normal";
constexpr absl::string_view kStopExecutionUrl = "/rw/rapid/execution/stop";

constexpr absl::string_view kReleaseMastershipUrl = "/rw/mastership/release";

constexpr absl::string_view kExecutionStateUrl = "/rw/rapid/execution";
constexpr absl::string_view kControllerStateUrl = "/rw/panel/ctrl-state";
constexpr absl::string_view kOperationModeUrl = "/rw/panel/opmode";

constexpr absl::string_view kRobotTypeUrl = "/rw/system/robottype";

constexpr absl::string_view kRetCodeBaseUrl = "/rw/retcode?code=";

constexpr absl::string_view kResetProgramPointerUrl =
    "/rw/rapid/execution/resetpp";

constexpr absl::string_view kStateMachineTaskUrl = "/rw/rapid/tasks/T_ROB1";
constexpr absl::string_view kStateMachineEgmModuleUrl =
    "/rw/rapid/tasks/T_ROB1/modules/TRobEGM";
constexpr absl::string_view kStateMachineMainModuleUrl =
    "/rw/rapid/tasks/T_ROB1/modules/TRobMain";

// Get the 15 latest logs from the controller
constexpr absl::string_view kGetElogsUrl =
    "/rw/elog/0?lang=en&order=lifo&limit=15";

constexpr absl::string_view kRapidStateMachineCurrentStateSymbolPath =
    "/RAPID/T_ROB1/TRobMain/current_state";

absl::StatusOr<int> SearchForErrorCodeInXml(absl::string_view xml) {
  std::regex description_regex(R"(<span class="code">([^<]+)</span>)");
  std::smatch match;

  // regex_search needs a string
  std::string xml_str(xml);

  if (std::regex_search(xml_str, match, description_regex)) {
    return std::stoi(match[1].str());
  } else {
    return absl::InvalidArgumentError(
        "Failed to get error code, no code tag found in server response.");
  }
}
// -init- The robot is starting up. It will shift to state motors off when it
// has started. -motoroff- The robot is in a standby state where there is no
// power to the robot's motors. The state has to be shifted to motors on before
// the robot can move. -motoron- The robot is ready to move, either by jogging
// or by running programs. -guardstop- The robot is stopped because the safety
// runchain is opened. For instance, a door to the robot's cell might be open.
// -emergencystop- The robot is stopped because emergency stop was activated.
// -emergencystopreset- The robot is ready to leave emergency stop state. The
// emergency stop is no longer activated, but the state transition isn't yet
// confirmed. -sysfail- The robot is in a system failure state. Restart
// required.
absl::StatusOr<abb_hardware_module::AbbControllerState>
AbbControllerStateFromString(absl::string_view state_str) {
  if (state_str == "init") {
    return abb_hardware_module::AbbControllerState::kInit;
  } else if (state_str == "motoroff") {
    return abb_hardware_module::AbbControllerState::kMotorOff;
  } else if (state_str == "motoron") {
    return abb_hardware_module::AbbControllerState::kMotorOn;
  } else if (state_str == "guardstop") {
    return abb_hardware_module::AbbControllerState::kGuardStop;
  } else if (state_str == "emergencystop") {
    return abb_hardware_module::AbbControllerState::kEmergencyStop;
  } else if (state_str == "emergencystopreset") {
    return abb_hardware_module::AbbControllerState::kEmergencyStopReset;
  } else if (state_str == "sysfail") {
    return abb_hardware_module::AbbControllerState::kSysFail;
  }
  return absl::InvalidArgumentError("Unknown controller state: " +
                                    std::string(state_str));
}

std::string AbbControllerStateToString(
    abb_hardware_module::AbbControllerState state) {
  switch (state) {
    case abb_hardware_module::AbbControllerState::kInit:
      return "init";
    case abb_hardware_module::AbbControllerState::kMotorOff:
      return "motoroff";
    case abb_hardware_module::AbbControllerState::kMotorOn:
      return "motoron";
    case abb_hardware_module::AbbControllerState::kGuardStop:
      return "guardstop";
    case abb_hardware_module::AbbControllerState::kEmergencyStop:
      return "emergencystop";
    case abb_hardware_module::AbbControllerState::kEmergencyStopReset:
      return "emergencystopreset";
    case abb_hardware_module::AbbControllerState::kSysFail:
      return "sysfail";
  }
}

absl::StatusOr<abb_hardware_module::AbbOperationMode>
AbbOperationModeFromString(absl::string_view mode_str) {
  if (mode_str == "AUTO") {
    return abb_hardware_module::AbbOperationMode::kAuto;
  } else if (mode_str == "MAN") {
    return abb_hardware_module::AbbOperationMode::kMan;
  } else if (mode_str == "MANF") {
    return abb_hardware_module::AbbOperationMode::kManf;
  }
  return absl::InvalidArgumentError("Unknown operation mode: " +
                                    std::string(mode_str));
}

}  // namespace

namespace abb_hardware_module {

absl::Status WaitFor(std::function<absl::StatusOr<bool>()> condition,
                     absl::Duration timeout, absl::Duration poll_interval,
                     const absl::Status& timeout_status) {
  absl::Time start_time = absl::Now();
  while (absl::Now() - start_time < timeout) {
    INTR_ASSIGN_OR_RETURN(bool condition_met, condition());
    if (condition_met) {
      return absl::OkStatus();
    }
    absl::SleepFor(poll_interval);
  }
  return timeout_status;
}

absl::Status RwsClient::WaitForStateMachineState(
    StateMachineState target_state) {
  return WaitFor(
      [this, target_state]() -> absl::StatusOr<bool> {
        INTR_ASSIGN_OR_RETURN(
            auto current_state_string,
            GetSymbolValue(kRapidStateMachineCurrentStateSymbolPath));
        StateMachineState current_state =
            static_cast<StateMachineState>(std::stoi(current_state_string));
        return current_state == target_state;
      },
      absl::Seconds(1), absl::Milliseconds(100),
      absl::DeadlineExceededError(
          absl::StrCat("Timeout waiting for state machine state. The requested "
                       "target state was:  ",
                       target_state)));
}

absl::StatusOr<std::vector<AbbEventLogEntry>> RwsClient::GetElogs(
    AbbEventLogEntry::Level level) {
  INTR_ASSIGN_OR_RETURN(auto response, Get(kGetElogsUrl),
                        _.LogError().SetPrepend() << "Failed to get elogs: ");
  return ParseAbbEventLog(response["_embedded"]["resources"], level);
}

absl::StatusOr<std::unique_ptr<RwsClient>> CreateRwsClient(absl::string_view ip,
                                                           int port) {
  std::string base_url = absl::StrCat("https://", ip, ":", port);
  auto client = std::make_unique<RwsClient>(base_url);
  INTR_RETURN_IF_ERROR(client->Init()).LogError().SetPrepend()
      << "Failed to initialize RWS client: ";
  return client;
}

void RwsClient::CurlCleanup() {
  if (curl_ != nullptr) {
    curl_easy_cleanup(curl_);
    curl_ = nullptr;
  }
  if (accept_header_list_ != nullptr) {
    curl_slist_free_all(accept_header_list_);
    accept_header_list_ = nullptr;
  }
  if (content_type_header_list_ != nullptr) {
    curl_slist_free_all(content_type_header_list_);
    content_type_header_list_ = nullptr;
  }
  if (accept_xml_header_list_ != nullptr) {
    curl_slist_free_all(accept_xml_header_list_);
    accept_xml_header_list_ = nullptr;
  }
}

absl::Status RwsClient::CurlInit() {
  if (curl_ != nullptr) {
    return absl::FailedPreconditionError("Curl is already initialized.");
  }

  curl_ = curl_easy_init();
  if (curl_ == nullptr) {
    return absl::InternalError("Failed to initialize curl.");
  }

  // Skip certificate verification for peer and host
  curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, 0L);
  curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, 0L);
  curl_easy_setopt(curl_, CURLOPT_COOKIEFILE, "");  // Enable cookie handling
  curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,
                   WriteCallback);  // Set callback function
  curl_easy_setopt(curl_, CURLOPT_USERPWD, "Default User:robotics");
  curl_easy_setopt(curl_, CURLOPT_FAILONERROR, 0L);
  curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT_MS,
                   300L);  // connect phase: 300ms
  curl_easy_setopt(curl_, CURLOPT_TIMEOUT_MS,
                   1000L);  // requests following connect: 1000ms

  accept_header_list_ =
      curl_slist_append(accept_header_list_, kAcceptJsonHeader);
  content_type_header_list_ =
      curl_slist_append(content_type_header_list_, kContentTypeHeader);
  content_type_header_list_ =
      curl_slist_append(content_type_header_list_, kAcceptJsonHeader);
  accept_xml_header_list_ =
      curl_slist_append(accept_xml_header_list_, kAcceptXmlHeader);

  return absl::OkStatus();
}

absl::Status RwsClient::Init() {
  StopConnectionCheckThread();
  CurlCleanup();
  INTR_RETURN_IF_ERROR(CurlInit());
  is_connected_ = false;
  INTR_RETURN_IF_ERROR(StartConnectionCheckThread());
  INTR_RETURN_IF_ERROR(WaitFor(
      [this]() -> absl::StatusOr<bool> { return is_connected_; },
      absl::Seconds(kConnectionCheckIntervalSeconds), absl::Milliseconds(100),
      absl::DeadlineExceededError(
          "Failed to connect to the ABB controller via RWS.")));
  return absl::OkStatus();
}

RwsClient::~RwsClient() {
  StopConnectionCheckThread();
  CurlCleanup();
}

absl::StatusOr<json> RwsClient::ParseResponseAndCheckForErrors(
    absl::string_view response) {
  json json_data;

  if (response.empty()) {
    // Not that an empty response typically indicates success.
    return json_data;
  }

  try {
    json_data = json::parse(response);
  } catch (json::parse_error& e) {
    // Sometimes RWS returns xml even though we request JSON :)
    // This seems to happen in particular for some (but not all!) cases
    // when we get an error code, so we try to parse the error code from the
    // xml.
    INTR_ASSIGN_OR_RETURN(
        auto error_code, SearchForErrorCodeInXml(response),
        _.LogError().SetCode(absl::StatusCode::kInternal).SetPrepend()
            << "Failed to parse JSON response, and could not find an error "
               "code in the xml response: ");
    return absl::InternalError("RWS returned an error: " +
                               InterpretErrorCode(error_code));
  }

  // ABB Errors may be embedded in the response.
  if (json_data.contains("status")) {
    if (json_data["status"]["code"] < 0) {
      return absl::InternalError(
          "RWS returned an error: " +
          InterpretErrorCode(json_data["status"]["code"]));
    }
  }

  return json_data;
}

absl::StatusOr<json> RwsClient::Get(absl::string_view url) {
  std::string response;
  {
    std::lock_guard lock(curl_mutex_);
    curl_easy_setopt(curl_, CURLOPT_URL, absl::StrCat(base_url_, url).c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, accept_header_list_);

    CURLcode res = curl_easy_perform(curl_);
    if (res != CURLE_OK) {
      auto error_string = std::string(curl_easy_strerror(res));
      return absl::InternalError(absl::StrCat(
          "Curl returned an error: ", error_string, ". URL: ", base_url_, url));
    }
  }

  return ParseResponseAndCheckForErrors(response);
}

std::string RwsClient::InterpretErrorCode(const int code) {
  std::string message;
  std::string response;
  CURLcode res;

  {
    std::lock_guard lock(curl_mutex_);
    const std::string url = absl::StrCat(kRetCodeBaseUrl, code);
    curl_easy_setopt(curl_, CURLOPT_URL, (base_url_ + url).c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);
    // We don't use the Get method since error code queries only support xml
    // returns.
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, accept_xml_header_list_);

    res = curl_easy_perform(curl_);

    message = "Error: " + std::to_string(code) + ": ";

    if (res != CURLE_OK) {
      message += "Failed to get error description, CURL error.";
      LOG(ERROR) << message;
      return message;
    }
  }

  std::regex description_regex(R"(<span class="description">([^<]+)</span>)");
  std::smatch match;

  if (std::regex_search(response, match, description_regex)) {
    message += match[1].str();
  } else {
    message +=
        "Failed to get error description, no description tag found in "
        "server response.";
    LOG(ERROR) << message;
  }
  return message;
}

absl::StatusOr<json> RwsClient::Post(absl::string_view url,
                                     absl::string_view post_data) {
  std::string response;
  {
    std::lock_guard lock(curl_mutex_);
    std::string complete_url = absl::StrCat(base_url_, url);
    curl_easy_setopt(curl_, CURLOPT_URL, complete_url.c_str());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, post_data);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, content_type_header_list_);

    CURLcode res = curl_easy_perform(curl_);
    if (res != CURLE_OK) {
      LOG(ERROR) << "Curl returned an error: " << curl_easy_strerror(res);
      return absl::InternalError("Curl returned an error: " +
                                 std::string(curl_easy_strerror(res)));
    }
  }
  return ParseResponseAndCheckForErrors(response);
}

absl::Status RwsClient::StartStateMachine() {
  // ensure the selected task is the state machine module
  // Note: you must absolutely not rename the state machine task or any of its
  // generated modules on the RAPID side.
  INTR_ASSIGN_OR_RETURN(auto task, Get(kStateMachineTaskUrl),
                        _.LogError().SetPrepend()
                            << "StartStateMachine failed when trying to assert "
                               "presence of task T_ROB1 in ABB Controller: ");
  INTR_ASSIGN_OR_RETURN(auto module, Get(kStateMachineEgmModuleUrl),
                        _.LogError().SetPrepend()
                            << "StartStateMachine failed when trying to assert "
                               "presence of EGM module in ABB Controller: ");
  INTR_ASSIGN_OR_RETURN(auto main_module, Get(kStateMachineMainModuleUrl),
                        _.LogError().SetPrepend()
                            << "StartStateMachine failed when trying to assert "
                               "presence of Main module in ABB Controller: ");

  // Make sure the state machine task is active
  if (task["state"][0]["active"] != "On") {
    return absl::FailedPreconditionError(
        "StartStateMachine failed to start because task T_ROB1 is not active.");
  }

  INTR_RETURN_IF_ERROR(StopExecution()).LogError().SetPrepend()
      << "StartStateMachine attemped to stop execution before starting, but "
         "failed: ";
  INTR_ASSIGN_OR_RETURN(auto exec_state, GetExecutionState());
  std::string execution_state = exec_state["ctrlexecstate"];
  LOG(INFO) << absl::StrCat(
      "Starting the RAPID state machine, current execution state is: ",
      execution_state);
  INTR_RETURN_IF_ERROR(SetProgramPointerToMain()).LogError().SetPrepend()
      << "Failed to start the RAPID state machine, could not set PP to main: ";

  INTR_RETURN_IF_ERROR(EnsureMotorsOn()).LogError().SetPrepend()
      << "Failed to start the RAPID state machine, motors are not on: ";

  INTR_RETURN_IF_ERROR(StartExecution()).LogError().SetPrepend()
      << "Failed to start the RAPID state machine, could not start execution: ";

  // Wait for the execution state to be RUNNING
  INTR_RETURN_IF_ERROR(WaitFor(
      [this]() -> absl::StatusOr<bool> {
        INTR_ASSIGN_OR_RETURN(auto exec_state, GetExecutionState());
        return exec_state["ctrlexecstate"] == "running";
      },
      absl::Seconds(1), absl::Milliseconds(100),
      absl::DeadlineExceededError(
          "Timeout waiting for the RAPID state machine to start.")));

  LOG(INFO) << "RAPID state machine started successfully.";
  return absl::OkStatus();
}

absl::Status RwsClient::SetSignal(absl::string_view signal,
                                  absl::string_view value) {
  std::string url = absl::StrCat("/rw/iosystem/signals/", signal, "/set-value");
  std::string post_data = absl::StrCat("lvalue=", value);

  INTR_ASSIGN_OR_RETURN(auto response, Post(url, post_data),
                        _.LogError().SetPrepend()
                            << "Failed to set signal " << signal << " to "
                            << value << ": ");
  return absl::OkStatus();
}

absl::Status RwsClient::SetEgmSignals(absl::string_view egm_stop,
                                      absl::string_view egm_stop_stream,
                                      absl::string_view egm_start_stream,
                                      absl::string_view egm_start_joint,
                                      absl::string_view egm_start_pose) {
  INTR_RETURN_IF_ERROR(SetSignal("EGM_STOP", egm_stop)).LogError().SetPrepend()
      << "Failed to set EGM_STOP signal: ";
  INTR_RETURN_IF_ERROR(SetSignal("EGM_STOP_STREAM", egm_stop_stream))
          .LogError()
          .SetPrepend()
      << "Failed to set EGM_STOP_STREAM signal: ";
  INTR_RETURN_IF_ERROR(SetSignal("EGM_START_STREAM", egm_start_stream))
          .LogError()
          .SetPrepend()
      << "Failed to set EGM_START_STREAM signal: ";
  INTR_RETURN_IF_ERROR(SetSignal("EGM_START_JOINT", egm_start_joint))
          .LogError()
          .SetPrepend()
      << "Failed to set EGM_START_JOINT signal: ";
  INTR_RETURN_IF_ERROR(SetSignal("EGM_START_POSE", egm_start_pose))
          .LogError()
          .SetPrepend()
      << "Failed to set EGM_START_POSE signal: ";
  return absl::OkStatus();
}

absl::Status RwsClient::InitEgm() {
  INTR_RETURN_IF_ERROR(SetEgmSignals("0", "0", "0", "0", "0"))
          .LogError()
          .SetPrepend()
      << "Failed to initialize EGM: ";
  INTR_RETURN_IF_ERROR(WaitForStateMachineState(StateMachineState::kIdle))
          .LogError()
          .SetPrepend()
      << "Could not verify IDLE mode achieved on controller: ";
  return absl::OkStatus();
}

absl::Status RwsClient::StartEgmJoint() {
  INTR_RETURN_IF_ERROR(SetEgmSignals("0", "0", "0", "1", "0"))
          .LogError()
          .SetPrepend()
      << "Failed to start EGM joint motion: ";
  INTR_RETURN_IF_ERROR(
      WaitForStateMachineState(StateMachineState::kRunEgmRoutine))
          .LogError()
          .SetPrepend()
      << "Could not verify RunEgmRoutine state achieved on controller: ";
  return absl::OkStatus();
}

absl::Status RwsClient::StopEgm() {
  INTR_RETURN_IF_ERROR(SetEgmSignals("1", "1", "0", "0", "0"))
          .LogError()
          .SetPrepend()
      << "Failed to stop EGM: ";
  INTR_RETURN_IF_ERROR(WaitForStateMachineState(StateMachineState::kIdle))
          .LogError()
          .SetPrepend()
      << "Could not verify IDLE mode achieved on controller: ";
  return absl::OkStatus();
}

absl::Status RwsClient::StartEgmStream() {
  INTR_RETURN_IF_ERROR(SetEgmSignals("0", "0", "1", "0", "0"))
          .LogError()
          .SetPrepend()
      << "Failed to start EGM stream: ";
  INTR_RETURN_IF_ERROR(
      WaitForStateMachineState(StateMachineState::kRunEgmRoutine))
          .LogError()
          .SetPrepend()
      << "Could not verify RunEgmRoutine state achieved on controller: ";
  return absl::OkStatus();
}

absl::StatusOr<std::string> RwsClient::GetSymbolValue(
    absl::string_view symbol) {
  std::string url = absl::StrCat("/rw/rapid/symbol", symbol, "/data");

  INTR_ASSIGN_OR_RETURN(auto response, Get(url),
                        _.LogError().SetPrepend()
                            << "Failed to get value of symbol " << symbol
                            << ": ");

  std::string content = response["state"][0]["value"];
  return content;
}

absl::Status RwsClient::SetSymbolValue(absl::string_view symbol,
                                       absl::string_view value) {
  std::string url = absl::StrCat("/rw/rapid/symbol", symbol, "/data");
  std::string post_data = absl::StrCat("value=", value);
  INTR_ASSIGN_OR_RETURN(auto response, Post(url, post_data),
                        _.LogError().SetPrepend()
                            << "Failed to set symbol " << symbol << " to "
                            << value << ": ");
  return absl::OkStatus();
}

absl::Status RwsClient::ReleaseMastership() {
  INTR_ASSIGN_OR_RETURN(auto response, Post(kReleaseMastershipUrl),
                        _.LogError().SetPrepend()
                            << "Failed to release mastership: ");
  return absl::OkStatus();
}

absl::Status RwsClient::SetProgramPointerToMain() {
  INTR_ASSIGN_OR_RETURN(auto response, Post(kResetProgramPointerUrl),
                        _.LogError().SetPrepend()
                            << "Failed to reset program pointer: ");
  return absl::OkStatus();
}

absl::Status RwsClient::StartExecution() {
  INTR_ASSIGN_OR_RETURN(
      auto response, Post(kStartExecutionUrl, kStartExecutionDataParameters),
      _.LogError().SetPrepend() << "Failed to start execution: ");
  return absl::OkStatus();
}

absl::Status RwsClient::StopExecution() {
  INTR_ASSIGN_OR_RETURN(
      auto response, Post(kStopExecutionUrl, kStopExecutionDataParameters),
      _.LogError().SetPrepend() << "Failed to stop execution: ");
  return absl::OkStatus();
}

absl::StatusOr<json> RwsClient::GetExecutionState() {
  INTR_ASSIGN_OR_RETURN(auto response, Get(kExecutionStateUrl),
                        _.LogError().SetPrepend()
                            << "Failed to get execution state: ");
  return response["state"][0];
}

absl::StatusOr<AbbControllerState> RwsClient::GetControllerState() {
  INTR_ASSIGN_OR_RETURN(auto response, Get(kControllerStateUrl),
                        _.LogError().SetPrepend()
                            << "Failed to get controller state: ");

  std::string ctrl_state_str;
  try {
    ctrl_state_str = response["state"][0]["ctrlstate"].get<std::string>();
  } catch (json::exception& e) {
    return absl::InternalError("Failed to parse motor state: " +
                               std::string(e.what()));
  }

  return AbbControllerStateFromString(ctrl_state_str);
}

absl::StatusOr<std::string> RwsClient::GetRobotType() {
  INTR_ASSIGN_OR_RETURN(json response, Get(kRobotTypeUrl),
                        _.LogError().SetPrepend()
                            << "Failed to get robot type: ");

  std::string robot_type_str;
  try {
    robot_type_str = response["state"][0]["robot-type"].get<std::string>();
  } catch (json::exception& e) {
    return absl::InternalError("Failed to parse robot type: " +
                               std::string(e.what()));
  }

  return robot_type_str;
}

absl::Status RwsClient::SetControllerState(AbbControllerState state) {
  std::string state_str = AbbControllerStateToString(state);
  std::string post_data = absl::StrCat("ctrl-state=", state_str);
  INTR_ASSIGN_OR_RETURN(auto response, Post(kControllerStateUrl, post_data),
                        _.LogError().SetPrepend()
                            << "Failed to set controller state to " << state_str
                            << ": ");
  return absl::OkStatus();
}

absl::Status RwsClient::EnsureMotorsOn() {
  INTR_ASSIGN_OR_RETURN(auto state, GetControllerState(),
                        _.LogError().SetPrepend()
                            << "Failed to EnsureMotorsOn: ");
  if (state == AbbControllerState::kMotorOn) {
    return absl::OkStatus();
  } else {
    LOG(INFO) << "Motors are not on. Trying to turn on "
                 "motors.. ";
    INTR_RETURN_IF_ERROR(SetControllerState(AbbControllerState::kMotorOn))
            .LogError()
            .SetPrepend()
        << "Failed to EnsureMotorsOn: ";
    INTR_RETURN_IF_ERROR(WaitFor(
        [this]() -> absl::StatusOr<bool> {
          INTR_ASSIGN_OR_RETURN(auto state, GetControllerState());
          return state == AbbControllerState::kMotorOn;
        },
        absl::Seconds(1), absl::Milliseconds(100),
        absl::DeadlineExceededError("Failed to EnsureMotorsOn. Timeout waiting "
                                    "for motors to be on after "
                                    "setting controller state.")));
    LOG(INFO) << "Motors are now on.";
  }
  return absl::OkStatus();
}

absl::StatusOr<AbbOperationMode> RwsClient::GetOperationMode() {
  INTR_ASSIGN_OR_RETURN(auto response, Get(kOperationModeUrl),
                        _.LogError().SetPrepend()
                            << "Failed to get operation mode: ");
  try {
    std::string opmode = response["state"][0]["opmode"].get<std::string>();
    return AbbOperationModeFromString(opmode);
  } catch (json::exception& e) {
    return absl::InternalError("Failed to parse operation mode: " +
                               std::string(e.what()));
  }
}

void RwsClient::ConnectionCheckLoop() {
  while (connection_check_thread_running_) {
    auto response = GetExecutionState();
    if (response.ok()) {
      is_connected_ = true;
    } else {
      is_connected_ = false;
    }
    absl::SleepFor(absl::Seconds(kConnectionCheckIntervalSeconds));
  }
}

absl::Status RwsClient::StartConnectionCheckThread() {
  if (connection_check_thread_running_) {
    // Already running
    return absl::OkStatus();
  }
  if (connection_check_thread_.joinable()) {
    connection_check_thread_.join();
  }
  connection_check_thread_running_ = true;
  INTR_ASSIGN_OR_RETURN(
      connection_check_thread_,
      intrinsic::CreateThread(kConnectionCheckThreadOptions(),
                              [this]() { return ConnectionCheckLoop(); }),
      _.LogError().SetPrepend()
          << "Failed to create connection check thread: ");
  return absl::OkStatus();
}

void RwsClient::StopConnectionCheckThread() {
  if (!connection_check_thread_running_) {
    return;
  }
  connection_check_thread_running_ = false;
  if (connection_check_thread_.joinable()) {
    connection_check_thread_.join();
  }
}

bool RwsClient::IsConnected() { return is_connected_; }

}  // namespace abb_hardware_module
