#ifndef ABB_HARDWARE_MODULE_ABB_HWM_RWS_CLIENT_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_RWS_CLIENT_H_

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "include/curl/curl.h"
#include "include/curl/easy.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/thread.h"
#include "nlohmann/json.hpp"
#include "third_party/abb_hardware_module/abb_hwm/abb_elog.h"

namespace abb_hardware_module {

using json = nlohmann::json;

template <bool edit_domain, bool motion_domain>
constexpr absl::string_view MastershipRequestUrl()
    INTRINSIC_CHECK_REALTIME_SAFE {
  static_assert(edit_domain || motion_domain,
                "At least one domain must be selected.");
  if (edit_domain && motion_domain) {
    return "/rw/mastership/request";
  } else if (motion_domain) {
    return "/rw/mastership/motion/request";
  }
  return "/rw/mastership/edit/request";
}

template <bool edit_domain, bool motion_domain>
constexpr absl::string_view MastershipStatusUrl()
    INTRINSIC_CHECK_REALTIME_SAFE {
  static_assert(
      edit_domain ^ motion_domain,
      "Exactly one of edit_domain or motion_domain must be selected.");
  if (motion_domain) {
    return "/rw/mastership/motion";
  }
  return "/rw/mastership/edit";
}

// Utility-wrapper to wait for a condition to be met. The provided status will
// be returned if the timeout is reached.
absl::Status WaitFor(std::function<absl::StatusOr<bool>()> condition,
                     absl::Duration timeout, absl::Duration poll_interval,
                     const absl::Status& timeout_status)
    INTRINSIC_NON_REALTIME_ONLY;

// An enum for keeping track of the state of the ABB controller.
enum class AbbControllerState {
  // -init- The robot is starting up. It will shift to state motors off when it
  // has started.
  kInit,
  // -motoroff- The robot is in a standby state where there is no power to the
  // robot's motors. The state has to be shifted to motors on before the robot
  // can move.
  kMotorOff,
  // -motoron- The robot is ready to move, either by jogging or by running
  // programs.
  kMotorOn,
  // -guardstop- The robot is stopped because the safety runchain is opened. For
  // instance, a door to the robot's cell might be open.
  kGuardStop,
  // -emergencystop- The robot is stopped because emergency stop was activated.
  kEmergencyStop,
  // -emergencystopreset- The robot is ready to leave emergency stop state. The
  // emergency stop is no longer activated, but the state transition isn't yet
  // confirmed.
  kEmergencyStopReset,
  // -sysfail- The robot is in a system failure state. Restart required.
  kSysFail
};

enum class AbbOperationMode { kAuto = 0, kMan = 1, kManf = 2 };

enum class StateMachineState : uint8_t {
  kIdle = 0,
  kInitialize = 1,
  kRunRapidRoutine = 2,
  kRunEgmRoutine = 3,
};

// RWSClient is used to communicate with the ABB robot controller via the Robot
// Web Services (RWS) API. Only the 2.0 version of RWS (hosted on Omnicore
// controllers) is supported. Older ABB Controllers (IRC5) host an earlier
// version of RWS which is not supported. Only the small subset of the RWS api
// needed for the ABBHardwareModule is exposed, but support for additional
// functionality can easily be added. RWS is a RESTful API, and the client uses
// the libcurl library to make HTTP requests to the controller. To avoid the
// need for setting up certificates on the Omnicore controller, RwsClient uses
// insecure communication via libcurl. The users needs to assess whether this is
// acceptable for their use case. If not, the libcurl options can be changed to
// use any desired security settings (and any other libcurl options). The RWS
// API is inherently (very) stateful. In particular, several operations require
// Mastership, which must be managed by the caller. It is very important to
// release mastership after use. Until mastership has been explicitly released,
// the controller will refuse to grant mastership to any other client,
// potientially requiring a controller reboot via the teach pendant. The method
// docstrings contain information about mastership requirements.
class RwsClient {
 public:
  RwsClient(const std::string& base_url) : base_url_(base_url) {};
  ~RwsClient();
  absl::Status Init() INTRINSIC_NON_REALTIME_ONLY;

  RwsClient(const RwsClient&) = delete;
  RwsClient& operator=(const RwsClient&) = delete;

  // Ask RWS to provide more information about a given error code.
  std::string InterpretErrorCode(const int code) INTRINSIC_NON_REALTIME_ONLY;

  // Generic Get and Post methods for making requests to the controller.
  absl::StatusOr<json> Get(absl::string_view url) INTRINSIC_NON_REALTIME_ONLY;
  absl::StatusOr<json> Post(absl::string_view url,
                            absl::string_view post_data = "")
      INTRINSIC_NON_REALTIME_ONLY;
  // Set a signal on the controller.
  absl::Status SetSignal(absl::string_view signal,
                         absl::string_view value) INTRINSIC_NON_REALTIME_ONLY;
  // Reset rapid program pointer to main.
  // Mastership is required.
  absl::Status SetProgramPointerToMain() INTRINSIC_NON_REALTIME_ONLY;

  // Start rapid execution.
  // The v=2.1 is supported from RW version 7.2.
  // The API with v=2.1 supports mastership as URL parameter.
  // Controller supports UTF-8 character for this API from RW7.1
  // This API requires mastership on "edit" domain.
  // In case of explicit mastership, client needs to take the mastership only on
  // "edit" domain before running this API. If the client explicitly holds
  // mastership on "motion" domain then the API won't execute successfully.
  absl::Status StartExecution() INTRINSIC_NON_REALTIME_ONLY;
  // Stop Rapid execution on the controller.
  absl::Status StopExecution() INTRINSIC_NON_REALTIME_ONLY;
  // Get the current state of the execution on the controller.
  absl::StatusOr<json> GetExecutionState() INTRINSIC_NON_REALTIME_ONLY;
  // Get the current ABB Controller State
  absl::StatusOr<AbbControllerState> GetControllerState()
      INTRINSIC_NON_REALTIME_ONLY;
  absl::StatusOr<std::string> GetRobotType() INTRINSIC_NON_REALTIME_ONLY;

  absl::Status SetControllerState(AbbControllerState state)
      INTRINSIC_NON_REALTIME_ONLY;

  absl::StatusOr<AbbOperationMode> GetOperationMode()
      INTRINSIC_NON_REALTIME_ONLY;

  absl::Status EnsureMotorsOn() INTRINSIC_NON_REALTIME_ONLY;

  // Note: It is very important to keep track of success of this request, as
  // there are many poorly documented failure modes. For example, if execution
  // on the controller has been started by someone else e.g. the teach pendant,
  // mastership will not be granted.
  template <bool edit_domain = true, bool motion_domain = true>
  absl::Status RequestMastership() INTRINSIC_NON_REALTIME_ONLY {
    INTR_ASSIGN_OR_RETURN(
        auto response,
        Post(MastershipRequestUrl<edit_domain, motion_domain>(), ""),
        _.LogError().SetPrepend() << "Failed to request mastership: ");
    return absl::OkStatus();
  }

  absl::Status ReleaseMastership() INTRINSIC_NON_REALTIME_ONLY;

  // Do we currently have mastership?
  template <bool edit_domain = true, bool motion_domain = false>
  absl::StatusOr<bool> GetMastershipStatus() INTRINSIC_NON_REALTIME_ONLY {
    INTR_ASSIGN_OR_RETURN(
        auto response, Get(MastershipStatusUrl<edit_domain, motion_domain>()),
        _.LogError().SetPrepend() << "Failed to get mastership status:");
    bool we_have_mastership =
        response["state"][0]["mastershipheldbyme"] == "TRUE";
    LOG(INFO) << "GetMastershipStatus: " << we_have_mastership;
    return response["state"][0]["mastershipheldbyme"] == "TRUE";
  }

  // Rapid symbol interaction.
  absl::StatusOr<std::string> GetSymbolValue(absl::string_view symbol_path)
      INTRINSIC_NON_REALTIME_ONLY;
  // Required mastership.
  absl::Status SetSymbolValue(absl::string_view symbol, absl::string_view value)
      INTRINSIC_NON_REALTIME_ONLY;

  bool IsConnected() INTRINSIC_CHECK_REALTIME_SAFE;

  // Get event logs from the ABB controller. You can specify the desiderd level
  // of severeity, default errors only.
  absl::StatusOr<std::vector<AbbEventLogEntry>> GetElogs(
      AbbEventLogEntry::Level level = AbbEventLogEntry::Level::ERROR)
      INTRINSIC_NON_REALTIME_ONLY;

  // =============== STATE MACHINE ADDIN METHODS ===============
  // The following methods are only relevant for usage with ABB controllers that
  // have the RobotWare StateMachine Addin installed and enabled.

  // Attempt to start a new EGM joint motion.
  absl::Status StartEgmJoint() INTRINSIC_NON_REALTIME_ONLY;
  // Stop any ongoing EGM operation.
  absl::Status StopEgm() INTRINSIC_NON_REALTIME_ONLY;
  // Prepare for EGM operations.
  absl::Status InitEgm() INTRINSIC_NON_REALTIME_ONLY;
  // Start streaming EGM data, i.e. getting robot state without returning
  // commands.
  absl::Status StartEgmStream() INTRINSIC_NON_REALTIME_ONLY;
  // Requires mastership on exactly "edit" domain. Will fail if mastership is
  // held on "motion" domain.
  absl::Status StartStateMachine() INTRINSIC_NON_REALTIME_ONLY;

  // =============== END STATE MACHINE ADDIN METHODS ===============

 private:
  static size_t WriteCallback(void* contents, size_t size, size_t nmemb,
                              std::string* userp) INTRINSIC_NON_REALTIME_ONLY {
    userp->append(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
  }
  absl::StatusOr<json> ParseResponseAndCheckForErrors(
      absl::string_view response) INTRINSIC_NON_REALTIME_ONLY;
  absl::Status SetEgmSignals(
      absl::string_view egm_stop, absl::string_view egm_stop_stream,
      absl::string_view egm_start_stream, absl::string_view egm_start_joint,
      absl::string_view egm_start_pose) INTRINSIC_NON_REALTIME_ONLY;
  void ConnectionCheckLoop() INTRINSIC_NON_REALTIME_ONLY;
  void CurlCleanup() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status CurlInit() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status StartConnectionCheckThread() INTRINSIC_NON_REALTIME_ONLY;
  void StopConnectionCheckThread() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status WaitForStateMachineState(StateMachineState target_state)
      INTRINSIC_NON_REALTIME_ONLY;

  CURL* curl_ = nullptr;
  struct curl_slist* accept_header_list_ = nullptr;
  struct curl_slist* content_type_header_list_ = nullptr;
  struct curl_slist* accept_xml_header_list_ = nullptr;
  std::string base_url_;

  std::atomic<bool> is_connected_;
  std::mutex curl_mutex_;
  intrinsic::Thread connection_check_thread_;
  std::atomic<bool> connection_check_thread_running_;
};

absl::StatusOr<std::unique_ptr<RwsClient>> CreateRwsClient(
    absl::string_view ip, int port) INTRINSIC_NON_REALTIME_ONLY;

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_RWS_CLIENT_H_
