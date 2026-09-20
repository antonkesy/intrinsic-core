#ifndef ABB_HARDWARE_MODULE_ABB_HWM_EGM_SERVER_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_EGM_SERVER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "third_party/abb_hardware_module/abb_hwm/egm_data.h"
#include "third_party/abb_hardware_module/abb_hwm/udp_server_interface.h"

namespace abb_hardware_module {

// The EGM communication is not realtime safe, so we run it with normal
// priority. The system must be able to handle this anyway.
inline intrinsic::ThreadOptions kEgmCommunicationThreadOptions() {
  return intrinsic::ThreadOptions()
      .SetName("ABB_EGM_Communication")
      .SetRealtimeHighPriorityAndScheduler();
}

// The EgmServer class is responsible for handling the communication with the
// ABB robot controller via EGM. It uses exclusively joint position streaming.
// Any cartesian motion should be handled by the user. The class is designed to
// be used in a realtime context and uses a lock-free swap for shifting data in
// and out of the non-realtime EGM communication. This is quite barebones and
// relies on responsible usage by the caller. It is important to make sure that
// the data is read and written (GetDataFromRobot and SetDataToRobot) at a rate
// closely matching the communication rate of the EGM interface, which is
// normally 250 Hz but can be configured to run slower from the ABB side.
// Typical usage is to create an instance of the class, and then repeatedly call
// GetDataFromRobot and SetDataToRobot in a loop.
class EgmServer {
 private:
  intrinsic::AsyncBuffer<egm_data::EgmSensor> to_robot_;
  intrinsic::AsyncBuffer<egm_data::EgmRobot> from_robot_;
  std::unique_ptr<UDPServerInterface> udp_server_;
  intrinsic::Thread abb_communication_thread_;
  intrinsic::ThreadOptions thread_options_;
  std::string HandleEgmMessage(const std::string& message)
      INTRINSIC_NON_REALTIME_ONLY;

 public:
  EgmServer(std::unique_ptr<UDPServerInterface> udp_server,
            intrinsic::ThreadOptions thread_options =
                kEgmCommunicationThreadOptions())
      : udp_server_(std::move(udp_server)), thread_options_(thread_options) {}
  ~EgmServer() { Stop(); }
  void Stop() INTRINSIC_NON_REALTIME_ONLY;
  absl::Status Init() INTRINSIC_NON_REALTIME_ONLY;
  // Read the latest available data from the robot. Note! This method will
  // return the same value multiple times if no new data has been received from
  // the robot controller since the last call.
  intrinsic::icon::RealtimeStatusOr<egm_data::EgmRobot> GetDataFromRobot()
      INTRINSIC_CHECK_REALTIME_SAFE;
  // Write data to the robot.
  void SetDataToRobot(const egm_data::EgmSensor& egm_sensor)
      INTRINSIC_CHECK_REALTIME_SAFE;
};

absl::StatusOr<std::unique_ptr<EgmServer>> CreateEgmServer(
    const std::string& allowed_client_ip, int port) INTRINSIC_NON_REALTIME_ONLY;

class EgmSequenceChecker {
 private:
  uint32_t last_seqno_;
  int64_t tolerance_;
  uint32_t repeat_count_;

 public:
  EgmSequenceChecker(int64_t tolerance)
      : last_seqno_(0), tolerance_(tolerance), repeat_count_(0) {}

  inline void Init(uint32_t seqno) INTRINSIC_CHECK_REALTIME_SAFE {
    last_seqno_ = seqno;
    repeat_count_ = 0;
  }

  inline intrinsic::icon::RealtimeStatus CheckSequenceNumber(uint32_t seqno)
      INTRINSIC_CHECK_REALTIME_SAFE {
    int64_t seq_diff = static_cast<int32_t>(seqno - last_seqno_);

    intrinsic::icon::RealtimeStatus status = intrinsic::icon::OkStatus();
    if (seq_diff < 0) {
      status = intrinsic::icon::RealtimeStatus(
          absl::StatusCode::kInternal,
          intrinsic::icon::FixedStrCat<
              intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
              "Seq. number decreased. Current:", seqno,
              " Last: ", last_seqno_));
    } else if (seq_diff > tolerance_) {
      status = intrinsic::icon::RealtimeStatus(
          absl::StatusCode::kInternal,
          intrinsic::icon::FixedStrCat<
              intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
              "Seq. number over tolerance. Current:", seqno,
              " Last:", last_seqno_));
    } else if (seq_diff == 0) {
      repeat_count_++;
      if (repeat_count_ > tolerance_ - 1) {
        status = intrinsic::icon::RealtimeStatus(
            absl::StatusCode::kInternal,
            intrinsic::icon::FixedStrCat<
                intrinsic::icon::RealtimeStatus::kMaxMessageLength>(
                "Seq. number repeating too many times. Number: ", seqno,
                " Repeats:", repeat_count_));
      }
    } else {
      repeat_count_ = 0;
    }
    last_seqno_ = seqno;
    return status;
  }

  inline bool IsLastSequenceNumberRepeated() INTRINSIC_CHECK_REALTIME_SAFE {
    return repeat_count_ > 0;
  }
};

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_EGM_SERVER_H_
