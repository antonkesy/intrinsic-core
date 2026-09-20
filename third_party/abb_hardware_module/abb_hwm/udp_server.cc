#include "third_party/abb_hardware_module/abb_hwm/udp_server.h"

#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"

namespace {

class SampleTimeStatisticsCollector {
  constexpr static size_t kMaxSamples = 1000;

 private:
  std::chrono::steady_clock::time_point last_tick_;
  std::vector<float> durations_;
  size_t iter_ = 0;

 public:
  void TickStart() { last_tick_ = std::chrono::steady_clock::now(); }

  void TickStop() {
    float dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - last_tick_)
                       .count();
    // pop the oldest sample if size exceeds kMaxSamples
    if (durations_.size() >= kMaxSamples) {
      durations_.erase(durations_.begin());
    }
    durations_.push_back(dur_ms);
  }
  void LogStats() {
    if (iter_++ % kMaxSamples != 0) return;

    // create a histogram of the durations
    // bins: 0-3 ms, 4-4.5 ms, 4.5-5 ms, 5-6 ms, 6-7 ms, 7-8 ms, 8-9 ms, 9-10 ms
    std::vector<size_t> bins(8, 0);
    for (auto dur : durations_) {
      if (dur < 3.5) {
        bins[0]++;
      } else if (dur < 4.5) {
        bins[1]++;
      } else if (dur < 5) {
        bins[2]++;
      } else if (dur < 6) {
        bins[3]++;
      } else if (dur < 7) {
        bins[4]++;
      } else if (dur < 8) {
        bins[5]++;
      } else if (dur < 9) {
        bins[6]++;
      } else {
        bins[7]++;
      }
    }
    LOG(INFO) << absl::StrFormat("Sample time statistics of last %d samples",
                                 durations_.size());
    LOG(INFO) << absl::StrFormat(" < 3.5 ms: %d, which is %f percent", bins[0],
                                 100.0 * bins[0] / durations_.size());
    LOG(INFO) << absl::StrFormat(" 3.5-4.5 ms: %d, which is %f percent",
                                 bins[1], 100.0 * bins[1] / durations_.size());
    LOG(INFO) << absl::StrFormat(" 4.5-5 ms: %d, which is %f percent", bins[2],
                                 100.0 * bins[2] / durations_.size());
    LOG(INFO) << absl::StrFormat(" 5-6 ms: %d, which is %f percent", bins[3],
                                 100.0 * bins[3] / durations_.size());
    LOG(INFO) << absl::StrFormat(" 6-7 ms: %d, which is %f percent", bins[4],
                                 100.0 * bins[4] / durations_.size());
    LOG(INFO) << absl::StrFormat(" 7-8 ms: %d, which is %f percent", bins[5],
                                 100.0 * bins[5] / durations_.size());
    LOG(INFO) << absl::StrFormat(" 8-9 ms: %d, which is %f percent", bins[6],
                                 100.0 * bins[6] / durations_.size());
    LOG(INFO) << absl::StrFormat(" > 9 ms: %d, which is %f percent", bins[7],
                                 100.0 * bins[7] / durations_.size());
  }
};

}  // namespace

namespace abb_hardware_module {

absl::Status UDPServer::Init(MessageHandler handler) {
  handler_ = handler;
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    return absl::InternalError("Error opening socket");
  }

  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port_);
  server_addr_.sin_addr.s_addr = INADDR_ANY;

  if (bind(sockfd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) <
      0) {
    close(sockfd_);
    return absl::InternalError("Error on binding");
  }
  return absl::OkStatus();
}

void UDPServer::run() {
  LOG(INFO) << "UDP Server listening on port " << port_ << "...";
  char buffer[1400];
  struct sockaddr_in clientAddr;
  socklen_t addr_size = sizeof(clientAddr);

  SampleTimeStatisticsCollector sample_time_collector;
  while (!bShouldDie) {
    sample_time_collector.TickStart();
    int recvMsgSize = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                               (struct sockaddr*)&clientAddr, &addr_size);
    sample_time_collector.TickStop();

    // Uncomment the next line to log sample time statistics
    // iterations sample_time_collector.LogStats();

    if (recvMsgSize < 0) {
      LOG(ERROR) << "Error in recvfrom()";
      continue;
    }

    if (clientAddr.sin_addr.s_addr == allowed_client_ip_) {
      std::string receivedMessage(buffer, recvMsgSize);
      std::string response = handler_(receivedMessage);
      if (!response.empty()) {
        sendto(sockfd_, response.c_str(), response.length(), 0,
               (struct sockaddr*)&clientAddr, addr_size);
      }
    } else {
      LOG(ERROR) << "Received message from unauthorized client: "
                 << inet_ntoa(clientAddr.sin_addr);
    }
  }
}

}  // namespace abb_hardware_module
