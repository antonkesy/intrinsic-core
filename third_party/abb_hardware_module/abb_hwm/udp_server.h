#ifndef ABB_HARDWARE_MODULE_ABB_HWM_UDP_SERVER_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_UDP_SERVER_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <iostream>
#include <string>

#include "absl/status/status.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "third_party/abb_hardware_module/abb_hwm/udp_server_interface.h"

namespace abb_hardware_module {

// Basic UDP Server which listens for incoming messages on a specified
// port and filtering for a specified sender IP.
// A message handler callback is provided at Init, and is called in response
// to new incoming messages.
class UDPServer : public UDPServerInterface {
 public:
  UDPServer(const std::string& allowed_client_ip, int port)
      : allowed_client_ip_(inet_addr(allowed_client_ip.c_str())), port_(port) {}

  absl::Status Init(MessageHandler handler) override
      INTRINSIC_NON_REALTIME_ONLY;

  ~UDPServer() { close(sockfd_); }

  void run() override INTRINSIC_NON_REALTIME_ONLY;

  void Stop() override INTRINSIC_NON_REALTIME_ONLY { bShouldDie = true; }

 private:
  int sockfd_;
  struct sockaddr_in server_addr_;
  in_addr_t allowed_client_ip_;
  int port_;
  MessageHandler handler_;
  bool bShouldDie = false;
};

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_UDP_SERVER_H_
