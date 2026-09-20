#ifndef ABB_HARDWARE_MODULE_ABB_HWM_UDP_SERVER_INTERFACE_H_
#define ABB_HARDWARE_MODULE_ABB_HWM_UDP_SERVER_INTERFACE_H_

#include <functional>
#include <string>

#include "absl/status/status.h"
#include "intrinsic/icon/testing/realtime_annotations.h"

namespace abb_hardware_module {

class UDPServerInterface {
 public:
  using MessageHandler = std::function<std::string(const std::string&)>;

  virtual absl::Status Init(MessageHandler handler)
      INTRINSIC_NON_REALTIME_ONLY = 0;

  virtual void run() INTRINSIC_NON_REALTIME_ONLY = 0;

  virtual void Stop() INTRINSIC_NON_REALTIME_ONLY = 0;

  virtual ~UDPServerInterface() = default;
};

}  // namespace abb_hardware_module

#endif  // ABB_HARDWARE_MODULE_ABB_HWM_UDP_SERVER_INTERFACE_H_
