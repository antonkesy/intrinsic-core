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

// This executable runs the KUKA RSI Loopback server, which mimics basic
// functionality a KUKA robot (position commands).

#include <cstdint>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_loopback.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/thread.h"

ABSL_FLAG(std::string, host, "127.0.0.1", "IP where the RSI HWM is running.");
ABSL_FLAG(uint16_t, port, 5852, "Port on which this RSI HWM is listening.");
ABSL_FLAG(double, control_frequency, 250,
          "Frequency at which the RSI loopback will publish new data.");
ABSL_FLAG(
    std::string, digital_inputs, "",
    "Comma-separated list of digital input values (1 or 0). Must match the "
    "number as configured on the running robot.");

absl::Status Run(absl::string_view host, uint16_t port,
                 double control_frequency) {
  intrinsic::kuka::KukaRsiLoopback loopback(host, port, control_frequency);
  INTR_RETURN_IF_ERROR(loopback.Prepare());
  std::vector<std::string> digital_inputs = absl::StrSplit(
      absl::GetFlag(FLAGS_digital_inputs), ',', absl::SkipEmpty());
  intrinsic::FixedVector<bool, intrinsic::kuka::kKukaMaxDigitalSignals>
      digital_input_values;
  for (const auto& digital_input : digital_inputs) {
    if (digital_input != "0" && digital_input != "1") {
      return absl::InvalidArgumentError(absl::StrCat(
          "Invalid digital input value: ", digital_input, ". Allowed: 0 or 1"));
    }
    digital_input_values.push_back(digital_input == "1");
  }
  loopback.SetDigitalInputs(digital_input_values);
  intrinsic::Thread loopback_rsi_robot_thread([&loopback] {
    intrinsic::RtLogInitForThisThread();
    CHECK(loopback.Run().ok());
  });
  loopback_rsi_robot_thread.join();
  return absl::OkStatus();
}

int main(int argc, char* argv[]) {
  InitIntrinsic(
      absl::StrCat("usage: ", std::string(argv[0]), " [options]").c_str(), argc,
      argv);
  QCHECK_OK(Run(absl::GetFlag(FLAGS_host), absl::GetFlag(FLAGS_port),
                absl::GetFlag(FLAGS_control_frequency)));
  return 0;
}
