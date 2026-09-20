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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_factory.h"

#include <memory>
#include <utility>
#include <variant>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_eki_system_control.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_client.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_system_control.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_interface.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kuka {

absl::StatusOr<std::unique_ptr<KukaSystemControlInterface>>
CreateKukaSystemControl(const KukaConfig& config) {
  if (std::holds_alternative<KukaPlcClientParams>(
          config.control_client_params)) {
    LOG(INFO) << "Creating KukaPlcSystemControl";
    INTR_ASSIGN_OR_RETURN(auto client,
                          KukaPlcClient::Create(std::get<KukaPlcClientParams>(
                              config.control_client_params)));
    std::unique_ptr<KukaSystemControlInterface> result(
        std::make_unique<KukaPlcSystemControl>(std::move(client)));
    return result;
  }
  if (std::holds_alternative<KukaEkiClientParams>(
          config.control_client_params)) {
    LOG(INFO) << "Creating KukaEkiSystemControl";
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<KukaSystemControlInterface> result,
        KukaEkiSystemControl::Create(
            std::get<KukaEkiClientParams>(config.control_client_params)));
    return result;
  }
  LOG(INFO) << "Creating NoOpKukaSystemControl";
  std::unique_ptr<KukaSystemControlInterface> result(
      std::make_unique<NoOpKukaSystemControl>());
  return std::move(result);
}

}  // namespace intrinsic::kuka
