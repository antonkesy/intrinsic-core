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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_factory.h"

#include <memory>
#include <utility>
#include <variant>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_eki_system_status.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_interface.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kuka {

absl::StatusOr<std::unique_ptr<KukaSystemStatusInterface>>
CreateKukaSystemStatus(const KukaConfig& config) {
  if (std::holds_alternative<KukaEkiClientParams>(
          config.status_client_params)) {
    LOG(INFO) << "Creating KukaEkiSystemStatus";
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<KukaSystemStatusInterface> result,
        KukaEkiSystemStatus::Create(
            std::get<KukaEkiClientParams>(config.status_client_params)));
    return result;
  }
  LOG(INFO) << "Creating NoOpKukaSystemStatus";
  std::unique_ptr<KukaSystemStatusInterface> result(
      std::make_unique<NoOpKukaSystemStatus>());
  return std::move(result);
}

}  // namespace intrinsic::kuka
