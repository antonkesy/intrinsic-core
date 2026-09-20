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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_SYSTEM_STATUS_FACTORY_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_SYSTEM_STATUS_FACTORY_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_status/kuka_system_status_interface.h"

namespace intrinsic::kuka {

// Creates a `KukaSystemStatusInterface` instance based on the content of
// `config`.
//
// Returns a `KukaEkiSystemControl` instance when
// `config.status_client_params` is contains the EKI config object.
// Otherwise returns `NoOpKukaSystemStatus`.
//
// Forwards errors of the creation function of the
// `KukaSystemStatusInterface` implementations.
absl::StatusOr<std::unique_ptr<KukaSystemStatusInterface>>
CreateKukaSystemStatus(const KukaConfig& config);

}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_SYSTEM_STATUS_FACTORY_H_
