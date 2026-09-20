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

#ifndef INTRINSIC_ICON_CONTROL_SERVICES_ASSEMBLY_SERVICE_H_
#define INTRINSIC_ICON_CONTROL_SERVICES_ASSEMBLY_SERVICE_H_

#include "intrinsic/kinematics/skeleton.h"

namespace intrinsic::icon {

// AssemblyService provides access to the robot's kinematic skeleton.
//
// See assembly_service_impl.cc for configuration details.
class AssemblyService {
 public:
  // Obtain the robot's kinematic skeleton. Returns nullptr if no kinematic
  // skeleton is configured.
  virtual const intrinsic::kinematics::Skeleton* GetSkeletonOrNull() const = 0;

  virtual ~AssemblyService() = default;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_SERVICES_ASSEMBLY_SERVICE_H_
