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

#include "intrinsic/perception/calibration/charuco_pattern_detector.h"
#include "intrinsic/perception/calibration/pattern_detector_interface.h"

// This registers the factory for the CharucoPatternDetector.
// We have this separate cc file so no header file gets the linker flag
// alwayslink=True, which would have far reaching effects and is discouraged.
namespace intrinsic::perception {
namespace {

REGISTER_PATTERN_DETECTOR_INTERFACE(CharucoPatternDetector,
                                    "charuco_pattern_detection_config",
                                    CharucoPatternDetector::Create);
}
}  // namespace intrinsic::perception
