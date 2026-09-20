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

#ifndef INTRINSIC_ICON_CONTROL_C_API_WRAPPERS_ACTION_FACTORY_CONTEXT_WRAPPER_H_
#define INTRINSIC_ICON_CONTROL_C_API_WRAPPERS_ACTION_FACTORY_CONTEXT_WRAPPER_H_

#include <array>
#include <memory>

#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/c_api/c_action_factory_context.h"
#include "intrinsic/icon/control/streaming_io_registry.h"

namespace intrinsic::icon {

// The wrapper class for RealtimeStreamingIoAccess needs to know these to
// retrieve streaming I/O values.
using WrappedCApiStreamingInputRealtimeType =
    std::shared_ptr<IntrinsicIconStreamingInputType>;
using WrappedCApiStreamingOutputRealtimeType =
    std::array<char, StreamingIoRegistry::kMaxStreamingOutputSizeBytes>;

// Casts `action_factory_context` to IntrinsicIconActionFactoryContext*.
IntrinsicIconActionFactoryContext* Wrap(
    ActionFactoryContext* action_factory_context);
// Returns a vtable struct with function pointers that implement the C API for
// an ActionFactoryContext. The functions in the vtable struct expect any
// IntrinsicIconActionFactoryContext* pointer arguments to point to an
// ActionFactoryContext (i.e., to be the output of Wrap() above).
IntrinsicIconActionFactoryContextVtable GetActionFactoryContextVtable();

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_C_API_WRAPPERS_ACTION_FACTORY_CONTEXT_WRAPPER_H_
