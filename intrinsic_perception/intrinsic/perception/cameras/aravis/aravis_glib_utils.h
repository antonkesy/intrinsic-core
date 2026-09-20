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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_ARAVIS_ARAVIS_GLIB_UTILS_H_
#define INTRINSIC_PERCEPTION_CAMERAS_ARAVIS_ARAVIS_GLIB_UTILS_H_

#include <arv.h>

#include <memory>

namespace intrinsic {
namespace perception {

struct GObjectDeleter {
  template <typename T>
  void operator()(T* p) {
    if (p != nullptr) {
      g_clear_object(reinterpret_cast<void**>(&p));
    }
  }

  void operator()(ArvStream* stream) {
    if (stream != nullptr) {
      // Prevents the invocation of callback associated with the stream which
      // might accidentally access the stream.
      arv_stream_set_emit_signals(stream, 0);
      g_clear_object(reinterpret_cast<void**>(&stream));
    }
  }
};

template <typename T>
using GObjectPtr = std::unique_ptr<T, GObjectDeleter>;

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CAMERAS_ARAVIS_ARAVIS_GLIB_UTILS_H_
