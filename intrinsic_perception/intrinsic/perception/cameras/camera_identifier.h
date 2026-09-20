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

#ifndef INTRINSIC_PERCEPTION_CAMERAS_CAMERA_IDENTIFIER_H_
#define INTRINSIC_PERCEPTION_CAMERAS_CAMERA_IDENTIFIER_H_

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "intrinsic/perception/core/image_file_reference.h"

namespace intrinsic::perception {

struct CameraIdentifier {
  struct GenICam {
    std::string device_id;
    auto operator<=>(const GenICam&) const = default;
    template <typename H>
    friend H AbslHashValue(H h, const GenICam& g) {
      return H::combine(std::move(h), g.device_id);
    }
    template <typename Sink>
    friend void AbslStringify(Sink& sink, const GenICam& g) {
      absl::Format(&sink, "genicam:%s", g.device_id);
    }
  };
  struct Ros {
    std::string driver_type;
    std::string device_id;
    auto operator<=>(const Ros&) const = default;
    template <typename H>
    friend H AbslHashValue(H h, const Ros& r) {
      return H::combine(std::move(h), r.driver_type, r.device_id);
    }
    template <typename Sink>
    friend void AbslStringify(Sink& sink, const Ros& r) {
      absl::Format(&sink, "ros:%s:%s", r.driver_type, r.device_id);
    }
  };
  struct FileCamera {
    struct Directory {
      std::string path;
      std::vector<ImageFileReference> sensor_references;
      bool remove_incomplete_frames = false;
      auto operator<=>(const Directory&) const = default;
      template <typename H>
      friend H AbslHashValue(H h, const Directory& d) {
        return H::combine(std::move(h), d.path, d.sensor_references,
                          d.remove_incomplete_frames);
      }
      template <typename Sink>
      friend void AbslStringify(Sink& sink, const Directory& d) {
        absl::Format(
            &sink,
            "file_dir:%s?sensor_references=[%s]&remove_incomplete_frames=%d",
            d.path, absl::StrJoin(d.sensor_references, ","),
            d.remove_incomplete_frames);
      }
    };
    struct Files {
      std::vector<std::vector<ImageFileReference>> sensor_files_lists;
      auto operator<=>(const Files&) const = default;
      template <typename H>
      friend H AbslHashValue(H h, const Files& m) {
        return H::combine(std::move(h), m.sensor_files_lists);
      }
      template <typename Sink>
      friend void AbslStringify(Sink& sink, const Files& m) {
        absl::Format(
            &sink, "file_files:[%s]",
            absl::StrJoin(
                m.sensor_files_lists, ",",
                [](std::string* out,
                   const std::vector<ImageFileReference>& sensor_files) {
                  absl::StrAppend(out, "[", absl::StrJoin(sensor_files, ","),
                                  "]");
                }));
      }
    };
    using Source = std::variant<std::monostate, Directory, Files>;
    template <typename Sink>
    friend void AbslStringify(Sink& sink, const Source& s) {
      std::visit(
          [&sink](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, std::monostate>) {
              absl::Format(&sink, "monostate");
            } else {
              absl::Format(&sink, "%s", absl::StrCat(v));
            }
          },
          s);
    }

    Source source = std::monostate();
    std::optional<int64_t> start_index;
    bool loop_files = false;
    auto operator<=>(const FileCamera&) const = default;
    template <typename H>
    friend H AbslHashValue(H h, const FileCamera& f) {
      return H::combine(std::move(h), f.source, f.start_index, f.loop_files);
    }
    template <typename Sink>
    friend void AbslStringify(Sink& sink, const FileCamera& f) {
      char sep = std::holds_alternative<Directory>(f.source) ? '&' : '?';
      absl::Format(&sink, "%s%cstart_index=%s&loop_files=%d",
                   absl::StrCat(f.source), sep,
                   f.start_index.has_value()
                       ? absl::StrCat(f.start_index.value())
                       : "nullopt",
                   f.loop_files);
    }
  };
  struct FakeGenICam {
    auto operator<=>(const FakeGenICam&) const = default;
    template <typename H>
    friend H AbslHashValue(H h, const FakeGenICam&) {
      return h;
    }
    template <typename Sink>
    friend void AbslStringify(Sink& sink, const FakeGenICam&) {
      absl::Format(&sink, "fake_genicam");
    }
  };
  using Driver = std::variant<std::monostate, GenICam, Ros, FileCamera,
                              FakeGenICam
                              >;
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Driver& d) {
    std::visit(
        [&sink](const auto& v) {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_same_v<T, std::monostate>) {
            absl::Format(&sink, "monostate");
          } else {
            absl::Format(&sink, "%s", absl::StrCat(v));
          }
        },
        d);
  }

  Driver driver = std::monostate();
  auto operator<=>(const CameraIdentifier&) const = default;
  template <typename H>
  friend H AbslHashValue(H h, const CameraIdentifier& c) {
    return H::combine(std::move(h), c.driver);
  }
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const CameraIdentifier& c) {
    absl::Format(&sink, "%s", absl::StrCat(c.driver));
  }
};

std::ostream& operator<<(std::ostream& os,
                         const CameraIdentifier& camera_identifier);

std::string CanonicalString(const CameraIdentifier& identifier);

}  // namespace intrinsic::perception

#endif  // INTRINSIC_PERCEPTION_CAMERAS_CAMERA_IDENTIFIER_H_
