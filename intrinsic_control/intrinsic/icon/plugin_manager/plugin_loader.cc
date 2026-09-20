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

#include "intrinsic/icon/plugin_manager/plugin_loader.h"

#include <dlfcn.h>

#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"

namespace intrinsic::icon {

absl::StatusOr<PluginLoader> PluginLoader::Create(
    const std::string& library_path) {
#ifndef __GNUC__
#error "PluginLoader requires GNU C extensions for dlmopen()."
#endif /* __GNUC__ */
  if (library_path.empty()) {
    return absl::InvalidArgumentError("no file path given");
  }

  dlerror();
  // TODO(b/378456719): Use dlmopen() instead of dlopen() to avoid issues when
  // plugin links libraries of different versions (i.e. absl).
  auto raw_library_ptr = dlopen(library_path.c_str(), RTLD_LOCAL | RTLD_LAZY);
  if (raw_library_ptr == nullptr) {
    return absl::InternalError(
        absl::StrCat("unable to open library: ", dlerror()));
  }

  PluginLoaderUniquePtr library_ptr(raw_library_ptr, [](void* raw_library_ptr) {
    if (raw_library_ptr != nullptr) {
      dlclose(raw_library_ptr);
    }
  });

  return PluginLoader(library_path, std::move(library_ptr));
}

PluginLoader::PluginLoader(absl::string_view library_path,
                           PluginLoaderUniquePtr library_ptr)
    : library_path_(library_path), library_ptr_(std::move(library_ptr)) {}

std::string PluginLoader::LibraryPath() const { return library_path_; }

absl::Status PluginLoader::HasSymbol(const std::string& symbol_name) const {
  if (symbol_name.empty()) {
    return absl::InvalidArgumentError("no symbol name given");
  }

  dlerror();
  void* symbol = dlsym(library_ptr_.get(), symbol_name.c_str());
  if (symbol == nullptr) {
    return absl::InvalidArgumentError(
        absl::StrCat("unable to find symbol: ", dlerror()));
  }
  return absl::OkStatus();
}

absl::StatusOr<void*> PluginLoader::GetSymbol(
    const std::string& symbol_name) const {
  auto ret = HasSymbol(symbol_name);
  if (!ret.ok()) {
    return ret;
  }

  // nullptr check is done via `HasSymbol`
  return dlsym(library_ptr_.get(), symbol_name.c_str());
}

}  // namespace intrinsic::icon
