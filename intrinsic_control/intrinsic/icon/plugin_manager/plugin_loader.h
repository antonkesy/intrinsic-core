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

#ifndef INTRINSIC_ICON_PLUGIN_MANAGER_PLUGIN_LOADER_H_
#define INTRINSIC_ICON_PLUGIN_MANAGER_PLUGIN_LOADER_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

namespace intrinsic::icon {

// Loader class for dlopen'ing external libraries.
// Given a path to a shared library (`.so`) file, the `PluginLoader` class
// loads the library via `dlopen` and lets users extract their symbols via
// `dlsym`.
// While it's technically possible to extract C++ name-mangled library symbols,
// we strongly advise to load unmangled symbols implemented via `extern "C"`.
class PluginLoader final {
 public:
  // This class is move-only.
  PluginLoader(const PluginLoader& other) = delete;
  PluginLoader& operator=(const PluginLoader& other) = delete;
  PluginLoader(PluginLoader&& other) noexcept = default;
  PluginLoader& operator=(PluginLoader&& other) noexcept = default;

  // Creates a new Pluginloader object and loads the specified library.
  // Returns `absl::InvalidArgumentError` in case no path is specified or
  // `absl::InternalError` if `dlopen` is unable to load the library.
  static absl::StatusOr<PluginLoader> Create(const std::string& library_path);

  // Returns the path to the loaded library.
  std::string LibraryPath() const;

  // Queries a loaded library for a specific symbol.
  // Returns `absl::InvalidArgumentError` if no symbol is specified,
  // or the specified symbol can't be found within the library.
  // Returns `absl::OkStatus()` indicating that the symbol does exist.
  absl::Status HasSymbol(const std::string& symbol_name) const;

  // Extracts a symbol given its name and extract it via an appropriate template
  // cast.
  // Please be aware that a wrongly specified template argument can lead to
  // crashes or undefined behavior. The template argument has to match the
  // symbols declaration within the library. Extracting C function pointers
  // require a C-Style function cast, i.e. `auto fcn = pl.GetSymbol<int *()>`
  // for a void function returning an int. The symbol is only valid as long as
  // the `PluginLoader` instance is valid. All extracted symbols are UB when
  // used after the destruction of the `PluginLoader` instance.
  // Returns a pointer to the loaded symbol if the symbol exists,
  // returns `absl::InvalidArgument` similar to `HasSymbol` is the symbol does
  // not exist.
  template <class T>
  absl::StatusOr<T> GetSymbol(const std::string& symbol_name) const {
    auto ret = GetSymbol(symbol_name);
    if (!ret.ok()) {
      return ret.status();
    }
    return reinterpret_cast<T>(*ret);
  }

 private:
  // We feature a `unique_ptr` with a custom deleter to guarantee a correct
  // unloading behavior of the library upong destruction.
  using PluginLoaderUniquePtr =
      std::unique_ptr<void, std::function<void(void*)>>;
  PluginLoader(absl::string_view library_path,
               PluginLoaderUniquePtr library_ptr);

  // Extracts a symbol given its name and returns a raw void pointer to it.
  // Unlike the templated `GetSymbol` function, this implementation returns the
  // raw pointer to the symbol as returned by `dlsym`.
  absl::StatusOr<void*> GetSymbol(const std::string& symbol_name) const;

  std::string library_path_;
  PluginLoaderUniquePtr library_ptr_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_PLUGIN_MANAGER_PLUGIN_LOADER_H_
