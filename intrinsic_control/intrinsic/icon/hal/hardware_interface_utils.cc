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

#include "intrinsic/icon/hal/hardware_interface_utils.h"

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include <filesystem>  // NOLINT
#include <string>
#include <string_view>
#include <system_error>  // NOLINT
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/domain_socket_utils.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/segment_header.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon::hal {
namespace {

static constexpr absl::Duration kConnectionTimeout = absl::ZeroDuration();

absl::StatusOr<SegmentHeader*> GetInterfaceHeader(absl::string_view memory_name,
                                                  int shm_fd) {
  if (shm_fd == -1) {
    return absl::InternalError(
        absl::StrCat("File descriptor for ", memory_name, " is invalid."));
  }
  uint8_t* data = static_cast<uint8_t*>(mmap(nullptr, sizeof(SegmentHeader),
                                             PROT_WRITE | PROT_READ, MAP_SHARED,
                                             shm_fd, 0));
  if (data == nullptr || data == MAP_FAILED) {
    return absl::InternalError(
        absl::StrCat("Unable to map shared memory segment: ", memory_name, "[",
                     strerror(errno), "]"));
  }

  return reinterpret_cast<SegmentHeader*>(data);
}

// Returns a map of interface names to type names for the given `module_name`.
// Returns DeadlineExceededError when no such module exists.
// Returns an empty map if the module doesn't expose any interfaces.
absl::StatusOr<absl::flat_hash_map<std::string, std::string>>
FindInterfacesToTypes(std::string_view memory_namespace,
                      absl::string_view module_name) {
  absl::flat_hash_map<std::string, std::string> interfaces_with_types;

  INTR_ASSIGN_OR_RETURN(const auto segment_name_to_file_descriptor_map,
                        ::intrinsic::icon::GetSegmentNameToFileDescriptorMap(
                            SocketDirectoryFromNamespace(memory_namespace),
                            module_name, kConnectionTimeout));

  for (auto& [name, fd] : segment_name_to_file_descriptor_map) {
    INTR_ASSIGN_OR_RETURN(auto interface_header, GetInterfaceHeader(name, fd));
    interfaces_with_types.insert(
        {name, std::string(interface_header->Type().TypeID())});
  }

  return interfaces_with_types;
}

// Returns all file stems in the given directory that have the domain socket
// suffix.
absl::StatusOr<std::vector<std::string>> FindSocketStemsInDirectory(
    absl::string_view socket_directory) noexcept {
  std::vector<std::string> modules;
  std::error_code error;
  for (const auto& entry :
       std::filesystem::directory_iterator(socket_directory, error)) {
    if (entry.path().extension() == domain_socket_internal::kSocketSuffix) {
      modules.push_back(std::string(entry.path().stem()));
    }
  }
  if (error) {
    return absl::NotFoundError(absl::StrCat(
        "Failed to list directory ", socket_directory, ": ", error.message()));
  }
  return modules;
}

}  // namespace

absl::StatusOr<std::vector<std::string>> FindModuleNames(
    std::string_view memory_namespace) {
  std::vector<std::string> modules;
  std::string socket_directory = SocketDirectoryFromNamespace(memory_namespace);
  return FindSocketStemsInDirectory(socket_directory);
}

absl::StatusOr<absl::flat_hash_map<
    std::string, absl::flat_hash_map<std::string, std::string>>>
FindModulesAndInterfacesWithTypes(std::string_view memory_namespace) {
  INTR_ASSIGN_OR_RETURN(const auto module_names,
                        FindModuleNames(memory_namespace));

  absl::flat_hash_map<std::string,
                      absl::flat_hash_map<std::string, std::string>>
      modules_and_interfaces_with_types;
  modules_and_interfaces_with_types.reserve(module_names.size());

  for (const auto& module : module_names) {
    const auto interface_to_type =
        FindInterfacesToTypes(memory_namespace, module);
    if (!interface_to_type.ok()) {
      LOG(WARNING) << "Failed to find interfaces for module '" << module
                   << "': " << interface_to_type.status();
      continue;
    }
    modules_and_interfaces_with_types.insert(
        {module, interface_to_type.value()});
    LOG(INFO) << "Found module '" << module << "' with "
              << interface_to_type.value().size() << " interfaces.";
  }
  return modules_and_interfaces_with_types;
}

}  // namespace intrinsic::icon::hal
