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

#include <signal.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "grpc/grpc_security_constants.h"
#include "grpcpp/security/server_credentials.h"
#include "grpcpp/server.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/support/status.h"
#include "intrinsic/icon/hal/hardware_interface_utils.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/tools/hardware_module_introspection.grpc.pb.h"
#include "intrinsic/icon/hal/tools/hardware_module_introspection.pb.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/status/status_macros_grpc.h"

ABSL_FLAG(int, port, 8080, "port to listen on");
ABSL_FLAG(bool, print_only, false, "Only print the module info.");

namespace {

using ::intrinsic::icon::HardwareModuleProxy;
using ::intrinsic::icon::hal::FindModuleNames;

absl::StatusOr<intrinsic_proto::icon::hal::HardwareModules> ModuleInfo() {
  INTR_ASSIGN_OR_RETURN(std::vector<std::string> module_names,
                        FindModuleNames(/*memory_namespace=*/""));

  std::vector<HardwareModuleProxy> module_proxies;
  for (const std::string& module_name : module_names) {
    auto proxy_or = HardwareModuleProxy::Attach(
        /*shared_memory_namespace=*/"", module_name, absl::ZeroDuration());

    if (!proxy_or.ok()) {
      LOG(WARNING) << "Failed to attach to module " << module_name << ": "
                   << proxy_or.status();
      continue;
    }

    module_proxies.emplace_back(std::move(proxy_or.value()));
  }

  intrinsic_proto::icon::hal::HardwareModules hardware_modules;
  for (const HardwareModuleProxy& proxy : module_proxies) {
    intrinsic_proto::icon::hal::HardwareModule hardware_module;
    hardware_module.set_name(proxy.Name());

    INTR_ASSIGN_OR_RETURN(std::vector<std::string> required_interfaces,
                          proxy.GetRequiredInterfaceNames());
    absl::flat_hash_set<std::string> required_interfaces_set(
        required_interfaces.begin(), required_interfaces.end());

    for (const std::string& interface : proxy.GetHardwareInterfaceNames()) {
      intrinsic_proto::icon::hal::HardwareInterface hardware_interface;
      hardware_interface.set_name(interface);
      hardware_interface.set_must_be_used(
          required_interfaces_set.contains(interface));

      hardware_module.mutable_interfaces()->Add(std::move(hardware_interface));
    }
    hardware_modules.mutable_modules()->Add(std::move(hardware_module));
  }

  return hardware_modules;
}

absl::Status PrintHardwareModuleInfo() {
  INTR_ASSIGN_OR_RETURN(intrinsic_proto::icon::hal::HardwareModules module_info,
                        ModuleInfo());

  if (module_info.modules_size() == 0) {
    LOG(INFO) << "No hardware modules found.";
  }

  LOG(INFO) << "Found " << module_info.modules_size() << " hardware module(s)";
  for (const intrinsic_proto::icon::hal::HardwareModule& hardware_module :
       module_info.modules()) {
    LOG(INFO) << hardware_module.name();
    for (const intrinsic_proto::icon::hal::HardwareInterface&
             hardware_interface : hardware_module.interfaces()) {
      LOG(INFO) << "\t" << hardware_interface.name() << " "
                << (hardware_interface.must_be_used() ? "[must_be_used]"
                                                      : "[optional]");
    }
  }

  return absl::OkStatus();
}

}  // namespace

class HardwareModuleInfoService
    : public intrinsic_proto::icon::hal::HardwareModuleInfo::Service {
 public:
  grpc::Status List(
      grpc::ServerContext* context,
      const intrinsic_proto::icon::hal::ListHardwareModuleRequest* request,
      intrinsic_proto::icon::hal::HardwareModules* response) override {
    auto module_info = ModuleInfo();
    if (!module_info.ok()) {
      return intrinsic::ToGrpcStatus(module_info.status());
    }
    *response = *module_info;
    return grpc::Status::OK;
  }
};

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);

  if (absl::GetFlag(FLAGS_print_only)) {
    CHECK_OK(PrintHardwareModuleInfo());
    return 0;
  }

  std::string server_address = absl::StrCat("[::]:", absl::GetFlag(FLAGS_port));
  HardwareModuleInfoService info_service;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(
      server_address, grpc::experimental::LocalServerCredentials(LOCAL_TCP));
  builder.RegisterService(&info_service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  LOG(INFO) << "Server listening on " << server_address;

  server->Wait();
  return 0;
}
