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

#ifndef INTRINSIC_STATS_OPENCENSUS_H_
#define INTRINSIC_STATS_OPENCENSUS_H_

#include <prometheus/exposer.h>

#include <memory>

#include "absl/strings/string_view.h"
#include "grpcpp/support/interceptor.h"
#include "grpcpp/support/server_interceptor.h"
#include "opencensus/exporters/stats/prometheus/prometheus_exporter.h"

namespace intrinsic {

// A class that initializes telemetry instrumentation for stats and gRPC
// tracing. The stats are exposed via at an HTTP endpoint that is scraped
// by Prometheus for monitoring and alerting. This needs to be configured using
// a ServiceMonitor in the deployment yaml. The traces are pushed to
// the OpenTelemetry Collector.
// If your program uses gRPC, a number of metrics are provided
// out-of-the-box (latency, error count, bytes sent etc). See
// third_party/grpc/src/cpp/ext/filters/census/grpc_plugin.h for a complete
// list.
// For documentation on how to define custom metrics see
// https://opencensus.io/quickstart/cpp/.
//
// Sample Usage:
//
//   int main(int argc, char** argv) {
//     InitGoogle(argv[0], &argc, &argv, true);
//     OpenCensusPlugin opencensus;
//
//     ... Define custom metrics and traces ...
//   }
//
// Then run the binary with --opencensus_metrics_port=9090 and
// --opencensus_tracing=true. To get prometheus to start
// scraping the metrics you will also need to add a ServiceMonitor and
// corresponding Service to your deployment's yaml file.
class OpenCensusPlugin {
 public:
  explicit OpenCensusPlugin(absl::string_view service_name = "",
                            bool use_otel_metrics = false,
                            bool initialize_sdk = true);

  // Disable copy and move to prevent multiple destructor calls.
  OpenCensusPlugin(const OpenCensusPlugin&) = delete;
  OpenCensusPlugin& operator=(const OpenCensusPlugin&) = delete;

  // Waits for all traces to be uploaded.
  ~OpenCensusPlugin();

 private:
  // Needs to be initialized before the gRPC plugins are registered.
  std::shared_ptr<opencensus::exporters::stats::PrometheusExporter>
      prometheus_exporter_;
  std::unique_ptr<prometheus::Exposer> prometheus_exposer_;
};

// An interceptor factory that creates a metric that can be used to detect the
// first RPC with a given status, unlike gRPC's built-in metrics. The metric,
// intrinsic/stats/rpc_count, is tagged with the RPC method name and the gRPC
// status.
//
// The factory should be passed to the gRPC ServerBuilder's
// http://cs/symbol:SetInterceptorCreators.
class OpenCensusInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface {
 public:
  explicit OpenCensusInterceptorFactory(absl::string_view service_full_name,
                                        bool use_otel_metrics = false);

  ::grpc::experimental::Interceptor* CreateServerInterceptor(
      ::grpc::experimental::ServerRpcInfo* info) override;

 private:
  bool use_otel_metrics_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_STATS_OPENCENSUS_H_
