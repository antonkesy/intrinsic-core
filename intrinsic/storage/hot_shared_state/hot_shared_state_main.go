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

// The hot_shared_state_main command launches a standalone process for the hot shared state (HSS)
// service.
package main

import (
	"context"
	"flag"
	"fmt"
	"net"
	"os"
	"os/signal"

	"intrinsic/kubernetes/data_store/datastoreserver"
	"intrinsic/kubernetes/intrinsic"
	"intrinsic/stats/go/telemetry"
	"intrinsic/storage/hot_shared_state/hotsharedstateservice"
	"intrinsic/storage/hot_shared_state/onpreminit"

	log "github.com/golang/glog"
	"go.opencensus.io/plugin/ocgrpc"
	"golang.org/x/sys/unix"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/reflection"

	idbcsgrpcpb "intrinsic/kubernetes/data_store/proto/cluster_service_go_proto"
	casgrpcpb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
	applicationgrpcpb "intrinsic/storage/hot_shared_state/proto/application_service_go_proto"
	clustergrpcpb "intrinsic/storage/hot_shared_state/proto/v1/cluster_service_go_proto"
	resourcesetgrpcpb "intrinsic/storage/hot_shared_state/proto/v1/resource_set_service_go_proto"
)

const maxRecvMsgBytes = 1024 * 1024 * 1024

var (
	clusterName             = flag.String("cluster_name", "", "Cluster name for GetCurrent*. Leave empty if there is no current cluster.")
	boltPath                = flag.String("bolt_path", "", "Path to the BoltDB file.")
	cloudIntrinsicDBAddress = flag.String("cloud_intrinsic_db_address", "", "Address of the cloud IntrinsicDB deployment.")
	pdfAddress              = flag.String("polymorphic_data_frontend_address", "", "Address of the onprem polymorphic data frontend.")
	portHSS                 = flag.Int("port_hss", 0, "The hot shared state server port")
	portIntrinsicDB         = flag.Int("port_intrinsic_db", 0, "The IntrinsicDB server port")
	// Temp flag for disabling the cluster doc backup
	disableClusterDocBackup = flag.Bool("disable_cluster_doc_backup", false, "Whether to disable the cluster doc backup (b/330691458).")

	// Tracing flags.
	traceProbability  = flag.Float64("trace_probability", 0.0, "The fraction of requests to upload to Stackdriver Trace.")
	prometheusPort    = flag.Int64("opencensus_metrics_port", 9101, "Which port to serve the prometheus endpoint on.")
	opencensusTracing = flag.Bool("opencensus_tracing", false, "Whether to send traces to opencensus.")
)

func startHSS(ctx context.Context, hssPort, intrinsicDBPort int, cloudAddress, casAddress, cluster string, disableClusterDocBackup bool) {
	log.InfoContext(ctx, "Using hot shared state service.")

	onpremIntrinsicDBAddress := fmt.Sprintf("0.0.0.0:%d", intrinsicDBPort)
	onpremIntrinsicDBConn, err := grpc.NewClient(
		onpremIntrinsicDBAddress,
		grpc.WithTransportCredentials(insecure.NewCredentials()),
		grpc.WithStatsHandler(new(ocgrpc.ClientHandler)),
		grpc.WithDefaultCallOptions(grpc.MaxCallRecvMsgSize(maxRecvMsgBytes)),
	)
	if err != nil {
		log.ExitContextf(ctx, "Failed to establish connection to the onprem IntrinsicDB service at %q: %v", onpremIntrinsicDBAddress, err)
	}
	onpremClusterC := idbcsgrpcpb.NewClusterServiceClient(onpremIntrinsicDBConn)


	var casC casgrpcpb.ContentAddressableStorageServiceClient
	if casAddress != "" {
		log.InfoContextf(ctx, "Connecting to CAS service %q", casAddress)
		casConn, err := grpc.NewClient(
			casAddress,
			grpc.WithTransportCredentials(insecure.NewCredentials()),
			grpc.WithStatsHandler(new(ocgrpc.ClientHandler)),
		)
		if err != nil {
			log.ExitContextf(ctx, "Failed to establish connection to the CAS service %q: %v", casAddress, err)
		}
		casC = casgrpcpb.NewContentAddressableStorageServiceClient(casConn)
	} else {
		log.InfoContextf(ctx, "CAS client not initialized, diagnostics upload will be unavailable.")
	}

	lis, err := net.Listen("tcp", fmt.Sprintf("0.0.0.0:%d", hssPort))
	if err != nil {
		log.ErrorContextf(ctx, "Failed to listen: %v", err)
	}
	grpcServer := grpc.NewServer(
		grpc.StatsHandler(&ocgrpc.ServerHandler{}),
		grpc.MaxRecvMsgSize(maxRecvMsgBytes),
	)
	opts := hotsharedstateservice.DefaultOptions()
	opts.ClusterName = cluster
	opts.DisableClusterDocBackup = disableClusterDocBackup

	hssSvc := hotsharedstateservice.New(opts, onpremClusterC, nil, casC)
	applicationgrpcpb.RegisterHotSharedStateApplicationServiceServer(grpcServer, hssSvc)
	clustergrpcpb.RegisterHotSharedStateClusterServiceServer(grpcServer, hssSvc)
	resourcesetgrpcpb.RegisterHotSharedStateResourceSetServiceServer(grpcServer, hssSvc)
	if err := onpreminit.InitCurrentCluster(ctx, onpremClusterC, nil, hssSvc, cluster); err != nil {
		log.ErrorContextf(ctx, "Could not initialize the current cluster: %v", err)
	}
	reflection.Register(grpcServer)
	log.InfoContextf(ctx, "Starting HSS service at %q", lis.Addr())
	go grpcServer.Serve(lis)
}

func main() {
	intrinsic.Init()
	if *boltPath == "" {
		log.Exitf("--bolt_path must be specified.")
	}

	errors := make(chan error)
	sigs := make(chan os.Signal, 1)
	signal.Notify(sigs, unix.SIGTERM)

	// Set up and serve telemetry.
	// The returned object could be used to implement a graceful shutdown.
	telemetry.Initialize(
		telemetry.WithTracing(*opencensusTracing), // on-prem!
		telemetry.WithProbability(*traceProbability),
		telemetry.EnableMetrics(*prometheusPort),
		telemetry.WithViews(append(ocgrpc.DefaultClientViews, ocgrpc.DefaultServerViews...)),
	)

	ctx := context.Background()

	dss, err := datastoreserver.NewClusterServiceBoltDB(ctx, *boltPath)
	if err != nil {
		log.ExitContext(ctx, err)
	}
	dss.Start(*portIntrinsicDB, errors)

	startHSS(ctx, *portHSS, *portIntrinsicDB, *cloudIntrinsicDBAddress, *pdfAddress, *clusterName, *disableClusterDocBackup)

	for {
		select {
		case <-sigs:
			dss.Stop()
		case err := <-errors:
			if err == nil {
				log.InfoContext(ctx, "Shutting down cleanly.")
				os.Exit(0)
			} else {
				log.FatalContextf(ctx, "Shutting down due to an error: %v", err)
			}
		}
	}
}
