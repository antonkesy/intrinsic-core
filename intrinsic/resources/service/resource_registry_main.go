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

package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"net"
	"time"

	"intrinsic/assets/internal/assetinfointernal"
	"intrinsic/kubernetes/intrinsic"
	"intrinsic/resources/service/resourcereader"
	"intrinsic/resources/service/resourceregistry"
	"intrinsic/resources/service/resourcetyperuntime"
	"intrinsic/skills/internal/skillcomposer"
	"intrinsic/stats/go/telemetry"
	"intrinsic/util/go/shutdown"

	log "github.com/golang/glog"
	"go.opencensus.io/plugin/ocgrpc"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/reflection"
	"k8s.io/client-go/informers"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"

	assetinfointernalgrpcpb "intrinsic/assets/proto/v1alpha1/asset_info_internal_go_proto"
	resourcespb "intrinsic/resources/proto/resource_registry_go_proto"
	rrintpb "intrinsic/resources/proto/resource_registry_internal_go_proto"
	rdbgrpcpb "intrinsic/resources/proto/runtime_db_go_proto"
	asgrpcpb "intrinsic/storage/hot_shared_state/proto/application_service_go_proto"
	resourcesetservicepb "intrinsic/storage/hot_shared_state/proto/v1/resource_set_service_go_proto"
)

var (
	hssServiceAddress       = flag.String("hss_service_address", "", "The hot shared	state service address")
	runtimeDbServiceAddress = flag.String("runtime_db_service_address", "", "The runtime DB service address")
	port                    = flag.Int("port", 8080, "Port to serve gRPC on.")
	assetInfoInternalPort   = flag.Int("asset_info_internal_port", 0, "Port on which to serve the asset info internal service.")
	prometheusPort          = flag.Int64("opencensus_metrics_port", 9101, "Which port to serve the prometheus scraper on.")
	opencensusTracing       = flag.Bool("opencensus_tracing", false, "Whether to send traces to opencensus.")
)

func clientSet() *kubernetes.Clientset {
	clusterconfig, err := rest.InClusterConfig()
	if err != nil {
		log.Exitf("Could not get kubernetes cluster config: %v", err.Error())
	}
	clientset, err := kubernetes.NewForConfig(clusterconfig)
	if err != nil {
		log.Exitf("Could not create a new kubernetes clientset: %v", err.Error())
	}
	return clientset
}

func main() {
	intrinsic.Init()
	ctx, cancel := shutdown.RegisterContext(context.Background())
	defer cancel()

	telem := telemetry.Initialize(
		telemetry.WithTracing(*opencensusTracing), // on-prem!
		telemetry.EnableMetrics(*prometheusPort),
		telemetry.WithViews(append(ocgrpc.DefaultClientViews, ocgrpc.DefaultServerViews...)),
	)

	hssConn, err := grpc.NewClient(
		*hssServiceAddress,
		grpc.WithTransportCredentials(insecure.NewCredentials()),
		grpc.WithStatsHandler(new(ocgrpc.ClientHandler)),
		grpc.WithDefaultCallOptions(grpc.MaxCallRecvMsgSize(1024*1024*1024)),
	)
	if err != nil {
		log.ExitContextf(ctx, "Failed to establish connection to the hot shared state service %q: %v", *hssServiceAddress, err)
	}
	resourceSetClient := resourcesetservicepb.NewHotSharedStateResourceSetServiceClient(hssConn)
	appClient := asgrpcpb.NewHotSharedStateApplicationServiceClient(hssConn)

	runtimeDBConn, err := grpc.NewClient(
		*runtimeDbServiceAddress,
		grpc.WithTransportCredentials(insecure.NewCredentials()),
		grpc.WithStatsHandler(new(ocgrpc.ClientHandler)),
		grpc.WithDefaultCallOptions(grpc.MaxCallRecvMsgSize(math.MaxInt)),
		grpc.WithDefaultCallOptions(grpc.MaxCallSendMsgSize(math.MaxInt)),
	)
	if err != nil {
		log.ExitContextf(ctx, "Failed to establish connection to the runtime db service %q: %v", *runtimeDbServiceAddress, err)
	}

	runtimeDBClient := rdbgrpcpb.NewRuntimeDbClient(runtimeDBConn)
	resourceTypeRuntimeClient := resourcetyperuntime.CreateClient(runtimeDBClient)

	rr := resourcereader.NewClient(resourcereader.NewClientOpts{
		RSSClient: resourceSetClient,
		RTRClient: resourceTypeRuntimeClient,
	})

	factory := informers.NewSharedInformerFactory(clientSet(), 10*time.Minute)
	composerClient := skillcomposer.NewClient(resourceTypeRuntimeClient, factory)

	factory.Start(ctx.Done())
	factory.WaitForCacheSync(ctx.Done())

	server := resourceregistry.NewServer(rr)
	internalserver := assetinfointernal.NewServer(assetinfointernal.NewServerOpts{
		AppClient:     appClient,
		RR:            rr,
		RTRClient:     resourceTypeRuntimeClient,
		SkillComposer: composerClient,
	})

	s := grpc.NewServer(grpc.StatsHandler(&ocgrpc.ServerHandler{}))
	resourcespb.RegisterResourceRegistryServer(s, server)
	rrintpb.RegisterResourceRegistryInternalServer(s, server)
	reflection.Register(s)

	log.InfoContextf(ctx, "Starting resource registry service at %d", *port)
	if lis, err := net.Listen("tcp", fmt.Sprintf(":%d", *port)); err != nil {
		log.ExitContextf(ctx, "Failed to listen: %v", err)
	} else {
		go s.Serve(lis)
	}

	assetInfoServer := grpc.NewServer(grpc.StatsHandler(&ocgrpc.ServerHandler{}))
	assetinfointernalgrpcpb.RegisterAssetInfoInternalServer(assetInfoServer, internalserver)
	reflection.Register(assetInfoServer)

	log.InfoContextf(ctx, "Starting asset info internal service at %d", *assetInfoInternalPort)
	if lis, err := net.Listen("tcp", fmt.Sprintf(":%d", *assetInfoInternalPort)); err != nil {
		log.ExitContextf(ctx, "Failed to listen: %v", err)
	} else {
		go assetInfoServer.Serve(lis)
	}

	select {
	case <-ctx.Done():
		log.InfoContext(ctx, "Received shutdown signal, stopping")
	}

	log.InfoContext(ctx, "Requesting shutdown of resource registry server")
	s.GracefulStop()
	log.InfoContext(ctx, "Requesting shutdown of asset info internal server")
	assetInfoServer.GracefulStop()
	log.InfoContext(ctx, "Requesting shutdown of telemetry")
	telem.Shutdown(ctx)
	log.InfoContext(ctx, "Shutdown complete")
}
