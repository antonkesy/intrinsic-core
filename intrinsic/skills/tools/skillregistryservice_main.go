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

/*
Creates a skill registry server using data from a SkillRegistryConfig proto file.

Example update of kubernetes ConfigMap using command-line tools:

# TODO(b/451781058) Update the example, since /bluebird is deprecated.

# build a skill registry config
bazel build -c opt intrinsic/apps/bluebird/skills:skill_registry_config

$ make edits in text editor
chmod a+rw bazel-genfiles/intrinsic/apps/bluebird/skills/skill_registry.pbtxt
emacs bazel-genfiles/intrinsic/apps/bluebird/skills/skill_registry.pbtxt

# serialize text proto
gqui textproto:bazel-genfiles/intrinsic/apps/bluebird/skills/skill_registry.pbtxt \
	proto intrinsic_proto.skills.SkillRegistryConfig \
	--outfile=rawproto:/tmp/test_registry.pbtxt

# replace skill-registry
kubectl create configmap skill-registry \
	--from-file=proto=/tmp/test_registry.pbtxt \
	--dry-run -o yaml | kubectl replace -f -
*/

package main

import (
	"context"
	"flag"
	"fmt"
	"math"
	"net"
	"time"

	"intrinsic/kubernetes/intrinsic"
	"intrinsic/resources/service/resourcetyperuntime"
	"intrinsic/skills/internal/skillcomposer"
	"intrinsic/skills/internal/skillregistryservice"
	"intrinsic/skills/internal/skillruntime"
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

	rdbgrpcpb "intrinsic/resources/proto/runtime_db_go_proto"
	btregistryinternalgrpcpb "intrinsic/skills/internal/proto/behavior_tree_registry_internal_go_proto"
	skillregistryinternalgrpcpb "intrinsic/skills/internal/proto/skill_registry_internal_go_proto"
	btregistrygrpcpb "intrinsic/skills/proto/behavior_tree_registry_go_proto"
	skillregistrygrpcpb "intrinsic/skills/proto/skill_registry_go_proto"
)

var (
	port                    = flag.Int("port", 12346, "Port to serve gRPC on.")
	runtimeDbServiceAddress = flag.String("runtime_db_service_address", "", "The runtime DB service address")
	prometheusPort          = flag.Int64("opencensus_metrics_port", 9101, "Which port to serve the prometheus scraper on.")
	opencensusTracing       = flag.Bool("opencensus_tracing", false, "Whether to send traces to opencensus.")
	traceProbability        = flag.Float64("trace_probability", 1.0, "The fraction of requests to upload to Stackdriver Trace.")
)

// GRPC targets will use this replacement string where %s is replaced by --config_add_namespace.
const namespaceAddReplace = "$1.%s:"

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

	// Set up and serve telemetry.
	// The returned object could be used to implement a graceful shutdown.
	telemetry.Initialize(
		telemetry.WithTracing(*opencensusTracing), // on-prem!
		telemetry.WithProbability(*traceProbability),
		telemetry.EnableMetrics(*prometheusPort),
		telemetry.WithViews(ocgrpc.DefaultServerViews),
	)

	lis, err := net.Listen("tcp", fmt.Sprintf("0.0.0.0:%d", *port))
	if err != nil {
		log.ExitContextf(ctx, "Server failed to listen: %v", err)
	}
	log.InfoContextf(ctx, "Server is now listening on port: %d", *port)

	server := grpc.NewServer(grpc.StatsHandler(&ocgrpc.ServerHandler{}))
	if err != nil {
		log.ExitContextf(ctx, "Failed to create new grpcprod server: %v", err)
	}
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
	rtrClient := resourcetyperuntime.CreateClient(runtimeDBClient)
	skillRuntimeClient := skillruntime.CreateClient(runtimeDBClient)

	clientset := clientSet()
	factory := informers.NewSharedInformerFactory(clientset, 10*time.Minute)
	composerClient := skillcomposer.NewClient(rtrClient, factory)

	factory.Start(ctx.Done())
	factory.WaitForCacheSync(ctx.Done())

	// This constructs a new skill registry server.
	registryserver := skillregistryservice.NewCombined(skillRuntimeClient, composerClient)

	skillregistryinternalgrpcpb.RegisterSkillRegistryInternalServer(server, registryserver)
	skillregistrygrpcpb.RegisterSkillRegistryServer(server, registryserver)

	btregistryinternalgrpcpb.RegisterBehaviorTreeRegistryInternalServer(server, registryserver)
	btregistrygrpcpb.RegisterBehaviorTreeRegistryServer(server, registryserver)

	// This sets up reflection for the grpc server.
	reflection.Register(server)

	// This call starts the server.
	server.Serve(lis)
}
