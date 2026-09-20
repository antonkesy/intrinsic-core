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
	"net"
	"strings"

	"intrinsic/kubernetes/intrinsic"
	"intrinsic/resources/service/runtimedb"
	"intrinsic/stats/go/telemetry"
	"intrinsic/util/go/shutdown"

	log "github.com/golang/glog"
	"go.opencensus.io/plugin/ocgrpc"
	"google.golang.org/grpc"
	"google.golang.org/grpc/reflection"

	rdgrpcpb "intrinsic/resources/proto/runtime_db_go_proto"
)

const (
	numBytesInGiB = 1024 * 1024 * 1024
)

var (
	buckets        = flag.String("buckets", "", "Comma-separated list of DB buckets to use.")
	path           = flag.String("path", "", "Path where the bolt DB file will be stored")
	port           = flag.Int("port", 0, "Port to serve gRPC on.")
	prometheusPort = flag.Int64("opencensus_metrics_port", 0, "Which port to serve the prometheus scraper on.")
)

func main() {
	intrinsic.Init()
	ctx, cancel := shutdown.RegisterContext(context.Background())
	defer cancel()

	if *buckets == "" {
		log.ExitContextf(ctx, "--buckets is required")
	}
	if *path == "" {
		log.ExitContextf(ctx, "--path is required")
	}
	if *port == 0 {
		log.ExitContextf(ctx, "--port is required")
	}
	if *prometheusPort == 0 {
		log.ExitContextf(ctx, "--opencensus_metrics_port is required")
	}
	log.InfoContextf(ctx, "Starting RuntimeDB server on port %d, with path %q, and buckets %q", *port, *path, *buckets)

	lis, err := net.Listen("tcp", fmt.Sprintf("0.0.0.0:%d", *port))
	if err != nil {
		log.ExitContextf(ctx, "Server failed to listen: %v", err)
	}
	log.InfoContextf(ctx, "Server is now listening on port: %d", *port)

	telem := telemetry.Initialize(
		telemetry.EnableMetrics(*prometheusPort),
		telemetry.WithViews(ocgrpc.DefaultServerViews),
	)

	grpcServer := grpc.NewServer(
		grpc.StatsHandler(&ocgrpc.ServerHandler{}),
		grpc.MaxRecvMsgSize(numBytesInGiB),
	)

	server, runtimeDBCleanup, err := runtimedb.NewServer(*path, strings.Split(*buckets, ","))
	if err != nil {
		log.ExitContextf(ctx, "Failed to create new runtime db server: %v", err)
	}

	rdgrpcpb.RegisterRuntimeDbServer(grpcServer, server)
	reflection.Register(grpcServer)
	go func() {
		if err := grpcServer.Serve(lis); err != nil && err != grpc.ErrServerStopped {
			log.Errorf("gRPC server failed: %v", err)
			cancel() // Unblock the main thread
		}
	}()

	<-ctx.Done()
	log.InfoContext(ctx, "Requesting shutdown of runtime DB server")
	grpcServer.GracefulStop()
	log.InfoContext(ctx, "Closing runtime DB database files")
	runtimeDBCleanup()
	log.InfoContext(ctx, "Requesting shutdown of telemetry")
	telem.Shutdown(ctx)
}
