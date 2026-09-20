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

// noop is a utility for resources to indicate they provide their real or
// simulation support through other means and do not need code running, but
// should not run the real version by default.  See b/263305147.  If a port is
// allocated to the resource, it will start a server on that port, but no
// provide any services, triggering an Unimplemented error if it is called.
package main

import (
	"context"
	"flag"
	"fmt"
	"net"

	"intrinsic/kubernetes/intrinsic"
	"intrinsic/util/proto/protoio"

	log "github.com/golang/glog"
	"google.golang.org/grpc"

	drgrpcpb "intrinsic/assets/services/proto/v1/dynamic_reconfiguration_go_proto"
	drpb "intrinsic/assets/services/proto/v1/dynamic_reconfiguration_go_proto"
	rcpb "intrinsic/resources/proto/runtime_context_go_proto"
)

var flagContextFile = flag.String("context", "/etc/intrinsic/runtime_config.pb", "The path to the resource instance proto file")

// noopDynamicReconfiguration is a no-op implementation of the DynamicReconfiguration service.
//
// Serving this service enables a Service that supports dynamic reconfiguration to use the noop
// image as its sim image.
type noopDynamicReconfiguration struct{}

func (noopDynamicReconfiguration) ApplyConfiguration(ctx context.Context, req *drpb.ApplyConfigurationRequest) (*drpb.ApplyConfigurationResponse, error) {
	return &drpb.ApplyConfigurationResponse{}, nil
}

// startDummyServer opens up a dummy gRPC server on the specified port.
func startDummyServer(port int32) error {
	address := fmt.Sprintf(":%d", port)
	lis, err := net.Listen("tcp", address)
	if err != nil {
		return fmt.Errorf("server failed to listen at %q: %v", address, err)
	}
	log.Infof("Server is now listening at %q", address)

	s := grpc.NewServer()
	drgrpcpb.RegisterDynamicReconfigurationServer(s, &noopDynamicReconfiguration{})

	if err := s.Serve(lis); err != nil {
		return fmt.Errorf("server failed to serve: %v", err)
	}
	return nil
}

func main() {
	intrinsic.Init()

	rc := new(rcpb.RuntimeContext)
	if p := *flagContextFile; p != "" {
		if err := protoio.ReadBinaryProto(p, rc); err != nil {
			log.Exitf("Failed to read runtime context from %q: %v", p, err)
		}
	}

	log.Infof("Resource Name: %q", rc.GetName())

	if p := rc.GetPort(); p != 0 {
		if err := startDummyServer(p); err != nil {
			log.Exitf("Failed to setup server: %v", err)
		}
	} else {
		// Since we have no port, part the thread until we get an unhandled
		// signal.
		select {}
	}
}
