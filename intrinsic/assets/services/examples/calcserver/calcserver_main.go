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

// calcserver_main_go is a reference example for how a service should be
// implemented in golang.
package main

import (
	"flag"
	"fmt"
	"net"

	deputils "intrinsic/assets/dependencies/utils"
	"intrinsic/assets/services/examples/calcserver/calcserver"
	"intrinsic/kubernetes/intrinsic"
	"intrinsic/util/proto/protoio"

	log "github.com/golang/glog"
	"google.golang.org/grpc"
	"google.golang.org/grpc/reflection"

	calcgrpcpb "intrinsic/assets/services/examples/calcserver/calc_server_go_proto"
	calcpb "intrinsic/assets/services/examples/calcserver/calc_server_go_proto"
	drgrpcpb "intrinsic/assets/services/proto/v1/dynamic_reconfiguration_go_proto"
	rcpb "intrinsic/resources/proto/runtime_context_go_proto"
)

var flagContextFile = flag.String("context", "/etc/intrinsic/runtime_config.pb", "The path to the service instance proto file")

func main() {
	intrinsic.Init()

	rc := &rcpb.RuntimeContext{}
	if err := protoio.ReadBinaryProto(*flagContextFile, rc); err != nil {
		log.Exitf("Failed to read runtime context from %q: %v", *flagContextFile, err)
	}

	config := &calcpb.CalculatorConfig{}

	if err := rc.GetConfig().UnmarshalTo(config); err != nil {
		log.Exitf("Failed to unpack config: %v", err)
	}

	log.Infof("Service Name: %q", rc.GetName())

	address := fmt.Sprintf(":%d", rc.GetPort())
	lis, err := net.Listen("tcp", address)
	if err != nil {
		log.Exitf("Server failed to listen at %q: %v", address, err)
	}
	log.Infof("Server is now listening at %q", address)

	opts := calcserver.NewCalcServerOptions{
		Config:         config,
		DatasetFetcher: deputils.GetDataPayload,
	}
	calcServer, err := calcserver.NewCalcServer(&opts)
	if err != nil {
		log.Exitf("Failed to create calc server: %v", err)
	}

	s := grpc.NewServer()
	calcgrpcpb.RegisterCalculatorServer(s, calcServer)
	drgrpcpb.RegisterDynamicReconfigurationServer(s, calcServer)
	reflection.Register(s)
	s.Serve(lis)
}
