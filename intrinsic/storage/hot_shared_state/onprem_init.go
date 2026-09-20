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

// Package onpreminit helps with initializing the initial state of the
// IntrinsicDB's  onprem volume.
package onpreminit

import (
	"context"
	"fmt"

	"intrinsic/storage/hot_shared_state/hotsharedstateservice"

	log "github.com/golang/glog"
	"google.golang.org/protobuf/proto"

	clusterpb "intrinsic/config/proto/cluster_go_proto"
	commonpb "intrinsic/config/proto/common_go_proto"
	grpcpb "intrinsic/kubernetes/data_store/proto/cluster_service_go_proto"
	pb "intrinsic/kubernetes/data_store/proto/cluster_service_go_proto"
	hsspb "intrinsic/storage/hot_shared_state/proto/v1/cluster_service_go_proto"
)

// InitCurrentCluster initializes the current cluster. This function must be
// used directly after starting IntrinsicDB, but before the server opens the
// port (lest we serve uninitialized data). It first tries to obtain a cluster
// document from the onprem volume (BoltDB); if it fails, from the cloud volume
// (Firestore); and generates an empty cluster proto as the last resort.
func InitCurrentCluster(ctx context.Context, onpremClusterC, cloudClusterC grpcpb.ClusterServiceClient, hss *hotsharedstateservice.HSS, cluster string) error {
	var cOnprem *clusterpb.Cluster
	if c, err := initFromOnpremVolume(ctx, onpremClusterC, cluster); err == nil { // If NO error.
		cOnprem = c
		log.InfoContextf(ctx, "Retrieved cluster from the onprem volume.")
	}

	var cCloud *clusterpb.Cluster
	if c, err := initFromCloudVolume(ctx, cloudClusterC, cluster); err == nil { // If NO error.
		cCloud = c
		log.InfoContextf(ctx, "Retrieved cluster from the cloud volume.")
	}

	if _, err := hss.SetCurrentCluster(ctx, &hsspb.SetCurrentClusterRequest{
		Cluster:   combineDocuments(cOnprem, cCloud, cluster),
		Overwrite: true,
	}); err != nil {
		return fmt.Errorf("could not overwrite current cluster: %w", err)
	}
	return nil
}

func combineDocuments(cOnprem, cCloud *clusterpb.Cluster, name string) *clusterpb.Cluster {
	if cOnprem == nil && cCloud == nil {
		log.Warningf("No cluster in the onprem volume and no cluster in the cloud volume, using an empty cluster document.")
		return mustInitEmpty(name)
	}
	if cOnprem != nil && cCloud == nil {
		log.Infof("No cluster available in the cloud volume, using the cluster document from the onprem volume.")
		cOnprem.CanDoReal = true
		return cOnprem
	}
	if cOnprem == nil && cCloud != nil {
		log.Infof("No cluster available in the onprem volume, using the cluster document from the cloud volume but without the application.")
		c := proto.Clone(cCloud).(*clusterpb.Cluster)
		c.Application = nil
		return c
	}
	log.Infof("Cluster document available in cloud and onprem, combining cloud info with the application from the onprem volume.")
	c := proto.Clone(cCloud).(*clusterpb.Cluster)
	c.Application = cOnprem.GetApplication()
	return c
}

func initFromOnpremVolume(ctx context.Context, cs grpcpb.ClusterServiceClient, cluster string) (*clusterpb.Cluster, error) {
	res, err := cs.GetClusterOnprem(ctx, &pb.GetClusterRequest{Name: cluster})
	if err != nil {
		return nil, err
	}
	return res.GetCluster(), nil
}

func initFromCloudVolume(ctx context.Context, cs grpcpb.ClusterServiceClient, cluster string) (*clusterpb.Cluster, error) {
	if cs == nil {
		return nil, fmt.Errorf("cloud cluster service client is nil")
	}

	res, err := cs.GetCluster(ctx, &pb.GetClusterRequest{Name: cluster})
	if err != nil {
		return nil, err
	}
	return res.GetCluster(), nil
}

func mustInitEmpty(cluster string) *clusterpb.Cluster {
	return &clusterpb.Cluster{
		Metadata: &commonpb.Metadata{
			DisplayName: cluster,
			Name:        cluster,
		},
		CanDoSim:    true,
		CanDoReal:   true,
		ClusterName: cluster,
	}
}
