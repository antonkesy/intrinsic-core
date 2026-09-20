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

// Package hotsharedstateservice (HSS service) implements an in-memory store for
// the current cluster document.
package hotsharedstateservice

import (
	"context"
	"fmt"
	"sync"
	"time"

	"intrinsic/config/applicationview"
	"intrinsic/config/convertrunnables"
	"intrinsic/config/util"
	"intrinsic/kubernetes/data_store/revision"

	log "github.com/golang/glog"
	"github.com/pborman/uuid"
	"go.opencensus.io/trace"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	applicationpb "intrinsic/config/proto/application_go_proto"
	clusterpb "intrinsic/config/proto/cluster_go_proto"
	commonpb "intrinsic/config/proto/common_go_proto"
	resourcesetpb "intrinsic/config/proto/resource_set_go_proto"
	idbcsgrpcpb "intrinsic/kubernetes/data_store/proto/cluster_service_go_proto"
	idbcspb "intrinsic/kubernetes/data_store/proto/cluster_service_go_proto"
	casgrpcpb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
	applicationsvcpb "intrinsic/storage/hot_shared_state/proto/application_service_go_proto"
	clustersvcpb "intrinsic/storage/hot_shared_state/proto/v1/cluster_service_go_proto"
	resourcesetsvcpb "intrinsic/storage/hot_shared_state/proto/v1/resource_set_service_go_proto"

	timestamppb "google.golang.org/protobuf/types/known/timestamppb"
)

var timeNow = time.Now // Stubbed out for testing.

// Options configure the [HSS] service.
type Options struct {
	ClusterName string
	// Maximum size of the current cluster document. If a request causes the cluster size to exceed
	// this limit, a [codes.ResourceExhausted] error code is returned.
	MaxClusterSize int
	// Disable the cluster doc backup (b/330691458).
	DisableClusterDocBackup bool
}

// DefaultOptions returns [Options] with reasonable default values set. Notice that you still need
// to set some fields manually, e.g. "ClusterName".
func DefaultOptions() Options {
	return Options{
		// LINT.IfChange
		MaxClusterSize: 50000000, // 50 MB
		// LINT.ThenChange(//intrinsic/storage/hot_shared_state/README.md)
	}
}

// HSS implements the hot shared state services for cluster, application,
// and resource set protos.
type HSS struct {
	mu             sync.RWMutex
	cluster        *clusterpb.Cluster
	revision       revision.Revision
	tq             chan func()
	onpremClusterC idbcsgrpcpb.ClusterServiceClient
	cloudClusterC  idbcsgrpcpb.ClusterServiceClient
	casC           casgrpcpb.ContentAddressableStorageServiceClient
	opts           Options
}

// New returns a [HSS]. It is initialized with an empty cluster document.
func New(opts Options, onpremClusterC, cloudClusterC idbcsgrpcpb.ClusterServiceClient, casC casgrpcpb.ContentAddressableStorageServiceClient) *HSS {
	tq := make(chan func(), 10)
	go func() {
		for f := range tq {
			f()
		}
	}()

	return &HSS{
		revision:       revision.FromString(uuid.New()),
		tq:             tq,
		onpremClusterC: onpremClusterC,
		cloudClusterC:  cloudClusterC,
		casC:           casC,
		opts:           opts,
	}
}

// backupCurrentCluster reads the current cluster from the onprem volume and
// uploads it to the BoltDB and to Firestore under the corresponding name. If
// the providers ([boltadapter.BoltAdapter] or [firestoreshim.FirestoreShim]
// respectively) are nil, skip.
func (s *HSS) backupCurrentCluster() {
	// TODO: b/330691458 - Figure out how to do authentication here.

	ctx := context.Background()
	ctx, cancel := context.WithTimeout(ctx, 30*time.Second)
	defer cancel()

	res, err := s.GetCurrentCluster(ctx, &clustersvcpb.GetCurrentClusterRequest{})
	if err != nil {
		log.WarningContextf(ctx, "Backup failed: could not get current cluster: %v", err)
		return
	}

	// We need to clone here because we will mutate the proto before uploading it to Firestore,
	// see b/270131227.
	cluster := proto.Clone(res.GetCluster()).(*clusterpb.Cluster)

	if s.onpremClusterC == nil {
		log.WarningContextf(ctx, "Backup skipped: no onprem cluster service available")
		return
	}
	if _, err := s.onpremClusterC.SetClusterOnprem(ctx, &idbcspb.SetClusterRequest{
		Cluster:   cluster,
		Overwrite: true,
	}); err != nil {
		log.WarningContextf(ctx, "Backup to BoltDB failed: %v", err)
	}

	// Strip application since it can be too large for a backup.
	app := cluster.GetApplication()
	cluster.Application = applicationview.MetadataOnly(app)
	if err := s.updateAppInFirestore(ctx, cluster); err != nil {
		log.WarningContextf(ctx, "Backup to Firestore failed: %v", err)
	}
}

func (s *HSS) updateAppInFirestore(ctx context.Context, clusterForBackup *clusterpb.Cluster) error {
	if s.opts.DisableClusterDocBackup {
		log.InfoContextf(ctx, "Skipping backup to firestore.")
		return nil
	}

	if s.cloudClusterC == nil {
		log.WarningContextf(ctx, "Backup to Firestore skipped: no cloud cluster service available")
		return nil
	}

	res, err := s.cloudClusterC.GetCluster(ctx, &idbcspb.GetClusterRequest{Name: s.opts.ClusterName})
	if c := status.Code(err); c == codes.NotFound {
		// Very improbable case that the cluster is not available in Firestore. Then use the cluster
		// proto from HSS "as is".
		if _, err := s.cloudClusterC.SetCluster(ctx, &idbcspb.SetClusterRequest{
			Cluster:   clusterForBackup,
			Overwrite: true,
		}); err != nil {
			return fmt.Errorf("cannot overwrite the cluster document in Firestore: %w", err)
		}
		return nil
	} else if c != codes.OK {
		return fmt.Errorf("cannot obtain the cluster document from Firestore: %w", err)
	}

	// If there already is a cluster proto in Firestore, don't overwrite it completely, update only
	// the application within it.
	cluster := res.GetCluster()
	cluster.Application = clusterForBackup.GetApplication()
	if _, err := s.cloudClusterC.SetCluster(ctx, &idbcspb.SetClusterRequest{
		Cluster:       cluster,
		RevisionToken: res.GetRevisionToken(),
	}); err != nil {
		return fmt.Errorf("cannot update the cluster document in Firestore: %w", err)
	}
	return nil
}

func (s *HSS) unsafeMutateCluster(mut func(*clusterpb.Cluster) *clusterpb.Cluster) error {
	s.cluster = mut(s.cluster)

	if s.cluster.GetMetadata() == nil {
		s.cluster.Metadata = &commonpb.Metadata{}
	}
	s.cluster.GetMetadata().UpdateTime = timestamppb.New(timeNow().UTC())
	s.cluster.GetMetadata().Name = s.opts.ClusterName
	s.cluster.ClusterName = s.opts.ClusterName

	s.revision = revision.FromString(uuid.New())

	bytes, err := proto.Marshal(s.cluster)
	if err != nil {
		return status.Errorf(codes.Internal, "could not serialize cluster document: %v", err)
	}
	if sz := len(bytes); sz > s.opts.MaxClusterSize {
		return status.Errorf(codes.ResourceExhausted, "cluster document too large: size %d > max %d", sz, s.opts.MaxClusterSize)
	}

	select {
	case s.tq <- s.backupCurrentCluster:
		log.Info("Queued a current cluster backup task")
	default:
		log.Warning("Task queue is full, no additional backup queued.")
	}

	return nil
}

func (s *HSS) unsafeSetClusterAndReturn(cluster *clusterpb.Cluster) (*clustersvcpb.SetCurrentClusterResponse, error) {
	if err := s.unsafeMutateCluster(func(*clusterpb.Cluster) *clusterpb.Cluster {
		return cluster
	}); err != nil {
		return nil, err // Don't wrap error because [HSS.unsafeMutateCluster] sets the canonical code.
	}
	return &clustersvcpb.SetCurrentClusterResponse{
		Cluster:       s.cluster,
		RevisionToken: s.revision.String(),
	}, nil
}

func (s *HSS) unsafeSetApplicationAndReturn(application *applicationpb.Application) (*applicationsvcpb.SetCurrentApplicationResponse, error) {
	if err := s.unsafeMutateCluster(func(c *clusterpb.Cluster) *clusterpb.Cluster {
		c.Application = application
		return c
	}); err != nil {
		return nil, err // Don't wrap error because [HSS.unsafeMutateCluster] sets the canonical code.
	}
	return &applicationsvcpb.SetCurrentApplicationResponse{
		Application:   s.cluster.GetApplication(),
		RevisionToken: s.revision.String(),
	}, nil
}

func (s *HSS) unsafeSetResourceSetAndReturn(resourceSet *resourcesetpb.ResourceSet) (*resourcesetsvcpb.SetCurrentResourceSetResponse, error) {
	if err := s.unsafeMutateCluster(func(c *clusterpb.Cluster) *clusterpb.Cluster {
		c.Application.Resources = resourceSet
		return c
	}); err != nil {
		return nil, err // Don't wrap error because [HSS.unsafeMutateCluster] sets the canonical code.
	}
	return &resourcesetsvcpb.SetCurrentResourceSetResponse{
		ResourceSet:   s.cluster.GetApplication().GetResources(),
		RevisionToken: s.revision.String(),
	}, nil
}

func (s *HSS) GetCurrentCluster(ctx context.Context, req *clustersvcpb.GetCurrentClusterRequest) (*clustersvcpb.GetCurrentClusterResponse, error) {
	_, span := trace.StartSpan(ctx, "HotSharedStateService.GetCurrentCluster")
	defer span.End()

	s.mu.RLock()
	defer s.mu.RUnlock()

	if s.cluster == nil {
		return nil, status.Errorf(codes.FailedPrecondition, "no current cluster")
	}

	cluster := proto.Clone(s.cluster).(*clusterpb.Cluster)
	// Omit the application ONLY if the view is `CLUSTER_VIEW_NO_APPLICATION`. Otherwise, return the
	// full cluster proto since the default view is `CLUSTER_VIEW_FULL`.
	if req.GetView() == clustersvcpb.ClusterView_CLUSTER_VIEW_NO_APPLICATION {
		cluster.Application = nil
		if cluster.GetMetadata() != nil {
			cluster.Metadata.IsIncomplete = true
		} else {
			cluster.Metadata = &commonpb.Metadata{
				IsIncomplete: true,
			}
		}
	}

	return &clustersvcpb.GetCurrentClusterResponse{
		Cluster:       cluster,
		RevisionToken: s.revision.String(),
	}, nil
}

func (s *HSS) SetCurrentCluster(ctx context.Context, req *clustersvcpb.SetCurrentClusterRequest) (*clustersvcpb.SetCurrentClusterResponse, error) {
	_, span := trace.StartSpan(ctx, "HotSharedStateService.SetCurrentCluster")
	defer span.End()

	// Perform operations only on the request proto, do not touch the internal
	// state before the mutex is locked.
	if req.GetCluster() == nil {
		return nil, status.Errorf(codes.InvalidArgument, "no cluster given")
	}

	if reqName := req.GetCluster().GetMetadata().GetName(); reqName != s.opts.ClusterName {
		return nil, status.Errorf(codes.InvalidArgument, "cluster name mismatch: %q given, want %q", reqName, s.opts.ClusterName)
	}

	if err := denySavingIncomplete(req.GetCluster().GetMetadata()); err != nil {
		return nil, err // Status is set by the helper function, do not wrap!
	}

	revisionReq := revision.FromString(req.GetRevisionToken())
	overwrite := req.GetOverwrite()
	clusterReq := req.GetCluster()
	if err := util.UnwrapAnyProtos(clusterReq); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "unwrapping protobuf.Any protos: %v", err)
	}
	if proc := req.GetCluster().GetApplication().GetProcess(); proc != nil {
		if err := convertrunnables.MoveBytesToRunnables(proc); err != nil {
			return nil, status.Errorf(codes.InvalidArgument, "moving bytes to runnables: %v", err)
		}
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	if s.cluster == nil {
		if revisionReq.IsEmpty() && overwrite {
			return s.unsafeSetClusterAndReturn(clusterReq)
		}
		return nil, status.Errorf(codes.InvalidArgument, "cluster document can be initialized only with an empty revision token and overwrite=true")
	}
	if overwrite {
		return s.unsafeSetClusterAndReturn(clusterReq)
	}
	if revisionReq.IsEmpty() {
		return nil, status.Errorf(codes.AlreadyExists, "%v", revision.ErrDocumentExistsNoTokenSpecified)
	}
	if s.revision != revisionReq {
		return nil, status.Errorf(codes.Aborted, "%v", revision.ErrTokenIsObsolete)
	}
	return s.unsafeSetClusterAndReturn(clusterReq)
}

func (s *HSS) GetCurrentApplication(ctx context.Context, req *applicationsvcpb.GetCurrentApplicationRequest) (*applicationsvcpb.GetCurrentApplicationResponse, error) {
	_, span := trace.StartSpan(ctx, "HotSharedStateService.GetCurrentApplication")
	defer span.End()

	s.mu.RLock()
	defer s.mu.RUnlock()

	if s.cluster == nil {
		return nil, status.Errorf(codes.FailedPrecondition, "no current cluster")
	}
	if s.cluster.GetApplication() == nil {
		return nil, status.Errorf(codes.NotFound, "no current application running on cluster")
	}

	return &applicationsvcpb.GetCurrentApplicationResponse{
		Application:   s.cluster.GetApplication(),
		RevisionToken: s.revision.String(),
	}, nil
}

func (s *HSS) SetCurrentApplication(ctx context.Context, req *applicationsvcpb.SetCurrentApplicationRequest) (*applicationsvcpb.SetCurrentApplicationResponse, error) {
	_, span := trace.StartSpan(ctx, "HotSharedStateService.SetCurrentApplication")
	defer span.End()

	// Perform operations only on the request proto, do not touch the internal
	// state before the mutex is locked.
	if req.GetApplication() == nil {
		return nil, status.Errorf(codes.InvalidArgument, "no application given")
	}

	if err := denySavingIncomplete(req.GetApplication().GetMetadata()); err != nil {
		return nil, err // Status is set by the helper function, do not wrap!
	}

	revisionReq := revision.FromString(req.GetRevisionToken())
	overwrite := req.GetOverwrite()
	applicationReq := req.GetApplication()
	if err := util.UnwrapAnyProtos(applicationReq); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "unwrapping protobuf.Any protos: %v", err)
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	if s.cluster == nil {
		return nil, status.Errorf(codes.FailedPrecondition, "the cluster document is not initialized")
	}
	if s.cluster.GetApplication() == nil {
		if revisionReq.IsEmpty() && overwrite {
			return s.unsafeSetApplicationAndReturn(applicationReq)
		}
		return nil, status.Errorf(codes.InvalidArgument, "application document can be initialized only with an empty revision token and overwrite=true")
	}
	if overwrite {
		return s.unsafeSetApplicationAndReturn(applicationReq)
	}
	if revisionReq.IsEmpty() {
		return nil, status.Errorf(codes.AlreadyExists, "%v", revision.ErrDocumentExistsNoTokenSpecified)
	}
	if s.revision != revisionReq {
		return nil, status.Errorf(codes.Aborted, "%v", revision.ErrTokenIsObsolete)
	}
	return s.unsafeSetApplicationAndReturn(applicationReq)
}

func (s *HSS) GetCurrentResourceSet(ctx context.Context, req *resourcesetsvcpb.GetCurrentResourceSetRequest) (*resourcesetsvcpb.GetCurrentResourceSetResponse, error) {
	_, span := trace.StartSpan(ctx, "HotSharedStateService.GetCurrentResourceSet")
	defer span.End()

	s.mu.RLock()
	defer s.mu.RUnlock()

	if s.cluster == nil {
		return nil, status.Errorf(codes.FailedPrecondition, "no current cluster")
	}
	if s.cluster.GetApplication() == nil {
		return nil, status.Errorf(codes.NotFound, "no current application running on cluster")
	}
	if s.cluster.GetApplication().GetResources() == nil {
		return nil, status.Errorf(codes.NotFound, "no resources in the current application running on cluster")
	}

	return &resourcesetsvcpb.GetCurrentResourceSetResponse{
		ResourceSet:   s.cluster.GetApplication().GetResources(),
		RevisionToken: s.revision.String(),
	}, nil
}

func (s *HSS) SetCurrentResourceSet(ctx context.Context, req *resourcesetsvcpb.SetCurrentResourceSetRequest) (*resourcesetsvcpb.SetCurrentResourceSetResponse, error) {
	_, span := trace.StartSpan(ctx, "HotSharedStateService.SetCurrentResourceSet")
	defer span.End()

	// Perform operations only on the request proto, do not touch the internal
	// state before the mutex is locked.
	resourceSetReq := req.GetResourceSet()
	if resourceSetReq == nil {
		return nil, status.Errorf(codes.InvalidArgument, "no resource set given")
	}

	revisionReq := revision.FromString(req.GetRevisionToken())
	overwrite := req.GetOverwrite()
	if err := util.UnwrapAnyProtos(resourceSetReq); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "unwrapping protobuf.Any protos: %v", err)
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	if s.cluster == nil {
		return nil, status.Errorf(codes.FailedPrecondition, "the cluster document is not initialized")
	}
	if s.cluster.GetApplication() == nil {
		return nil, status.Errorf(codes.FailedPrecondition, "the application is not initialized")
	}
	if s.cluster.GetApplication().GetResources() == nil {
		if revisionReq.IsEmpty() && overwrite {
			return s.unsafeSetResourceSetAndReturn(resourceSetReq)
		}
		return nil, status.Errorf(codes.InvalidArgument, "resource set document can be initialized only with an empty revision token and overwrite=true")
	}
	if overwrite {
		return s.unsafeSetResourceSetAndReturn(resourceSetReq)
	}
	if revisionReq.IsEmpty() {
		return nil, status.Errorf(codes.AlreadyExists, "%v", revision.ErrDocumentExistsNoTokenSpecified)
	}
	if s.revision != revisionReq {
		return nil, status.Errorf(codes.Aborted, "%v", revision.ErrTokenIsObsolete)
	}
	return s.unsafeSetResourceSetAndReturn(resourceSetReq)
}

func denySavingIncomplete(m *commonpb.Metadata) error {
	if m.GetIsIncomplete() {
		return status.Error(codes.InvalidArgument, "invalid request: cannot save an incomplete document")
	}
	return nil
}
