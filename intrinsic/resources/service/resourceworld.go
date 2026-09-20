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

// Package resourceworld contains functions that help synchronize the state
// between resources and the world.
package resourceworld

import (
	"context"

	log "github.com/golang/glog"
	"github.com/pkg/errors"
	"google.golang.org/grpc/status"

	rspb "intrinsic/config/proto/resource_set_go_proto"
	gridpb "intrinsic/resources/proto/geometric_resource_data_go_proto"
	grpcrsspb "intrinsic/storage/hot_shared_state/proto/v1/resource_set_service_go_proto"
	rsspb "intrinsic/storage/hot_shared_state/proto/v1/resource_set_service_go_proto"
	grpcowspb "intrinsic/world/public/proto/object_world_service_go_proto"
	owspb "intrinsic/world/public/proto/object_world_service_go_proto"
	owupb "intrinsic/world/public/proto/object_world_updates_go_proto"
)

// ResourceReader holds method to retrieve resource instance data.
type ResourceReader interface {
	GeometricResourceInstanceData(ctx context.Context, rs *rspb.ResourceSet) ([]*gridpb.GeometricResourceInstanceData, error)
}

// Client has methods that help keep resources and the world in sync.
type Client struct {
	rssClient grpcrsspb.HotSharedStateResourceSetServiceClient
	owsClient grpcowspb.ObjectWorldServiceClient
	rr        ResourceReader
}

// UpdateResourceSetWorldRelations retrieves the current set of
// ObjectWorldUpdates protos from the world and writes them to the resource set.
func (c *Client) UpdateResourceSetWorldRelations(ctx context.Context, localWorldID string) error {
	response, err := c.rssClient.GetCurrentResourceSet(ctx, &rsspb.GetCurrentResourceSetRequest{})
	if err != nil {
		return errors.Wrap(err, "GetCurrentCluster")
	}
	log.InfoContextf(ctx, "Updating resource-world-relations in world: %q", localWorldID)

	currentRIDs, err := c.rr.GeometricResourceInstanceData(ctx, response.GetResourceSet())
	if err != nil {
		return errors.Wrap(err, "(Current) GeometricResourceInstanceData")
	}

	// Extract the resource instances and updates separate to the rest of the world.
	if extractResp, err := c.owsClient.ExtractResourceInstances(ctx, &owspb.ExtractResourceInstancesRequest{
		WorldId:              localWorldID,
		ResourceInstanceData: currentRIDs,
	}); err != nil {
		return errors.Wrap(err, "ExtractResourceInstances")
	} else {
		response.GetResourceSet().ObjectWorldUpdates = extractResp.GetUpdateWorldResources()
	}

	// See if it's possible to compose the world from this resource set
	rids, err := c.rr.GeometricResourceInstanceData(ctx, response.GetResourceSet())
	if err != nil {
		return errors.Wrap(err, "GeometricResourceInstanceData")
	}

	var composedWorldID string
	if resp, err := c.owsClient.CreateWorldFromResourceInstances(ctx, &owspb.CreateWorldFromResourceInstancesRequest{
		ResourceInstanceData: rids,
		UpdateWorldResources: response.GetResourceSet().GetObjectWorldUpdates(),
	}); err != nil {
		return errors.Wrap(err, "Could not compose world from new resource set.")
	} else {
		composedWorldID = resp.GetId()
	}
	defer func() {
		// Ignore the error for deleting the world.
		if _, err := c.owsClient.DeleteWorld(ctx, &owspb.DeleteWorldRequest{
			WorldId: composedWorldID,
		}); err != nil {
			log.ErrorContextf(ctx, "DeleteWorld for %q failed: %v", composedWorldID, err)
		}
	}()

	// World is OK -- write to cluster.
	if _, err := c.rssClient.SetCurrentResourceSet(ctx, &rsspb.SetCurrentResourceSetRequest{
		ResourceSet:   response.GetResourceSet(),
		RevisionToken: response.GetRevisionToken(),
	}); err != nil {
		return errors.Wrap(err, "SetCurrentResourceSet")
	}

	return nil
}

// UpdateWorldFromResourceSetOption is an option for UpdateWorldFromResourceSet.
type UpdateWorldFromResourceSetOption interface {
	set(opts *updateWorldFromResourceSetOptions)
}

// SkipInvalidUpdates is an option for UpdateWorldFromResourceSet.
// If true, updates that fail to apply to the world will be skipped.
func SkipInvalidUpdates(skipInvalidUpdates bool) UpdateWorldFromResourceSetOption {
	return updateWorldFromResourceSetOption(func(opts *updateWorldFromResourceSetOptions) { opts.skipInvalidUpdates = skipInvalidUpdates })
}

type updateWorldFromResourceSetOption func(*updateWorldFromResourceSetOptions)

func (o updateWorldFromResourceSetOption) set(opts *updateWorldFromResourceSetOptions) { o(opts) }

type updateWorldFromResourceSetOptions struct {
	skipInvalidUpdates bool
}

// SkippedWorldUpdateWithError holds an update that was skipped and the error that was encountered
// when trying to apply the update.
type SkippedWorldUpdateWithError struct {
	Update *owupb.ObjectWorldUpdate
	Err    error
}

// SkippedSceneObjectWithError holds the name of a scene object that was skipped and the error that
// was encountered when trying to apply the update.
type SkippedSceneObjectWithError struct {
	SceneObjectName string
	Err             error
}

// UpdateWorldFromResourceSetResult holds the result of UpdateWorldFromResourceSet.
type UpdateWorldFromResourceSetResult struct {
	SkippedUpdates      []SkippedWorldUpdateWithError
	SkippedSceneObjects []SkippedSceneObjectWithError
}

// UpdateWorldFromResourceSet updates the world in 'localWorldID' to contain the
// composition of the resource set and all of its world updates. Optionally, the
// caller may enable or disable any of the following:
//   - Pausing the world updater during the world composition (default: enabled)
//   - Skipping world updates that throw errors upon application
//     (default: disabled)
func (c *Client) UpdateWorldFromResourceSet(ctx context.Context, localWorldID string, options ...UpdateWorldFromResourceSetOption) (result UpdateWorldFromResourceSetResult, err error) {
	opts := &updateWorldFromResourceSetOptions{}
	for _, opt := range options {
		opt.set(opts)
	}

	response, e := c.rssClient.GetCurrentResourceSet(ctx, &rsspb.GetCurrentResourceSetRequest{})
	if e != nil {
		err = errors.Wrap(e, "GetCurrentCluster")
		return
	}
	resourceSet := response.GetResourceSet()

	rids, e := c.rr.GeometricResourceInstanceData(ctx, resourceSet)
	if e != nil {
		err = e
		return
	}

	updatePolicy := owspb.CreateWorldFromResourceSetDataRequest_UPDATE_POLICY_FAIL_ON_FIRST_ERROR
	if opts.skipInvalidUpdates {
		updatePolicy = owspb.CreateWorldFromResourceSetDataRequest_UPDATE_POLICY_SKIP_FAILED_UPDATES
	}
	composeResponse, e := c.owsClient.CreateWorldFromResourceSetData(ctx, &owspb.CreateWorldFromResourceSetDataRequest{
		ResourceInstanceData: rids,
		UpdateWorldResources: resourceSet.GetObjectWorldUpdates(),
		UpdatePolicy:         updatePolicy,
	})
	if e != nil {
		err = errors.Wrap(e, "CreateWorldFromResourceSetData")
		return
	}

	for _, compositionError := range composeResponse.GetCompositionErrors() {
		stErr := status.ErrorProto(compositionError.GetStatus())
		switch p := compositionError.Problem.(type) {
		case *owspb.CreateWorldFromResourceSetDataResponse_CompositionError_ProblemUpdate:
			update := p.ProblemUpdate.GetUpdate()
			idx := p.ProblemUpdate.GetIndex()
			log.InfoContextf(ctx, "Failed to apply update at index %d in the resource set, skipping update: %v -- %v", idx, update, stErr)
			result.SkippedUpdates = append(result.SkippedUpdates, SkippedWorldUpdateWithError{Update: update, Err: stErr})
		case *owspb.CreateWorldFromResourceSetDataResponse_CompositionError_ProblemResourceName:
			sceneObjectName := p.ProblemResourceName
			log.InfoContextf(ctx, "Skipping scene object '%s'. Failed to add to the world: %v", sceneObjectName, stErr)
			result.SkippedSceneObjects = append(result.SkippedSceneObjects, SkippedSceneObjectWithError{
				SceneObjectName: sceneObjectName,
				Err:             stErr,
			})
		}
	}

	newWorldID := composeResponse.GetMetadata().GetId()

	// Replace the given world id with the newly composed world.
	defer func() {
		// Delete the world after we're done.
		if _, delErr := c.owsClient.DeleteWorld(ctx, &owspb.DeleteWorldRequest{
			WorldId: newWorldID,
		}); delErr != nil {
			log.ErrorContextf(ctx, "DeleteWorld for %q failed: %v", newWorldID, delErr)
			// Propagate the error to the caller.
			// Not sure if propagating deletion error to the caller is the best strategy  :shrug:. But
			// ignoring the error is not great either.
			err = errors.Wrap(delErr, "DeleteWorld")
		}
	}()
	if _, e := c.owsClient.CloneWorld(ctx, &owspb.CloneWorldRequest{
		WorldId:        newWorldID,
		ClonedWorldId:  localWorldID,
		AllowOverwrite: true, // Overwrite localWorldID with newWorldID
	}); e != nil {
		err = errors.Wrap(e, "CloneWorld")
		return
	}
	return
}

// NewClientOpts holds options for NewClient.
type NewClientOpts struct {
	RSSClient grpcrsspb.HotSharedStateResourceSetServiceClient
	OWSClient grpcowspb.ObjectWorldServiceClient
	RR        ResourceReader
}

// NewClient returns a new resource reader client with the given options.
func NewClient(opts NewClientOpts) *Client {
	return &Client{
		rssClient: opts.RSSClient,
		owsClient: opts.OWSClient,
		rr:        opts.RR,
	}
}
