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

// Package skillruntime contains client code that can read/write SkillRuntime protos
package skillruntime

import (
	"context"

	"intrinsic/util/grpc/statusutil"

	"google.golang.org/protobuf/proto"

	rdbgrpcpb "intrinsic/resources/proto/runtime_db_go_proto"
	rdbpb "intrinsic/resources/proto/runtime_db_go_proto"
	srpb "intrinsic/skills/proto/skill_runtime_go_proto"
)

const (
	bucket = "skills"
)

// Client contains functions to read/write SkillRuntime information.
type Client interface {
	Add(ctx context.Context, sr *srpb.SkillRuntime) error
	BatchPut(context.Context, []*srpb.SkillRuntime) error
	Get(ctx context.Context, idVersion string) (*srpb.SkillRuntime, error)
	BatchGet(context.Context, []string) ([]*srpb.SkillRuntime, error)
	List(ctx context.Context) ([]string, error)
	// Delete removes the particular idVersion.  It will not return an error for
	// the entry does not exist.
	Delete(ctx context.Context, idVersion string) error
	// BatchDelete removes the specified idVersions.  It will not return an error
	// if any entry does not exist.
	BatchDelete(ctx context.Context, idVersions []string) error
	Clear(ctx context.Context) error
}

type runtimeDBBackedClient struct {
	client rdbgrpcpb.RuntimeDbClient
}

func (s *runtimeDBBackedClient) Add(ctx context.Context, sr *srpb.SkillRuntime) error {
	marshalled, err := proto.Marshal(sr)
	if err != nil {
		return statusutil.NewInternalError("could not marshal proto value")
	}
	_, err = s.client.Put(ctx,
		&rdbpb.PutRequest{
			Bucket: bucket,
			Key:    sr.GetIdVersion(),
			Value:  marshalled,
		})
	if err != nil {
		return statusutil.Wrap(err, "could not set skill runtime")
	}
	return nil
}

func (s *runtimeDBBackedClient) BatchPut(ctx context.Context, srs []*srpb.SkillRuntime) error {
	values := make(map[string][]byte, len(srs))
	for _, sr := range srs {
		marshalled, err := proto.Marshal(sr)
		if err != nil {
			return statusutil.NewInternalError("could not marshal proto value for %q", sr.GetIdVersion())
		}
		values[sr.GetIdVersion()] = marshalled
	}
	if _, err := s.client.BatchPut(ctx, &rdbpb.BatchPutRequest{
		Bucket: bucket,
		Values: values,
	}); err != nil {
		return statusutil.Wrap(err, "could not set skill runtime")
	}
	return nil
}

func (s *runtimeDBBackedClient) Get(ctx context.Context, idVersion string) (*srpb.SkillRuntime, error) {
	resp, err := s.client.Get(ctx,
		&rdbpb.GetRequest{
			Bucket: bucket,
			Key:    idVersion,
		})
	if err != nil {
		return nil, statusutil.Wrap(err, "could not retrieve skill runtime info for %q", idVersion)
	}
	unmarshalled := &srpb.SkillRuntime{}
	if err := proto.Unmarshal(resp.GetValue(), unmarshalled); err != nil {
		return nil, statusutil.NewInternalError("retrieved but could not unmarshal value for key %q", idVersion)
	}
	return unmarshalled, nil
}

func (s *runtimeDBBackedClient) BatchGet(ctx context.Context, idVersions []string) ([]*srpb.SkillRuntime, error) {
	resp, err := s.client.BatchGet(ctx, &rdbpb.BatchGetRequest{
		Bucket: bucket,
		Keys:   idVersions,
	})
	if err != nil {
		return nil, statusutil.Wrap(err, "could not retrieve skill runtime info")
	}
	var srs []*srpb.SkillRuntime
	if len(idVersions) > 0 {
		srs = make([]*srpb.SkillRuntime, 0, len(idVersions))
	}
	for _, k := range idVersions {
		v, ok := resp.GetValues()[k]
		if !ok {
			return nil, statusutil.NewInternalError("id version %q not contained in successful response", k)
		}
		unmarshalled := &srpb.SkillRuntime{}
		if err := proto.Unmarshal(v, unmarshalled); err != nil {
			return nil, statusutil.NewInternalError("retrieved but could not unmarshal value for key %q", k)
		}
		srs = append(srs, unmarshalled)
	}
	return srs, nil
}

func (s *runtimeDBBackedClient) List(ctx context.Context) ([]string, error) {
	resp, err := s.client.List(ctx, &rdbpb.ListRequest{Bucket: bucket})
	if err != nil {
		return nil, statusutil.Wrap(err, "could not retrieve all skill runtime infos: %v")
	}
	return resp.GetKeys(), nil
}

func (s *runtimeDBBackedClient) Delete(ctx context.Context, idVersion string) error {
	_, err := s.client.Delete(ctx, &rdbpb.DeleteRequest{Bucket: bucket, Key: idVersion})
	if err != nil {
		return statusutil.Wrap(err, "could not delete skill runtime info for %q", idVersion)
	}
	return nil
}

func (s *runtimeDBBackedClient) BatchDelete(ctx context.Context, idVersions []string) error {
	_, err := s.client.BatchDelete(ctx, &rdbpb.BatchDeleteRequest{
		Bucket: bucket,
		Keys:   idVersions,
	})
	if err != nil {
		return statusutil.Wrap(err, "could not delete skill runtime infos")
	}
	return nil
}

func (s *runtimeDBBackedClient) Clear(ctx context.Context) error {
	_, err := s.client.Clear(ctx, &rdbpb.ClearRequest{Bucket: bucket})
	if err != nil {
		return statusutil.Wrap(err, "could not clear all skill runtime infos")
	}
	return nil
}

// CreateClient creates a new runtime-db-backed client that can read/write
// SkillRuntime protos.
func CreateClient(client rdbgrpcpb.RuntimeDbClient) Client {
	return &runtimeDBBackedClient{
		client: client,
	}
}
