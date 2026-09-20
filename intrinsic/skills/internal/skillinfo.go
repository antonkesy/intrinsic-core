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

// Package skillinfo contains a client that fetches Skills from a Skill
// Information service. It caches both the gRPC connection and any previously
// retrieved skill data.
package skillinfo

import (
	"context"
	"sync"

	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/credentials/insecure"
	grpcstatus "google.golang.org/grpc/status"

	skillservicegrpcpb "intrinsic/skills/proto/skill_service_go_proto"
	spb "intrinsic/skills/proto/skills_go_proto"

	emptypb "google.golang.org/protobuf/types/known/emptypb"
)

// Client is an interface that facilitates fetching skill information.
type Client interface {
	Get(ctx context.Context, idVersion string, address string) (*spb.Skill, error)
	ReleaseFromCache(idVersion string)
}

type skillEntry struct {
	skill   *spb.Skill
	conn    *grpc.ClientConn
	mu      *sync.RWMutex
	deleted bool
}

// CachedClient implements a cached version of a skillinfo Client.
type CachedClient struct {
	skills sync.Map
}

// Get fetches skill information for the given skill at address.
func (c *CachedClient) Get(ctx context.Context, idVersion string, address string) (*spb.Skill, error) {
	e, _ := c.skills.LoadOrStore(idVersion, &skillEntry{mu: &sync.RWMutex{}})
	entry := e.(*skillEntry)
	entry.mu.Lock()
	defer entry.mu.Unlock()

	// This rare condition should only happen if a Get is requested shortly after
	// a ReleaseFromCache. This normally means the skill has just stopped running
	// and so we return Unavailable.
	if entry.deleted {
		return nil, grpcstatus.Errorf(codes.Unavailable, "skill info service for skill %s at %s is unavailable", idVersion, address)
	}

	if entry.skill == nil && entry.conn == nil {
		conn, err := grpc.Dial(address, grpc.WithTransportCredentials(insecure.NewCredentials()), grpc.WithDisableServiceConfig())
		if err != nil {
			return nil, grpcstatus.Errorf(codes.Internal, "could not establish skill info connection for skill %s at %s", idVersion, address)
		}
		entry.conn = conn
	}

	if entry.skill == nil {
		client := skillservicegrpcpb.NewSkillInformationClient(entry.conn)
		resp, err := client.GetSkillInfo(ctx, &emptypb.Empty{})
		if grpcstatus.Code(err) == codes.Unavailable {
			return nil, grpcstatus.Errorf(codes.Unavailable, "skill info service for skill %s at %s is unavailable", idVersion, address)
		} else if err != nil {
			return nil, grpcstatus.Errorf(codes.Internal, "could not reach skill info service for skill %s at %s", idVersion, address)
		}
		entry.skill = resp.Skill
		entry.skill.IdVersion = idVersion
	}

	return entry.skill, nil
}

// ReleaseFromCache frees any cached information for the given skill.
func (c *CachedClient) ReleaseFromCache(idVersion string) {
	e, exists := c.skills.LoadAndDelete(idVersion)
	if !exists {
		return
	}
	// Note that another thread may still have access to this entry. We mark the
	// entry as deleted to prevent other threads from reopening the connection.
	entry := e.(*skillEntry)
	entry.mu.Lock()
	defer entry.mu.Unlock()
	if entry.conn != nil {
		entry.conn.Close()
		entry.conn = nil
		entry.deleted = true
	}
}

// NewCachedClient returns a new cached client that is used to retrieve skill
// info data. It holds on to gRPC channels and also caches previously-retrieve
// skills until ReleaseFromCache is called.
func NewCachedClient() Client {
	return &CachedClient{
		skills: sync.Map{},
	}
}
