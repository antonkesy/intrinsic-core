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

package registry

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"time"

	"intrinsic/storage/artifacts/internal/utils"

	"github.com/containerd/containerd/content"
	log "github.com/golang/glog"
	"github.com/google/go-containerregistry/pkg/name"
	"github.com/google/go-containerregistry/pkg/v1/partial"
	"github.com/google/go-containerregistry/pkg/v1/remote"
	"github.com/opencontainers/go-digest"
	v1 "github.com/opencontainers/image-spec/specs-go/v1"
	"go.uber.org/atomic"

	artifactpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

// MemoryManifest is image Manifest updater with in memory temporary cache.
// it implements internal.ContentUpdater
type MemoryManifest struct {
	data        *bytes.Buffer
	request     *artifactpb.UpdateRequest
	finalized   *atomic.Bool
	imageName   name.Reference
	remoteOpts  []remote.Option
	ctx         context.Context
	startedTime time.Time
}

// NewManifestUpdater create internal.ContentUpdater to upload an image manifest to target repository.
func NewManifestUpdater(ctx context.Context, request *artifactpb.UpdateRequest, imgName name.Reference, opts ...remote.Option) (*MemoryManifest, error) {
	updater := &MemoryManifest{
		ctx:         ctx,
		data:        bytes.NewBuffer(make([]byte, 0, 4096)),
		request:     request,
		finalized:   atomic.NewBool(false),
		imageName:   imgName,
		remoteOpts:  opts,
		startedTime: time.Now(),
	}

	return updater, nil
}

// Close implements corresponding method from internal.ContentUpdater.
func (m *MemoryManifest) Close() error {
	return nil // no-op
}

// Update implements corresponding method from internal.ContentUpdater
func (m *MemoryManifest) Update(_ context.Context, request *artifactpb.UpdateRequest) (*artifactpb.UpdateResponse, error) {
	if m.finalized.Load() {
		return nil, ErrUpdaterAlreadyFinalized
	}

	if m.request.Ref != request.Ref {
		return nil, errors.New("invalid reference update, reference mismatch")
	}
	if m.ctx.Err() != nil {
		return nil, fmt.Errorf("already cancelled")
	}

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_UNDEFINED {
		return nil, errors.New("undefined update action")
	}

	log.Infof("[%s]: update request action: %q", utils.AsShortName(request.Ref), request.Action)

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_STAT {
		return utils.ActionStatResponse(m.getStatus(), request), nil
	}

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_ABORT {
		return nil, m.Abort()
	}

	_, err := m.data.Write(request.Data)
	if err != nil {
		return nil, fmt.Errorf("cannot write manifest to local buffer: %w", err)
	}

	if request.Action == artifactpb.UpdateAction_UPDATE_ACTION_COMMIT {
		m.finalized.Store(true)
		err = m.publishManifest()
		if err != nil {
			return nil, fmt.Errorf("cannot commit, error publishing manifest: %w", err)
		}

	}

	return utils.ActionStatResponse(m.getStatus(), request), nil
}

func (m *MemoryManifest) publishManifest() error {
	md := &manifestData{manifest: m.data.Bytes()}

	// Validate all the content is uploaded prior to
	var manifest v1.Manifest

	if err := json.Unmarshal(md.manifest, &manifest); err != nil {
		return fmt.Errorf("cannot read manifest: %w", err)
	}

	// for validation, we are going to treat config as layer (it's the same thing anyway)
	descriptors := append([]v1.Descriptor{manifest.Config}, manifest.Layers...)

	missingRefs := make([]string, 0)

	opts := append(m.remoteOpts, remote.WithContext(m.ctx))

	for _, ref := range descriptors {
		layerDigest := m.imageName.Context().Digest(ref.Digest.String())
		layer, err := remote.Layer(layerDigest, opts...)
		if err != nil {
			missingRefs = append(missingRefs, ref.Digest.String())
			log.Warningf("image %s part %s is missing upstream; got error: %s", m.imageName, ref.Digest, err)
		}
		exists, err := partial.Exists(layer)
		if err != nil {
			missingRefs = append(missingRefs, ref.Digest.String())
			log.Warningf("image %s part %s is missing upstream; got error: %s", m.imageName, ref.Digest, err)
		}
		if !exists {
			missingRefs = append(missingRefs, ref.Digest.String())
			log.Infof("image %s part %s is missing upstream; not uploaded yet", m.imageName, ref.Digest)
		}
	}

	if len(missingRefs) > 0 {
		return fmt.Errorf("cannot continue: following parts missing upstream (%s)", strings.Join(missingRefs, ", "))
	}

	if err := remote.Put(m.imageName, md, opts...); err != nil {
		return fmt.Errorf("cannot push manifest to remote: %w", err)
	}

	return nil
}

// Abort implements corresponding method from internal.ContentUpdater.
func (m *MemoryManifest) Abort() error {
	if m.finalized.CompareAndSwap(false, true) {
		m.data.Truncate(0)
		m.data = nil
	}
	return nil
}

// IsFinalized implements corresponding method from internal.ContentUpdater.
func (m *MemoryManifest) IsFinalized() bool {
	return m.finalized.Load()
}

func (m *MemoryManifest) getStatus() content.Status {
	return content.Status{
		Ref:       m.request.Ref,
		Offset:    int64(m.data.Len()), // at this stage we didn't read anything from buffer, so we are good
		Total:     *m.request.Content.Size,
		Expected:  digest.Digest(utils.ValueOrEmptyStr(m.request.ExpectedDigest)),
		StartedAt: m.startedTime,
		UpdatedAt: time.Now(),
	}
}

type manifestData struct {
	manifest []byte
}

func (md *manifestData) RawManifest() ([]byte, error) {
	return md.manifest, nil
}
