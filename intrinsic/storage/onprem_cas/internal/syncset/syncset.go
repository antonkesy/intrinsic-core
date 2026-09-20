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

package syncset

import (
	"fmt"
	"maps"
	"slices"
	"sync"
	"time"

	"intrinsic/storage/onprem_cas/internal/activeset"
)

type Digest = activeset.Digest

// Optional is a generic wrapper representing a value that may or may not be set.
type Optional[T any] = activeset.Optional[T]

// ObjectStatus represents the state of an object in a SyncSet.
type ObjectStatus = activeset.ObjectStatus

// NewOptional creates a populated Optional[T].
func NewOptional[T any](val T) Optional[T] {
	return activeset.NewOptional(val)
}

// SyncState represents the minimal state of a sync set operation.
type SyncState string

const (
	// SyncStateSyncing indicates the SyncSet is still reconciling.
	SyncStateSyncing SyncState = "SYNCING"
	// SyncStateDone indicates that the SyncSet finished reconciling successfully.
	// All requested objects are available locally and upstream.
	SyncStateDone SyncState = "DONE"
	// SyncStateDoneError indicates that the SyncSet finished reconciling but encountered permanent errors.
	// Permanent errors might result in data not being uploaded to the cloud or objects missing locally.
	SyncStateDoneError SyncState = "DONE_ERROR"
)

// activeStates contains all SyncStates that indicate a SyncSet is still reconciling.
var activeStates = []SyncState{SyncStateSyncing}

// doneStates contains all terminal SyncStates.
var doneStates = []SyncState{SyncStateDone, SyncStateDoneError}

// IsActive returns true if the SyncState indicates reconciling.
func (s SyncState) IsActive() bool {
	return slices.Contains(activeStates, s)
}

// IsDone returns true if the SyncState is a terminal state.
func (s SyncState) IsDone() bool {
	return slices.Contains(doneStates, s)
}

// SyncSetStatus holds the transient, in-memory progress of a sync set operation.
type SyncSetStatus struct {
	mu      sync.RWMutex
	Objects map[Digest]Optional[ObjectStatus] `json:"objects,omitempty"` // Map keyed by object digest
}

// GetObjectStatus retrieves the ObjectStatus for the given digest in a thread-safe manner.
func (s *SyncSetStatus) GetObjectStatus(d Digest) (Optional[ObjectStatus], bool) {
	if s == nil {
		return Optional[ObjectStatus]{}, false
	}
	s.mu.RLock()
	defer s.mu.RUnlock()
	status, ok := s.Objects[d]
	return status, ok
}

// SetObjectStatus sets the ObjectStatus for the given digest in a thread-safe manner.
func (s *SyncSetStatus) SetObjectStatus(d Digest, status Optional[ObjectStatus]) {
	if s == nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.Objects == nil {
		s.Objects = make(map[Digest]Optional[ObjectStatus])
	}
	s.Objects[d] = status
}

// Update safely modifies the Objects map while holding the write lock.
func (s *SyncSetStatus) Update(fn func(objects map[Digest]Optional[ObjectStatus])) {
	if s == nil || fn == nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.Objects == nil {
		s.Objects = make(map[Digest]Optional[ObjectStatus])
	}
	fn(s.Objects)
}

// Generation represents the monotonically increasing sequence number of a SyncSet.
type Generation uint64

// SyncSet represents a set of objects that should be synchronized.
type SyncSet struct {
	// Name is an identifier of the SyncSet. Name + Generation are unique.
	Name string `json:"name"`
	// Generation is monotonically increasing and indicates the updates to the SyncSet.
	Generation Generation `json:"generation"`
	// The digests associated with the SyncSet.
	Objects []Digest `json:"objects"`
	// State indicates the state of the SyncSet in its lifecycle.
	State SyncState `json:"state"`
	// Created is the timestamp when the SyncSet was created.
	Created time.Time `json:"created"`
	// Updated is the timestamp when the SyncSet was last updated.
	Updated time.Time `json:"updated"`
	// Status provides detailed visibility into the sync process or last observed error state per object.
	// By default, temporary status is excluded from JSON persistence (`Objects` is empty on disk unless errors occur and are requested to be written).
	Status SyncSetStatus `json:"status,omitzero"`
}

// Key returns the formatted string key ("name/generation") identifying this SyncSet generation.
func (s *SyncSet) Key() string {
	return fmt.Sprintf("%s/%d", s.Name, s.Generation)
}

// Copy creates a deep copy of the SyncSet, including cloning the Objects slice and the Status object map.
func (s *SyncSet) Copy() *SyncSet {
	if s == nil {
		return nil
	}
	cp := &SyncSet{
		Name:       s.Name,
		Generation: s.Generation,
		State:      s.State,
		Created:    s.Created,
		Updated:    s.Updated,
	}
	if len(s.Objects) > 0 {
		cp.Objects = slices.Clone(s.Objects)
	}
	s.Status.mu.RLock()
	if len(s.Status.Objects) > 0 {
		objectsCopy := make(map[Digest]Optional[ObjectStatus], len(s.Status.Objects))
		maps.Copy(objectsCopy, s.Status.Objects)
		cp.Status.Objects = objectsCopy
	}
	s.Status.mu.RUnlock()
	return cp
}
