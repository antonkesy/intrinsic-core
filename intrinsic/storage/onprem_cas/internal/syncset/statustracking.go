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
	"sync"

	"google.golang.org/grpc/codes"
)

// objCategory represents the mutually exclusive lifecycle classification of an object status in the central cache.
type objCategory int

const (
	// catNeedsUpload indicates the object is not yet available upstream and requires stat/upload.
	catNeedsUpload objCategory = iota
	// catNeedsDownload indicates the object is available upstream but missing locally, requiring download.
	catNeedsDownload
	// catDlError indicates an error occurred while trying to download the object locally.
	catDlError
	// catUpError indicates an error occurred while trying to stat or upload the object upstream.
	catUpError
	// catDone indicates the object is fully reconciled and available both locally and upstream.
	catDone
)

// classify categorizes an object status into its intrinsic lifecycle state category.
// Notice that classification is independent of supersedence; whether download requirements
// or download errors affect a SyncSet's aggregate state is decided later in getSyncState.
func classify(st *ObjectStatus) objCategory {
	// If untracked or nil, we assume it has not yet been discovered upstream.
	if st == nil {
		return catNeedsUpload
	}
	// If an error is set, classify by whether it occurred during download vs upload/stat.
	if st.ErrorMsg.Set {
		if st.Upstream.Set && st.Upstream.Val {
			return catDlError
		}
		return catUpError
	}
	// If not confirmed available upstream, it needs upload or stat.
	if !st.Upstream.Set || !st.Upstream.Val {
		return catNeedsUpload
	}
	// If available upstream but not confirmed locally, it needs local download.
	if !st.Local.Set || !st.Local.Val {
		return catNeedsDownload
	}
	// Available both locally and upstream.
	return catDone
}

// syncSetProgress maintains aggregate counts of objects in each lifecycle category for a single SyncSet.
// Using counters instead of per-category object maps significantly reduces memory overhead.
type syncSetProgress struct {
	objects       []Digest
	needsUpload   int
	needsDownload int
	dlErrors      int
	upErrors      int
	done          int
}

// syncSetKey identifies a specific generation of a SyncSet in the progress tracking map without string parsing.
type syncSetKey struct {
	name       string
	generation Generation
}

// statusTracking manages global object status caching and per-SyncSet progress counting.
// It shares a single object status instance across multiple SyncSets that reference the same digest.
type statusTracking struct {
	mu sync.Mutex
	// statuses stores the latest ObjectStatus for every actively tracked digest across all SyncSets.
	statuses map[Digest]*ObjectStatus
	// syncSets maps a SyncSet key (name + generation) to its aggregate category counters.
	syncSets map[syncSetKey]*syncSetProgress
	// refCounts tracks how many actively syncing SyncSets reference each digest, allowing garbage collection.
	refCounts map[Digest]int
}

// newStatusTracking initializes a new statusTracking instance.
func newStatusTracking() *statusTracking {
	return &statusTracking{
		statuses:  make(map[Digest]*ObjectStatus),
		syncSets:  make(map[syncSetKey]*syncSetProgress),
		refCounts: make(map[Digest]int),
	}
}

// add registers the static list of objects for a SyncSet generation in the central cache.
// If the key is already registered, this is a no-op.
func (st *statusTracking) add(key syncSetKey, objects []Digest) {
	st.mu.Lock()
	defer st.mu.Unlock()

	if st.syncSets[key] != nil {
		return
	}

	prog := &syncSetProgress{
		objects: objects,
	}
	for _, d := range objects {
		st.refCounts[d]++
	}
	st.syncSets[key] = prog
	st.recalculateLocked(prog)
}

// recalculateLocked re-tallies the aggregate category counters across the static objects for a SyncSet.
// The caller must hold st.mu.
func (st *statusTracking) recalculateLocked(prog *syncSetProgress) {
	prog.needsUpload = 0
	prog.needsDownload = 0
	prog.dlErrors = 0
	prog.upErrors = 0
	prog.done = 0
	for _, d := range prog.objects {
		switch classify(st.statuses[d]) {
		case catNeedsUpload:
			prog.needsUpload++
		case catNeedsDownload:
			prog.needsDownload++
		case catDlError:
			prog.dlErrors++
		case catUpError:
			prog.upErrors++
		case catDone:
			prog.done++
		}
	}
}

// updateStatus records an updated object status in the central cache and recalculates category counts for a SyncSet.
func (st *statusTracking) updateStatus(key syncSetKey, status *ObjectStatus) {
	st.mu.Lock()
	defer st.mu.Unlock()

	// 1. Update the central status cache if a new status report arrived.
	if status != nil {
		st.statuses[status.Digest] = status.Copy()
	}

	// 2. Re-tally the aggregate category counters across the static objects registered for this key.
	prog := st.syncSets[key]
	if prog == nil {
		return
	}
	st.recalculateLocked(prog)
}

// getSyncState determines the lifecycle state of a SyncSet from its current category counts.
// Superseded generations ignore download requirements and download errors.
func (st *statusTracking) getSyncState(key syncSetKey, isSuperseded bool) SyncState {
	st.mu.Lock()
	defer st.mu.Unlock()

	prog := st.syncSets[key]
	if prog == nil {
		return SyncStateSyncing
	}
	// 1. Any object needing upstream upload keeps the SyncSet syncing, even if superseded.
	if prog.needsUpload > 0 {
		return SyncStateSyncing
	}
	// 2. If superseded, local download requirements and download errors are ignored.
	if isSuperseded {
		if prog.upErrors > 0 {
			return SyncStateDoneError
		}
		return SyncStateDone
	}
	// 3. For active (non-superseded) generations, check if local downloads are still ongoing.
	if prog.needsDownload > 0 {
		return SyncStateSyncing
	}
	// 4. If any download or upload errors occurred, finish in an error state.
	if prog.dlErrors > 0 || prog.upErrors > 0 {
		return SyncStateDoneError
	}
	return SyncStateDone
}

// delete evicts tracking maps for a SyncSet from the cache, and removes object statuses no longer referenced by any active SyncSet.
func (st *statusTracking) delete(key syncSetKey) {
	st.mu.Lock()
	defer st.mu.Unlock()
	prog := st.syncSets[key]
	if prog == nil {
		return
	}
	delete(st.syncSets, key)

	for _, d := range prog.objects {
		st.refCounts[d]--
		if st.refCounts[d] <= 0 {
			delete(st.refCounts, d)
			delete(st.statuses, d)
		}
	}
}

// isTracked returns true if a SyncSet has active progress tracking in the cache.
func (st *statusTracking) isTracked(k syncSetKey) bool {
	st.mu.Lock()
	defer st.mu.Unlock()
	return st.syncSets[k] != nil
}

// populateStatus copies cached object statuses into a SyncSet's per-object status map.
// Empty [Optional]s are returned for each object if the [SyncSet] is not tracked.
func (st *statusTracking) populateStatus(ss *SyncSet) *SyncSet {
	if ss == nil {
		return ss
	}
	key := syncSetKey{name: ss.Name, generation: ss.Generation}
	st.mu.Lock()
	// 1. If the SyncSet is no longer actively tracked (completed or evicted), return unset Optionals.
	if st.syncSets[key] == nil {
		st.mu.Unlock()
		ss.Status.Update(func(objects map[Digest]Optional[ObjectStatus]) {
			for _, d := range ss.Objects {
				objects[d] = Optional[ObjectStatus]{}
			}
		})
		return ss
	}

	// 2. Copy matching statuses under the cache mutex to avoid races during external map population.
	statuses := make(map[Digest]*ObjectStatus, len(ss.Objects))
	for _, d := range ss.Objects {
		if status := st.statuses[d]; status != nil {
			statuses[d] = status.Copy()
		}
	}
	st.mu.Unlock()

	ss.Status.Update(func(objects map[Digest]Optional[ObjectStatus]) {
		for _, d := range ss.Objects {
			if status := statuses[d]; status != nil {
				objects[d] = NewOptional(*status)
			} else {
				objects[d] = Optional[ObjectStatus]{}
			}
		}
	})
	return ss
}

// populateStatuses copies cached object statuses across a batch of [SyncSet]s in bulk.
// The input slice list must not contain any nil pointers.
// It acquires st.mu exactly once to extract all matching object statuses across the entire list,
// eliminating repeated mutex lock/unlock cycling when listing large numbers of active SyncSets.
func (st *statusTracking) populateStatuses(list []*SyncSet) []*SyncSet {
	if len(list) == 0 {
		return list
	}

	// Phase 1: Snapshot cached object statuses while holding st.mu.
	// We extract shallow copies of each object's status so we can release st.mu safely
	// before populating individual SyncSet status maps.
	st.mu.Lock()
	allStatuses := make([]map[Digest]*ObjectStatus, len(list))
	for i, ss := range list {
		key := syncSetKey{name: ss.Name, generation: ss.Generation}
		// If the SyncSet is no longer actively tracked (completed or evicted),
		// we leave allStatuses[i] as nil so Phase 2 assigns unset Optionals.
		if st.syncSets[key] == nil {
			continue
		}
		statuses := make(map[Digest]*ObjectStatus, len(ss.Objects))
		for _, d := range ss.Objects {
			if status := st.statuses[d]; status != nil {
				statuses[d] = status.Copy()
			}
		}
		allStatuses[i] = statuses
	}
	st.mu.Unlock()

	// Phase 2: Populate each SyncSet's internal status map without holding st.mu.
	// Updating ss.Status.Objects acquires ss.Status.mu; keeping st.mu unlocked here
	// guarantees there are no lock ordering deadlocks between st.mu and ss.Status.mu.
	for i, ss := range list {
		statuses := allStatuses[i]
		// If allStatuses[i] is nil (untracked SyncSet), populate with unset Optionals.
		if statuses == nil {
			ss.Status.Update(func(objects map[Digest]Optional[ObjectStatus]) {
				for _, d := range ss.Objects {
					objects[d] = Optional[ObjectStatus]{}
				}
			})
			continue
		}
		// Otherwise, populate with the snapshotted status from the central cache,
		// or an unset Optional if the specific object status is not currently known.
		ss.Status.Update(func(objects map[Digest]Optional[ObjectStatus]) {
			for _, d := range ss.Objects {
				if status := statuses[d]; status != nil {
					objects[d] = NewOptional(*status)
				} else {
					objects[d] = Optional[ObjectStatus]{}
				}
			}
		})
	}
	return list
}

// hasErrorStatus returns true if the status contains an active error message or a non-OK gRPC error code.
func hasErrorStatus(st Optional[ObjectStatus]) bool {
	if !st.Set {
		return false
	}
	return st.Val.ErrorMsg.Set || (st.Val.ErrorCode.Set && st.Val.ErrorCode.Val != codes.OK)
}
