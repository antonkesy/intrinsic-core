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
	"encoding/json"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"sync"
	"time"
)

// store encapsulates the in-memory storage and disk persistence of SyncSets.
type store struct {
	mu            sync.RWMutex
	syncsetDir    string
	retentionDays int
	syncSets      map[string]map[Generation]*SyncSet
	latest        map[string]*SyncSet
}

// newStore creates a new store.
// retentionDays specifies the age in days after which outdated completed SyncSets are deleted from disk during loadActive.
// A zero value for retentionDays configures the store to always delete older completed SyncSets immediately.
// Notice that when an old SyncSet is deleted from disk, the long running operation result for that SyncSet is also deleted.
func newStore(syncsetDir string, retentionDays int) *store {
	return &store{
		syncsetDir:    syncsetDir,
		retentionDays: retentionDays,
		syncSets:      make(map[string]map[Generation]*SyncSet),
		latest:        make(map[string]*SyncSet),
	}
}

// New creates a new SyncSet generation with next generation number and adds it to the store.
func (s *store) New(name string) (*SyncSet, error) {
	if err := validateName(name); err != nil {
		return nil, err
	}
	s.mu.Lock()
	var gen Generation = 1
	if latest, exists := s.latest[name]; exists {
		gen = latest.Generation + 1
	}
	ss := &SyncSet{
		Name:       name,
		Generation: gen,
		State:      SyncStateSyncing,
		Created:    time.Now(),
		Updated:    time.Now(),
	}
	s.addLocked(ss)
	s.mu.Unlock()

	RecordInMemoryMetrics(s)
	return ss, nil
}

var validNameRegex = regexp.MustCompile("^[a-zA-Z0-9_-]{1,128}$")

// validateName checks that a SyncSet name is valid and safe for filesystem usage.
func validateName(name string) error {
	if !validNameRegex.MatchString(name) {
		return fmt.Errorf("invalid syncset name %q: must match %q", name, validNameRegex)
	}
	return nil
}

// SyncSet retrieves a specific generation of a SyncSet from memory.
// It returns an error if the generation is not already loaded.
func (s *store) SyncSet(name string, gen Generation) (*SyncSet, error) {
	if err := validateName(name); err != nil {
		return nil, err
	}
	s.mu.RLock()
	defer s.mu.RUnlock()

	gens := s.syncSets[name]
	if gens == nil {
		return nil, fmt.Errorf("syncset %q not found", name)
	}
	ss, exists := gens[gen]
	if !exists {
		return nil, fmt.Errorf("syncset %q generation %d not found", name, gen)
	}
	return ss, nil
}

// latestSyncSet retrieves the latest generation of a SyncSet by name.
func (s *store) latestSyncSet(name string) (*SyncSet, error) {
	if err := validateName(name); err != nil {
		return nil, err
	}
	s.mu.RLock()
	defer s.mu.RUnlock()

	latest, exists := s.latest[name]
	if !exists {
		return nil, fmt.Errorf("syncset %q not found", name)
	}
	return latest, nil
}

// List returns all SyncSets from the store.
func (s *store) List() []*SyncSet {
	s.mu.RLock()
	defer s.mu.RUnlock()

	var list []*SyncSet
	for _, gens := range s.syncSets {
		for _, ss := range gens {
			list = append(list, ss)
		}
	}
	sort.Slice(list, func(i, j int) bool {
		if list[i].Name != list[j].Name {
			return list[i].Name < list[j].Name
		}
		return list[i].Generation < list[j].Generation
	})
	return list
}

// Names returns all syncset names.
func (s *store) Names() []string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	names := make([]string, 0, len(s.syncSets))
	for name := range s.syncSets {
		names = append(names, name)
	}
	return names
}

// activeObjects returns the union of all object digests across the latest generation of all SyncSets.
func (s *store) activeObjects() map[string]struct{} {
	s.mu.RLock()
	defer s.mu.RUnlock()

	active := make(map[string]struct{})
	for _, ss := range s.latest {
		for _, d := range ss.Objects {
			active[d] = struct{}{}
		}
	}
	return active
}

// Update updates a SyncSet of a specific generation.
func (s *store) Update(name string, gen Generation, updateFn func(*SyncSet)) {
	s.mu.Lock()
	updated := false
	if gens := s.syncSets[name]; gens != nil {
		if ss, exists := gens[gen]; exists {
			updateFn(ss)
			ss.Updated = time.Now()
			updated = true
		}
	}
	s.mu.Unlock()

	if updated {
		RecordInMemoryMetrics(s)
	}
}

// filepath returns the path to the JSON file for a given SyncSet generation.
func (s *store) filepath(name string, gen Generation) string {
	filename := fmt.Sprintf("%s.%d.json", name, gen)
	return filepath.Join(s.syncsetDir, filename)
}

// persist writes a SyncSet to disk.
// By default (`writeErrorStatus == false`), persist does not store any temporary object status (`Status.Objects` is omitted/cleared on disk).
// When `writeErrorStatus == true`, it persists only the last observed error status (`ErrorMsg.Set` or non-OK `ErrorCode`) per object to disk, filtering out non-error temporary statuses.
func (s *store) persist(ss *SyncSet, writeErrorStatus bool) error {
	if ss == nil {
		return fmt.Errorf("cannot persist nil SyncSet")
	}
	if err := validateName(ss.Name); err != nil {
		return err
	}
	path := s.filepath(ss.Name, ss.Generation)
	tempPath := path + ".tmp"

	s.mu.RLock()
	cp := ss.Copy()
	if writeErrorStatus {
		if len(cp.Status.Objects) > 0 {
			for d, st := range cp.Status.Objects {
				if !hasErrorStatus(st) {
					delete(cp.Status.Objects, d)
				}
			}
		}
	} else {
		cp.Status.Objects = nil
	}
	data, err := json.Marshal(cp)
	s.mu.RUnlock()
	if err != nil {
		return err
	}
	if err := os.WriteFile(tempPath, data, 0644); err != nil {
		_ = os.Remove(tempPath) // Cleanup
		return fmt.Errorf("failed to write temp syncset file: %w", err)
	}
	if err := os.Rename(tempPath, path); err != nil {
		_ = os.Remove(tempPath) // Cleanup
		return fmt.Errorf("failed to rename syncset file: %w", err)
	}
	return nil
}

// loadFromFile loads a SyncSet from a JSON file.
func loadFromFile(path string) (*SyncSet, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var ss SyncSet
	if err := json.Unmarshal(data, &ss); err != nil {
		return nil, fmt.Errorf("unmarshalling %q: %w", filepath.Base(path), err)
	}
	return &ss, nil
}

// addLocked adds a SyncSet to the store.
// The caller must hold the write lock on s.mu.
func (s *store) addLocked(ss *SyncSet) {
	gens := s.syncSets[ss.Name]
	if gens == nil {
		gens = make(map[Generation]*SyncSet)
		s.syncSets[ss.Name] = gens
	}
	gens[ss.Generation] = ss

	currentLatest := s.latest[ss.Name]
	if currentLatest == nil || ss.Generation > currentLatest.Generation {
		s.latest[ss.Name] = ss
	}
}

// loadActive loads active SyncSet generations (!ss.State.IsDone()) and the latest
// completed generation per SyncSet name from disk into memory. Older completed generations
// are discarded to conserve memory. Completed generations whose age in days exceeds retentionDays
// that are not the latest completed generation are permanently deleted from disk. When retentionDays
// is 0, all older completed generations are deleted immediately.
// Notice that when an old SyncSet is deleted from disk, any long running operation (LRO) result
// querying that SyncSet is also deleted.
// It returns the number of successfully loaded SyncSets, the number of failed loads,
// and any fatal error (e.g. directory read failure). Non-fatal errors (individual file loading)
// are logged and counted but do not stop the loading process.
func (s *store) loadActive() (int, int, error) {
	slog.Info("SyncSet: Loading active SyncSets", slog.String("directory", s.syncsetDir))
	// Step 1: Read the syncset storage directory to find all JSON generation files.
	files, err := os.ReadDir(s.syncsetDir)
	if err != nil {
		return 0, 0, err
	}

	s.mu.Lock()

	var activeSets []*SyncSet
	var completedSets []*SyncSet
	latestCompleted := make(map[string]*SyncSet)
	failedCount := 0

	// Step 2: Load and partition generations into active (in-progress) vs completed generations.
	for _, f := range files {
		if f.IsDir() || filepath.Ext(f.Name()) != ".json" {
			continue
		}

		path := filepath.Join(s.syncsetDir, f.Name())
		ss, err := loadFromFile(path)
		if err != nil {
			slog.Error("SyncSet: Failed to load syncset from file", slog.String("path", path), slog.Any("error", err))
			failedCount++
			continue
		}

		if !ss.State.IsDone() {
			activeSets = append(activeSets, ss)
		} else {
			completedSets = append(completedSets, ss)
			// Step 3: For completed generations, retain only the latest completed generation
			// per SyncSet name to minimize memory consumption.
			cur, exists := latestCompleted[ss.Name]
			if !exists || ss.Generation > cur.Generation {
				latestCompleted[ss.Name] = ss
			}
		}
	}

	// Delete completed generations from disk if their age exceeds retentionDays (or immediately if retentionDays == 0),
	// provided they are not the latest completed generation for that name.
	// Note: deleting an old SyncSet also deletes the long running operation result for that SyncSet.
	for _, ss := range completedSets {
		if ss != latestCompleted[ss.Name] && (s.retentionDays == 0 || time.Since(ss.Updated) > time.Duration(s.retentionDays)*24*time.Hour) {
			path := s.filepath(ss.Name, ss.Generation)
			if err := os.Remove(path); err != nil {
				slog.Error("SyncSet: Failed to delete old completed syncset file", slog.String("path", path), slog.Any("error", err))
			} else {
				slog.Info("SyncSet: Deleted old completed syncset file", slog.String("path", path), slog.String("name", ss.Name), slog.Uint64("generation", uint64(ss.Generation)))
			}
		}
	}

	// Step 4: Add all active generations and the latest completed generations into
	// the in-memory store.
	loadedCount := 0
	for _, ss := range activeSets {
		s.addLocked(ss)
		slog.Debug("SyncSet: Loaded active generation",
			slog.String("name", ss.Name), slog.Uint64("generation", uint64(ss.Generation)),
			slog.Int("objects_count", len(ss.Objects)), slog.String("state", string(ss.State)))
		loadedCount++
	}
	for _, ss := range latestCompleted {
		s.addLocked(ss)
		slog.Debug("SyncSet: Loaded latest completed generation",
			slog.String("name", ss.Name), slog.Uint64("generation", uint64(ss.Generation)),
			slog.Int("objects_count", len(ss.Objects)), slog.String("state", string(ss.State)))
		loadedCount++
	}

	s.mu.Unlock()

	RecordInMemoryMetrics(s)

	return loadedCount, failedCount, nil
}
