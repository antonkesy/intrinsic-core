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
	"context"
	"sync"

	"go.opencensus.io/stats"
	"go.opencensus.io/stats/view"
	"go.opencensus.io/tag"
)

var (
	KeyName  = tag.MustNewKey("name")
	KeyState = tag.MustNewKey("state")

	MSyncSetsInMemory = stats.Int64("syncset/in_memory", "Number of SyncSets in memory per name and state", stats.UnitDimensionless)
)

var (
	SyncSetsInMemoryView = &view.View{
		Name:        "onpremcas/syncset/in_memory",
		Measure:     MSyncSetsInMemory,
		Description: "Number of SyncSets in memory per name and state",
		TagKeys:     []tag.Key{KeyName, KeyState},
		Aggregation: view.LastValue(),
	}

	Views = []*view.View{
		SyncSetsInMemoryView,
	}
)

type nameStateKey struct {
	name  string
	state SyncState
}

// knownKeys records every (name, state) pair ever observed so that if all SyncSets for
// a given pair are removed or transition away, RecordInMemoryMetrics still records 0
// for that tag combination rather than omitting it (ensuring LastValue gauge resets to 0).
var (
	knownKeysMu sync.Mutex
	knownKeys   = make(map[nameStateKey]struct{})
)

// RecordInMemoryMetrics iterates all SyncSets in the store and records the number of SyncSets per (name, state).
func RecordInMemoryMetrics(s *store) {
	if s == nil {
		return
	}
	counts := make(map[nameStateKey]int64)
	for _, ss := range s.List() {
		if ss == nil {
			continue
		}
		counts[nameStateKey{name: ss.Name, state: ss.State}]++
	}

	knownKeysMu.Lock()
	for k := range counts {
		knownKeys[k] = struct{}{}
	}
	keys := make([]nameStateKey, 0, len(knownKeys))
	for k := range knownKeys {
		keys = append(keys, k)
	}
	knownKeysMu.Unlock()

	ctx := context.Background()
	for _, k := range keys {
		count := counts[k]
		mutators := []tag.Mutator{
			tag.Upsert(KeyName, k.name),
			tag.Upsert(KeyState, string(k.state)),
		}
		if ctxTagged, err := tag.New(ctx, mutators...); err == nil {
			stats.Record(ctxTagged, MSyncSetsInMemory.M(count))
		}
	}
}
