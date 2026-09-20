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

// Package syncutils provides synchronization utilities.
package syncutils

import (
	"context"
	"sync"
)

// SyncMap is a sync.Map-like struct that provides concrete typing and additional functionality.
type SyncMap[K comparable, V any] struct {
	m sync.Map
}

// Load returns the value stored in the map for a key, or nil if no value is present.
//
// The ok result indicates whether value was found in the map.
func (m *SyncMap[K, V]) Load(key K) (value V, ok bool) {
	valueAny, ok := m.m.Load(key)
	if ok {
		value = valueAny.(V)
	}
	return value, ok
}

// Store sets the value for a key.
func (m *SyncMap[K, V]) Store(key K, value V) {
	m.m.Store(key, value)
}

// LoadOrStore returns the existing value for the key if present.
//
// Otherwise, it stores and returns the given value.
//
// The loaded result is true if the value was loaded, false if stored.
func (m *SyncMap[K, V]) LoadOrStore(key K, value V) (actual V, loaded bool) {
	actualAny, loaded := m.m.LoadOrStore(key, value)
	return actualAny.(V), loaded
}

// Delete deletes the value for a key.
func (m *SyncMap[K, V]) Delete(key K) {
	m.m.Delete(key)
}

// LoadAndDelete deletes the value for a key, returning the previous value if any.
// The loaded result reports whether the key was present.
func (m *SyncMap[K, V]) LoadAndDelete(key K) (value V, loaded bool) {
	valueAny, loaded := m.m.LoadAndDelete(key)
	if loaded {
		value = valueAny.(V)
	}
	return value, loaded
}

// Range calls f sequentially for each key and value present in the map.
// If f returns false, range stops the iteration.
func (m *SyncMap[K, V]) Range(f func(key K, value V) bool) {
	m.m.Range(func(keyAny, valueAny any) bool {
		return f(keyAny.(K), valueAny.(V))
	})
}

type valueWithError[V any] struct {
	value V
	err   error
}

// SyncCachedFunction is a function that is cached per key.
//
// It is thread-safe and caches results to avoid redundant calls to the underlying function.
type SyncCachedFunction[K comparable, V any] struct {
	f func(context.Context, K) (V, error)
	m SyncMap[K, func() valueWithError[V]]
}

// NewSyncCachedFunction returns a new SyncCachedFunction.
func NewSyncCachedFunction[K comparable, V any](f func(context.Context, K) (V, error)) *SyncCachedFunction[K, V] {
	return &SyncCachedFunction[K, V]{
		f: f,
	}
}

// Call returns the result of calling the specified function for the given key.
//
// If the function has not yet been called for the key, it is called and the result is stored.
//
// Otherwise, the previously cached result is returned.
func (s *SyncCachedFunction[K, V]) Call(ctx context.Context, key K) (V, error) {
	f, _ := s.m.LoadOrStore(key, sync.OnceValue(func() valueWithError[V] {
		value, err := s.f(ctx, key)
		return valueWithError[V]{value: value, err: err}
	}))
	result := f()
	return result.value, result.err
}
