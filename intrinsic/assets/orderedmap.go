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

// Package orderedmap implements a map that preserves the insertion order of its keys.
package orderedmap

import (
	"fmt"
	"iter"
	"maps"
	"slices"
	"strings"
)

// OrderedMap is a map that maintains the insertion order of keys.
type OrderedMap[K comparable, V any] struct {
	data  map[K]V
	order []K
}

// New returns a new OrderedMap.
func New[K comparable, V any]() *OrderedMap[K, V] {
	return &OrderedMap[K, V]{
		data:  make(map[K]V),
		order: make([]K, 0),
	}
}

// FromMap creates an OrderedMap from a map and an order slice.
//
// The slice and map must have the same length and the slice must contain all keys from the map.
func FromMap[K comparable, V any](m map[K]V, order []K) (*OrderedMap[K, V], error) {
	if len(m) != len(order) {
		return nil, fmt.Errorf("map and order must have the same length - got lengths %d (map) and %d (order)", len(m), len(order))
	}

	mNew := make(map[K]V, len(m))
	for _, key := range order {
		if _, ok := m[key]; !ok {
			return nil, fmt.Errorf("key %v not found in map", key)
		}
		mNew[key] = m[key]
	}

	if len(mNew) != len(m) {
		var extraKeys []string
		for k := range m {
			if _, ok := mNew[k]; !ok {
				extraKeys = append(extraKeys, fmt.Sprintf("%v", k))
			}
		}
		return nil, fmt.Errorf("map has key-value pairs that are not present in order: %s", strings.Join(extraKeys, ", "))
	}

	return &OrderedMap[K, V]{
		data:  mNew,
		order: slices.Clone(order),
	}, nil
}

// Get returns the value for the given key.
//
// The ok result is true if the key was found in the map, false otherwise.
func (m *OrderedMap[K, V]) Get(key K) (V, bool) {
	value, ok := m.data[key]
	return value, ok
}

// Set sets the value for the given key.
//
// If the key already exists, the value is updated and the key is moved to the end of the order.
func (m *OrderedMap[K, V]) Set(key K, value V) {
	if _, exists := m.data[key]; exists {
		// Remove key from the ordered slice
		i := slices.Index(m.order, key)
		if i != -1 {
			m.order = slices.Delete(m.order, i, i+1)
		}
	}
	m.data[key] = value
	m.order = append(m.order, key)
}

// Delete removes the key-value pair from the OrderedMap if the key exists.
//
// It does nothing if the key does not exist.
func (m *OrderedMap[K, V]) Delete(key K) {
	if _, exists := m.data[key]; !exists {
		return
	}
	delete(m.data, key)
	i := slices.Index(m.order, key)
	if i != -1 {
		m.order = slices.Delete(m.order, i, i+1)
	}
}

// Len returns the number of key-value pairs in the OrderedMap.
func (m *OrderedMap[K, V]) Len() int {
	return len(m.order)
}

// AsMap returns a copy of the underlying map.
func (m *OrderedMap[K, V]) AsMap() map[K]V {
	return maps.Clone(m.data)
}

// Keys returns an iterator over the keys of the OrderedMap in insertion order.
//
// It is safe to delete keys from the OrderedMap during iteration.
func (m *OrderedMap[K, V]) Keys() iter.Seq[K] {
	return func(yield func(K) bool) {
		for _, key := range slices.Clone(m.order) {
			if _, ok := m.data[key]; !ok {
				continue
			}
			if !yield(key) {
				return
			}
		}
	}
}

// Values returns an iterator over the values of the OrderedMap in insertion order.
//
// It is safe to delete keys from the OrderedMap during iteration.
func (m *OrderedMap[K, V]) Values() iter.Seq[V] {
	return func(yield func(V) bool) {
		for _, key := range slices.Clone(m.order) {
			val, ok := m.data[key]
			if !ok {
				continue
			}
			if !yield(val) {
				return
			}
		}
	}
}

// Items returns an iterator over the key-value pairs of the OrderedMap in insertion order.
//
// It is safe to delete keys from the OrderedMap during iteration.
func (m *OrderedMap[K, V]) Items() iter.Seq2[K, V] {
	return func(yield func(K, V) bool) {
		for _, key := range slices.Clone(m.order) {
			val, ok := m.data[key]
			if !ok {
				continue
			}
			if !yield(key, val) {
				return
			}
		}
	}
}
