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

// Package configmapwatcher contains utilities for processing k8s configmap updates.
package configmapwatcher

import (
	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/client-go/informers"
	"k8s.io/client-go/tools/cache"
)

type options struct {
	// Function called everytime a ConfigMap is added or changed.
	updaterFn func(*corev1.ConfigMap)
	// Function called everytime a ConfigMap is deleted.
	deleterFn func(*corev1.ConfigMap)
	// Optional namespace filter.
	namespace string
	// Optional k8s label selection filter.
	labelSelector string
	// Optional name filter.
	name string
}

// Option configures the ConfigMapWatcher.
type Option func(*options)

// WithNamespace configures the watcher to only track changes in this namespace.
func WithNamespace(namespace string) Option {
	return func(opts *options) {
		opts.namespace = namespace
	}
}

// WithName configures the watcher to only track changes in the give configmap.
func WithName(name string) Option {
	return func(opts *options) {
		opts.name = name
	}
}

// WithLabelSelector configures the watcher to apply the given label selector.
func WithLabelSelector(labelSelector string) Option {
	return func(opts *options) {
		opts.labelSelector = labelSelector
	}
}

// WithUpdaterFn configures the watcher to call this function for every ConfigMap add or update event.
func WithUpdaterFn(updaterFn func(*corev1.ConfigMap)) Option {
	return func(opts *options) {
		opts.updaterFn = updaterFn
	}
}

// WithDeleterFn configures the watcher to call this function everytime a ConfigMap is deleted.
func WithDeleterFn(deleterFn func(*corev1.ConfigMap)) Option {
	return func(opts *options) {
		opts.deleterFn = deleterFn
	}
}

type cmFilter struct {
	namespace string
	name      string
	selector  labels.Selector
}

func (f *cmFilter) Match(cm *corev1.ConfigMap) bool {
	if f.namespace != "" && cm.Namespace != f.namespace {
		return false
	}
	if f.name != "" && cm.Name != f.name {
		return false
	}
	if f.selector != nil && !f.selector.Matches(labels.Set(cm.Labels)) {
		return false
	}
	return true
}

func (opts options) clientFilter() *cmFilter {
	f := &cmFilter{
		namespace: opts.namespace,
		name:      opts.name,
	}
	if opts.labelSelector != "" {
		selector, err := labels.Parse(opts.labelSelector)
		if err == nil {
			f.selector = selector
		}
	}
	return f
}

// StartConfigMapWatcher starts a watcher on ConfigMap resources with the given options.
func StartConfigMapWatcher(factory informers.SharedInformerFactory, opts ...Option) {
	var options options
	for _, opt := range opts {
		opt(&options)
	}

	informer := factory.Core().V1().ConfigMaps().Informer()

	filter := options.clientFilter()

	handlers := cache.ResourceEventHandlerFuncs{}
	if options.updaterFn != nil {
		handlers.AddFunc = func(obj any) {
			cm := obj.(*corev1.ConfigMap)
			if !filter.Match(cm) {
				return
			}
			options.updaterFn(cm)
		}
		handlers.UpdateFunc = func(oldObj any, newObj any) {
			cm := newObj.(*corev1.ConfigMap)
			if !filter.Match(cm) {
				return
			}
			options.updaterFn(cm)
		}
	}
	if options.deleterFn != nil {
		handlers.DeleteFunc = func(obj any) {
			cm := obj.(*corev1.ConfigMap)
			if !filter.Match(cm) {
				return
			}
			options.deleterFn(cm)
		}
	}
	informer.AddEventHandler(handlers)
}
