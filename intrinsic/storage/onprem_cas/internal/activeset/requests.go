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

package activeset

import (
	"intrinsic/storage/onprem_cas/internal/transfer"
)

// request defines the internal interface for coordinator loop operations,
// providing lifecycle management via completion notification channels.
type request interface {
	closeDone()
	done() <-chan struct{}
	initDone(chan struct{})
}

// baseRequest implements common channel initialization and completion signaling for requests.
type baseRequest struct {
	doneCh chan struct{}
}

// initDone initializes the completion signaling channel.
func (r *baseRequest) initDone(ch chan struct{}) {
	r.doneCh = ch
}

// closeDone safely closes the completion signaling channel without blocking or panicking on double closure.
func (r *baseRequest) closeDone() {
	if r.doneCh != nil {
		select {
		case <-r.doneCh:
		default:
			close(r.doneCh)
		}
	}
}

// done returns a read-only channel that is closed when the request completes.
func (r *baseRequest) done() <-chan struct{} {
	return r.doneCh
}

// addRequest registers sets of object digests to be downloaded and uploaded for a specific key.
type addRequest struct {
	baseRequest
	key RefKey
	dls []Digest
	ups []Digest
}

// removeRequest removes sets of object digests from being downloaded or uploaded for a specific key.
type removeRequest struct {
	baseRequest
	key RefKey
	dls []Digest
	ups []Digest
}

// clearRequest removes all tracking for a specific key and evicts unreferenced objects.
type clearRequest struct {
	baseRequest
	key RefKey
}

// getSizesRequest queries the number of currently tracked objects and keys.
type getSizesRequest struct {
	baseRequest
	sizesCh chan<- [2]int
}

// statResult holds the outcome of an asynchronous stat check for a digest.
type statResult struct {
	d       Digest
	statRes transfer.StatResult
	err     error
}

// downloadResult holds the outcome of an asynchronous download operation for a digest.
type downloadResult struct {
	d   Digest
	err error
}

// uploadResult holds the outcome of an asynchronous upload operation for a digest.
type uploadResult struct {
	d   Digest
	err error
}
