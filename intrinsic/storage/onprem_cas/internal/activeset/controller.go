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
	"context"
	"fmt"
	"log/slog"
	"maps"
	"slices"
	"sync"

	"intrinsic/storage/onprem_cas/internal/transfer"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

// Transfer abstracts the transfer operations (download, upload, stat).
type Transfer interface {
	Download(ctx context.Context, d Digest) (<-chan transfer.DownloadResult, error)
	Upload(ctx context.Context, d Digest) (<-chan transfer.UploadResult, error)
	Stat(ctx context.Context, d Digest) (<-chan transfer.StatResult, error)
}

type trackedObject struct {
	status *ObjectStatus
	// reconciling indicates whether an asynchronous transfer operation (stat, download, or upload)
	// is currently in flight for this object. When true, it prevents duplicate concurrent transfer
	// requests and protects the object from being evicted from memory while workers are active.
	reconciling bool
	// refs tracks all keys ("name/generation") referencing this object across any operation.
	refs map[RefKey]struct{}
	// downloadRefs tracks specific keys actively requesting local download of this object.
	downloadRefs map[RefKey]struct{}
	// uploadRefs tracks specific keys actively requesting upstream upload of this object.
	uploadRefs map[RefKey]struct{}
}

// OnChangeFunc is called by Controller whenever a single object status changes for a key.
// Callbacks are invoked synchronously on the controller's internal coordinator goroutine
// (loop) and must not block or call back into synchronous Controller methods (Add, Remove,
// Clear, TrackingSizes) to avoid deadlocks.
type OnChangeFunc func(key RefKey, status *ObjectStatus)

// Controller manages the active set of object digests requiring download and upload across keys.
type Controller struct {
	transfer Transfer
	onChange OnChangeFunc

	// event channels
	requestsChan chan request
	resultsChan  chan any

	// objects maps unique digests to their internal transfer state, deduplicating work across keys.
	objects map[Digest]*trackedObject
	// keyObjects maps a key to all object digests belonging to it, enabling fast per-key lookup and cleanup.
	keyObjects map[RefKey]map[Digest]struct{}
	// background context / go-outine management
	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup
}

// NewController creates a new Controller and launches its coordinator loop.
func NewController(ctx context.Context, tr Transfer, onChange OnChangeFunc) *Controller {
	ctx, cancel := context.WithCancel(ctx)
	c := &Controller{
		ctx:          ctx,
		cancel:       cancel,
		transfer:     tr,
		onChange:     onChange,
		requestsChan: make(chan request, 100),
		resultsChan:  make(chan any, 100),
		objects:      make(map[Digest]*trackedObject),
		keyObjects:   make(map[RefKey]map[Digest]struct{}),
	}
	c.wg.Go(c.loop)
	return c
}

// Close cancels the context and waits for background operations to exit.
func (c *Controller) Close() {
	c.cancel()
	c.wg.Wait()
}

// sendRequest synchronously enqueues a request to the coordinator loop and blocks until processed or canceled.
func (c *Controller) sendRequest(req request) {
	req.initDone(make(chan struct{}))
	select {
	case <-c.ctx.Done():
		return
	case c.requestsChan <- req:
		select {
		case <-c.ctx.Done():
		case <-req.done():
		}
	}
}

// Add registers download and upload digest sets for a specific key.
func (c *Controller) Add(k RefKey, dl []Digest, up []Digest) {
	if len(k) == 0 || (len(dl) == 0 && len(up) == 0) {
		return
	}
	c.sendRequest(&addRequest{key: k, dls: dl, ups: up})
}

// Remove removes download and upload digest sets for a specific key.
func (c *Controller) Remove(k RefKey, dl []Digest, up []Digest) {
	if len(k) == 0 || (len(dl) == 0 && len(up) == 0) {
		return
	}
	c.sendRequest(&removeRequest{key: k, dls: dl, ups: up})
}

// Clear removes all tracking for a specific key and evicts unreferenced objects.
func (c *Controller) Clear(k RefKey) {
	if len(k) == 0 {
		return
	}
	c.sendRequest(&clearRequest{key: k})
}

// TrackingSizes returns the number of tracked objects and keys.
func (c *Controller) TrackingSizes() (int, int) {
	sizesCh := make(chan [2]int, 1)
	c.sendRequest(&getSizesRequest{sizesCh: sizesCh})
	select {
	case res := <-sizesCh:
		return res[0], res[1]
	default:
		return 0, 0
	}
}

// loop runs the single-goroutine coordinator event loop, serializing all map access and state changes.
func (c *Controller) loop() {
	for {
		select {
		case req := <-c.requestsChan:
			c.handleRequest(req)
		case res := <-c.resultsChan:
			c.handleResult(res)
		case <-c.ctx.Done():
			// Drain remaining pending synchronous requests during shutdown so callers do not hang.
			for {
				select {
				case req := <-c.requestsChan:
					req.closeDone()
				default:
					return
				}
			}
		}
	}
}

// getOrCreateObject retrieves an existing tracking object for a digest or initializes a new one.
func (c *Controller) getOrCreateObject(d Digest) *trackedObject {
	obj := c.objects[d]
	if obj == nil {
		obj = &trackedObject{
			status:       &ObjectStatus{Digest: d},
			refs:         make(map[RefKey]struct{}),
			downloadRefs: make(map[RefKey]struct{}),
			uploadRefs:   make(map[RefKey]struct{}),
		}
		c.objects[d] = obj
	}
	return obj
}

// triggerReconcile checks if an object requires transfer operations and initiates a stat check if not already reconciling.
func (c *Controller) triggerReconcile(d Digest) {
	obj := c.objects[d]
	if obj == nil || obj.reconciling {
		return
	}
	needDl := len(obj.downloadRefs) > 0
	needUp := len(obj.uploadRefs) > 0
	if !needDl && !needUp {
		// No need for reconciling if no-one requested that object to up or downloaded
		return
	}
	// Handle the cases where the object is already available
	if (!needDl || obj.status.Local.GetOr(false)) && (!needUp || obj.status.Upstream.GetOr(false)) {
		c.handleSuccess(d)
		return
	}
	obj.status.ErrorMsg = Optional[string]{}
	obj.status.ErrorCode = Optional[codes.Code]{}
	// Mark reconciling so we avoid launching duplicate stat/transfer requests concurrently.
	obj.reconciling = true
	// Perform a Stat operation. Afterwards [handleStatResult] is called to determine further reconcilation steps.
	c.triggerStat(d)
}

// triggerStat initiates an asynchronous stat check to determine local and upstream object availability.
func (c *Controller) triggerStat(d Digest) {
	if c.ctx.Err() != nil {
		return
	}
	statCh, err := c.transfer.Stat(c.ctx, d)
	c.wg.Go(func() {
		res, err := wait(c.ctx, statCh, err)
		select {
		case <-c.ctx.Done():
		case c.resultsChan <- statResult{d: d, statRes: res, err: err}:
		}
	})
}

// triggerDownload initiates an asynchronous download operation for a digest.
func (c *Controller) triggerDownload(d Digest) {
	if c.ctx.Err() != nil {
		return
	}
	dlCh, err := c.transfer.Download(c.ctx, d)
	c.wg.Go(func() {
		_, err := wait(c.ctx, dlCh, err)
		select {
		case <-c.ctx.Done():
		case c.resultsChan <- downloadResult{d: d, err: err}:
		}
	})
}

// triggerUpload initiates an asynchronous upload operation for a digest.
func (c *Controller) triggerUpload(d Digest) {
	if c.ctx.Err() != nil {
		return
	}
	upCh, err := c.transfer.Upload(c.ctx, d)
	c.wg.Go(func() {
		_, err := wait(c.ctx, upCh, err)
		select {
		case <-c.ctx.Done():
		case c.resultsChan <- uploadResult{d: d, err: err}:
		}
	})
}

// notifyChange passes a copy of an object's current status to the subscriber for a key.
func (c *Controller) notifyChange(key RefKey, obj *trackedObject) {
	if c.onChange == nil || obj == nil {
		return
	}
	c.onChange(key, obj.status.Copy())
}

// notifyChangeAll collects all unique keys currently referencing obj and notifies them of its status.
func (c *Controller) notifyChangeAll(obj *trackedObject) {
	if c.onChange == nil || obj == nil {
		return
	}
	// Notify all unique keys across download and uploads
	keyMap := make(map[RefKey]struct{}, len(obj.downloadRefs)+len(obj.uploadRefs))
	maps.Copy(keyMap, obj.downloadRefs)
	maps.Copy(keyMap, obj.uploadRefs)
	for _, k := range slices.Sorted(maps.Keys(keyMap)) {
		c.notifyChange(k, obj)
	}
}

// handleRequest processes synchronous tracking requests inside the serialized coordinator loop.
// It updates reference maps, initiates transfer reconciliation for new objects, and unblocks waiting callers.
func (c *Controller) handleRequest(req request) {
	switch r := req.(type) {
	// Add requests register new download and upload responsibilities for a key, trigger reconciliation, and notify subscribers.
	case *addRequest:
		if c.keyObjects[r.key] == nil {
			c.keyObjects[r.key] = make(map[Digest]struct{})
		}
		// Register objects to download
		for _, d := range r.dls {
			c.keyObjects[r.key][d] = struct{}{}
			obj := c.getOrCreateObject(d)
			obj.refs[r.key] = struct{}{}
			obj.downloadRefs[r.key] = struct{}{}
			c.triggerReconcile(d)
			c.notifyChange(r.key, obj)
		}
		// Register objects to upload
		for _, d := range r.ups {
			c.keyObjects[r.key][d] = struct{}{}
			obj := c.getOrCreateObject(d)
			obj.refs[r.key] = struct{}{}
			obj.uploadRefs[r.key] = struct{}{}
			c.triggerReconcile(d)
			c.notifyChange(r.key, obj)
		}
	// Clear requests completely deregister a key, removing all its object references and evicting idle objects from memory.
	case *clearRequest:
		objects := c.keyObjects[r.key]
		delete(c.keyObjects, r.key)
		for d := range objects {
			if obj := c.objects[d]; obj != nil {
				delete(obj.refs, r.key)
				delete(obj.downloadRefs, r.key)
				delete(obj.uploadRefs, r.key)
				if len(obj.refs) == 0 && !obj.reconciling {
					delete(c.objects, d)
				}
			}
		}
	// Remove requests revoke specific download or upload responsibilities for a key (e.g., cancelling downloads when superseded).
	case *removeRequest:
		for _, d := range r.dls {
			if obj := c.objects[d]; obj != nil {
				delete(obj.downloadRefs, r.key)
				// if the object is not referenced anymore, remove it from all maps
				_, dl := obj.downloadRefs[r.key]
				_, up := obj.uploadRefs[r.key]
				if !dl && !up {
					delete(obj.refs, r.key)
					delete(c.keyObjects[r.key], d)
					if len(obj.refs) == 0 && !obj.reconciling {
						delete(c.objects, d)
					}
				}
				c.notifyChange(r.key, obj)
			}
		}
		for _, d := range r.ups {
			if obj := c.objects[d]; obj != nil {
				delete(obj.uploadRefs, r.key)
				// if the object is not referenced anymore, remove it from all maps
				_, dl := obj.downloadRefs[r.key]
				_, up := obj.uploadRefs[r.key]
				if !dl && !up {
					delete(obj.refs, r.key)
					delete(c.keyObjects[r.key], d)
					if len(obj.refs) == 0 && !obj.reconciling {
						delete(c.objects, d)
					}
				}
				c.notifyChange(r.key, obj)
			}
		}
	// GetSizes requests report the total counts of currently tracked object digests and keys.
	case *getSizesRequest:
		if r.sizesCh != nil {
			r.sizesCh <- [2]int{len(c.objects), len(c.keyObjects)}
		}
	}
	req.closeDone()
}

// handleResult processes asynchronous completion of stat, download, or upload worker goroutines.
func (c *Controller) handleResult(res any) {
	switch r := res.(type) {
	// Stat results update local and upstream availability and trigger download or upload transfers if needed.
	case statResult:
		c.handleStatResult(r)
	// Download results mark the object locally available upon success or record error metadata upon failure.
	case downloadResult:
		obj := c.objects[r.d]
		if obj == nil {
			return
		}
		if r.err != nil {
			c.handleError(r.d, "download", r.err)
			return
		}
		obj.status.Local = NewOptional(true)
		c.handleSuccess(r.d)
	// Upload results mark the object available upstream upon success or record error metadata upon failure.
	case uploadResult:
		obj := c.objects[r.d]
		if obj == nil {
			return
		}
		if r.err != nil {
			c.handleError(r.d, "upload", r.err)
			return
		}
		obj.status.Upstream = NewOptional(true)
		c.handleSuccess(r.d)
	}
}

// handleStatResult updates object status from stat results and initiates any required download or upload operations.
func (c *Controller) handleStatResult(r statResult) {
	obj := c.objects[r.d]
	if obj == nil {
		return
	}
	if r.err != nil {
		c.handleError(r.d, "stat", r.err)
		return
	}
	// reset a potential previous error state
	obj.status.ErrorMsg = Optional[string]{}
	obj.status.ErrorCode = Optional[codes.Code]{}
	// Record discovered availability and size metadata onto the tracked object status.
	obj.status.Local = NewOptional(r.statRes.Locally)
	obj.status.Upstream = NewOptional(r.statRes.Upstream)
	obj.status.Size = NewOptional(r.statRes.Size)

	// Notify all referencing keys immediately so they observe updated availability and size metadata.
	c.notifyChangeAll(obj)

	locally := r.statRes.Locally
	upstream := r.statRes.Upstream
	needDl := len(obj.downloadRefs) > 0
	needUp := len(obj.uploadRefs) > 0

	// Decide next transfer action based on availability and subscriber requirements:

	// 1. Available both locally and upstream: no transfer needed, so mark completed immediately.
	if locally && upstream {
		c.handleSuccess(r.d)
		return
	}
	// 2. Missing everywhere: transfer cannot proceed, so record a NotFound error.
	if !locally && !upstream {
		c.handleError(r.d, "stat", status.Error(codes.NotFound, "missing everywhere"))
		return
	}
	// 3. Available upstream only: trigger local download if requested; otherwise mark completed.
	if !locally && upstream {
		if needDl {
			slog.Debug("ActiveSet: Enqueueing download", slog.String("digest", r.d))
			c.triggerDownload(r.d)
		} else {
			c.handleSuccess(r.d)
		}
		return
	}
	// 4. Available locally only: trigger upstream upload if requested; otherwise mark completed.
	if locally && !upstream {
		if needUp {
			slog.Debug("ActiveSet: Enqueueing upload", slog.String("digest", r.d))
			c.triggerUpload(r.d)
		} else {
			c.handleSuccess(r.d)
		}
		return
	}
}

// handleSuccess marks reconciliation finished, clears active download/upload references, and notifies subscribers.
func (c *Controller) handleSuccess(d Digest) {
	obj := c.objects[d]
	if obj == nil {
		return
	}
	// clear a potential previous error state
	obj.status.ErrorMsg = Optional[string]{}
	obj.status.ErrorCode = Optional[codes.Code]{}
	c.notifyChangeAll(obj)
	for k := range obj.downloadRefs {
		delete(obj.downloadRefs, k)
	}
	for k := range obj.uploadRefs {
		delete(obj.uploadRefs, k)
	}
	obj.reconciling = false
	// If no keys reference this object anymore, evict it from memory.
	if len(obj.refs) == 0 {
		delete(c.objects, d)
	}
}

// grpcErrorCode extracts a gRPC status code if err is a gRPC error, returning empty Optional otherwise.
func grpcErrorCode(err error) Optional[codes.Code] {
	if err == nil {
		return Optional[codes.Code]{}
	}
	if s, ok := status.FromError(err); ok {
		return NewOptional(s.Code())
	}
	return Optional[codes.Code]{}
}

// handleError records transfer error metadata, clears pending active references, and notifies affected keys.
func (c *Controller) handleError(d Digest, op string, err error) {
	obj := c.objects[d]
	if obj == nil {
		return
	}
	slog.Warn("ActiveSet: Transfer failed", slog.String("op", op), slog.String("digest", d), slog.Any("error", err))
	errorCode := grpcErrorCode(err)
	obj.status.ErrorMsg = NewOptional(err.Error())
	obj.status.ErrorCode = errorCode

	c.notifyChangeAll(obj)
	for k := range obj.downloadRefs {
		delete(obj.downloadRefs, k)
	}
	for k := range obj.uploadRefs {
		delete(obj.uploadRefs, k)
	}
	obj.reconciling = false
	// Evict object if no keys reference it anymore.
	if len(obj.refs) == 0 {
		delete(c.objects, d)
	}
}

// wait awaits the first result from a transfer channel or context cancellation, wrapping any immediate enqueue errors.
func wait[T transfer.Result](ctx context.Context, ch <-chan T, enqueueErr error) (T, error) {
	var zero T
	if enqueueErr != nil {
		return zero, fmt.Errorf("enqueuing task for transfer failed: %w", enqueueErr)
	}
	select {
	case <-ctx.Done():
		return zero, ctx.Err()
	case res, ok := <-ch:
		if !ok {
			return zero, fmt.Errorf("transfer channel closed without result")
		}
		if err := res.GetErr(); err != nil {
			return res, err
		}
		return res, nil
	}
}
