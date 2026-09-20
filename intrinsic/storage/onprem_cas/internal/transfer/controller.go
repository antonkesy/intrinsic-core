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

package transfer

import (
	"context"
	"fmt"
	"time"

	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"

	"log/slog"
)

// Config defines the configuration for the Controller.
type Timeout struct {
	Max         time.Duration
	MinSpeedBps uint64
	Min         time.Duration
}

// Effective calculates the timeout to use for a given object size.
// If size is 0, it returns the Max timeout.
func (to Timeout) Effective(size uint64) time.Duration {
	if size == 0 {
		return to.Max
	}
	timeoutSec := size / to.MinSpeedBps
	maxSec := uint64(to.Max / time.Second)
	if timeoutSec >= maxSec {
		return to.Max
	}
	dynamicTimeout := time.Duration(timeoutSec) * time.Second
	return max(to.Min, dynamicTimeout)
}

// Config defines the configuration for the Controller.
type Config struct {
	DownloadWorkers int
	UploadWorkers   int
	StatWorkers     int
	DownloadTimeout Timeout
	UploadTimeout   Timeout
	StatCacheSize   int
	StatCacheDir    string
}

// Validate adjusts invalid values (<= 0) to their defaults.
func (c *Config) Validate() {
	if c.DownloadWorkers <= 0 {
		c.DownloadWorkers = defaultDownloadWorkers
	}
	if c.UploadWorkers <= 0 {
		c.UploadWorkers = defaultUploadWorkers
	}
	if c.StatWorkers <= 0 {
		c.StatWorkers = defaultStatWorkers
	}
	if c.DownloadTimeout.Max <= 0 {
		c.DownloadTimeout.Max = defaultMaxDownloadDuration
	}
	if c.UploadTimeout.Max <= 0 {
		c.UploadTimeout.Max = defaultMaxUploadDuration
	}
	if c.DownloadTimeout.Min <= 0 {
		c.DownloadTimeout.Min = defaultMinTimeout
	}
	if c.UploadTimeout.Min <= 0 {
		c.UploadTimeout.Min = defaultMinTimeout
	}
	if c.DownloadTimeout.MinSpeedBps <= 0 {
		c.DownloadTimeout.MinSpeedBps = defaultMinSpeedBps
	}
	if c.UploadTimeout.MinSpeedBps <= 0 {
		c.UploadTimeout.MinSpeedBps = defaultMinSpeedBps
	}
	if c.StatCacheSize <= 0 {
		c.StatCacheSize = defaultStatCacheSize
	}
}

// DefaultConfig returns a Config with default values.
func DefaultConfig() Config {
	return Config{
		DownloadWorkers: defaultDownloadWorkers,
		UploadWorkers:   defaultUploadWorkers,
		StatWorkers:     defaultStatWorkers,
		DownloadTimeout: Timeout{
			Max:         defaultMaxDownloadDuration,
			MinSpeedBps: defaultMinSpeedBps,
			Min:         defaultMinTimeout,
		},
		UploadTimeout: Timeout{
			Max:         defaultMaxUploadDuration,
			MinSpeedBps: defaultMinSpeedBps,
			Min:         defaultMinTimeout,
		},
		StatCacheSize: defaultStatCacheSize,
		StatCacheDir:  "",
	}
}

const (
	defaultDownloadWorkers     = 4
	defaultUploadWorkers       = 2
	defaultStatWorkers         = 4
	defaultMaxDownloadDuration = 5 * time.Hour
	defaultMaxUploadDuration   = 10 * time.Minute
	defaultMinTimeout          = 10 * time.Second
	defaultMinSpeedBps         = 1
	defaultStatCacheSize       = 1000
)

// Controller orchestrates download, upload, and stat operations using de-duplicating queues.
type Controller struct {
	dl *Downloader
	ul *Uploader
	ds *dualStatter

	downloadQueue *queue[*DownloadResult]
	uploadQueue   *queue[*UploadResult]
	statQueue     *queue[*StatResult]
}

// NewController creates a new Controller, instantiating all queues.
func NewController(
	ctx context.Context,
	upstreamClient caspb.ContentAddressableStorageServiceClient,
	localClient caspb.ContentAddressableStorageServiceClient,
	cfg *Config,
) (*Controller, error) {
	var finalCfg Config
	if cfg == nil {
		finalCfg = DefaultConfig()
	} else {
		finalCfg = *cfg
		finalCfg.Validate()
	}

	ds, err := newDualStatter(upstreamClient, localClient, finalCfg.StatCacheSize, finalCfg.StatCacheDir)
	if err != nil {
		return nil, fmt.Errorf("failed to create dualStatter: %w", err)
	}
	dl := NewDownloader(upstreamClient, localClient, ds, finalCfg.DownloadTimeout)
	ul := NewUploader(upstreamClient, localClient, ds, finalCfg.UploadTimeout)

	c := &Controller{
		dl: dl,
		ul: ul,
		ds: ds,
	}

	c.downloadQueue = newQueue(ctx, finalCfg.DownloadWorkers, func(qCtx context.Context, digest Digest) (*DownloadResult, error) {
		return c.dl.Download(qCtx, digest)
	})

	c.uploadQueue = newQueue(ctx, finalCfg.UploadWorkers, func(qCtx context.Context, digest Digest) (*UploadResult, error) {
		return c.ul.Upload(qCtx, digest)
	})

	c.statQueue = newQueue(ctx, finalCfg.StatWorkers, func(qCtx context.Context, digest Digest) (*StatResult, error) {
		return c.ds.stat(qCtx, digest)
	})

	return c, nil
}

// Close stops all queues and waits for running workers.
func (c *Controller) Close() {
	c.downloadQueue.Close()
	c.uploadQueue.Close()
	c.statQueue.Close()
}

// Download enqueues a download job and returns a channel that will receive the result.
// If it returns (nil, err), the async operation was not started.
// If it returns (ch, nil), the operation was started. If the operation fails later,
// the returned DownloadResult will have its Err field set.
func (c *Controller) Download(ctx context.Context, digest string) (<-chan DownloadResult, error) {
	slog.Debug("Controller: Enqueueing download", slog.String("digest", digest))

	// 1. Enqueue the job to the download queue.
	jCh, err := c.downloadQueue.Enqueue(ctx, digest)
	if err != nil {
		return nil, err // Async operation failed to start (e.g. queue closed/context canceled)
	}

	// 2. Create a buffered channel (size 1) to return to the caller.
	// Buffer size 1 prevents the bridge goroutine from blocking if the caller stops listening.
	ch := make(chan DownloadResult, 1)

	// 3. Spawn a background goroutine to bridge the queue's channel to the return channel.
	go func() {
		defer close(ch) // Always close the channel when the operation is finished

		select {
		case <-ctx.Done():
			// Context canceled before we got the result
			ch <- DownloadResult{Digest: digest, Err: ctx.Err()}
		case job := <-jCh:
			if job.Err != nil {
				// Job failed; return error result
				ch <- DownloadResult{Digest: digest, Err: job.Err}
				return
			}
			if job.Val != nil {
				// Job succeeded; forward the result (ensuring Err is nil)
				r := *job.Val
				r.Err = nil
				ch <- r
				return
			}
			ch <- DownloadResult{Digest: digest, Err: fmt.Errorf("internal error: download finished with no result")}
		}
	}()
	return ch, nil
}

// Upload enqueues an upload job and returns a channel that will receive the result.
// If it returns (nil, err), the async operation was not started.
// If it returns (ch, nil), the operation was started. If the operation fails later,
// the returned UploadResult will have its Err field set.
func (c *Controller) Upload(ctx context.Context, digest string) (<-chan UploadResult, error) {
	slog.Debug("Controller: Enqueueing upload", slog.String("digest", digest))

	// 1. Enqueue the job to the upload queue.
	jCh, err := c.uploadQueue.Enqueue(ctx, digest)
	if err != nil {
		return nil, err // Async operation failed to start
	}

	// 2. Create a buffered channel (size 1) to return to the caller.
	ch := make(chan UploadResult, 1)

	// 3. Spawn a background goroutine to bridge the queue's channel to the return channel.
	go func() {
		defer close(ch)

		select {
		case <-ctx.Done():
			ch <- UploadResult{Digest: digest, Err: ctx.Err()}
		case job := <-jCh:
			if job.Err != nil {
				ch <- UploadResult{Digest: digest, Err: job.Err}
				return
			}
			if job.Val != nil {
				r := *job.Val
				r.Err = nil
				ch <- r
				return
			}
			ch <- UploadResult{Digest: digest, Err: fmt.Errorf("internal error: upload finished with no result")}
		}
	}()
	return ch, nil
}

// Stat enqueues a stat query job and returns a channel that will receive the result.
// If it returns (nil, err), the async operation was not started.
// If it returns (ch, nil), the operation was started. If the operation fails later,
// the returned StatResult will have its Err field set.
func (c *Controller) Stat(ctx context.Context, digest string) (<-chan StatResult, error) {
	slog.Debug("Controller: Enqueueing stat", slog.String("digest", digest))

	// 1. Enqueue the job to the stat queue.
	jCh, err := c.statQueue.Enqueue(ctx, digest)
	if err != nil {
		return nil, err // Async operation failed to start
	}

	// 2. Create a buffered channel (size 1) to return to the caller.
	ch := make(chan StatResult, 1)

	// 3. Spawn a background goroutine to bridge the queue's channel to the return channel.
	go func() {
		defer close(ch)

		select {
		case <-ctx.Done():
			ch <- StatResult{Err: ctx.Err()}
		case job := <-jCh:
			if job.Err != nil {
				ch <- StatResult{Err: job.Err}
				return
			}
			if job.Val != nil {
				r := *job.Val
				r.Err = nil
				ch <- r
				return
			}
			ch <- StatResult{Err: fmt.Errorf("internal error: stat finished with no result")}
		}
	}()
	return ch, nil
}

// Result is an interface for transfer results (DownloadResult, UploadResult, StatResult).
type Result interface {
	GetErr() error
}

// GetErr returns the error associated with the DownloadResult.
func (r DownloadResult) GetErr() error { return r.Err }

// GetErr returns the error associated with the UploadResult.
func (r UploadResult) GetErr() error { return r.Err }

// GetErr returns the error associated with the StatResult.
func (r StatResult) GetErr() error { return r.Err }
