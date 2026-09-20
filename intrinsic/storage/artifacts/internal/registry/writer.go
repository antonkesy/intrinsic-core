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

package registry

import (
	"context"
	"fmt"
	"io"
	"time"

	log "github.com/golang/glog"
)

// writeResult carries the outcome of the background pipe write
type writeResult struct {
	n   int
	err error
}

// TimeoutWriter is Worker pattern guard for io.Pipe protecting writers
// from unresponsive readers, allowing writer to give up on write after
// certain timeout was hit. This allows passing io.Pipe to libraries and
// external consumers more safely as it allows callers to deal with
// misbehaving readers.
type TimeoutWriter struct {
	pw     *io.PipeWriter
	ctx    context.Context
	cancel context.CancelFunc

	// Coordination
	dataReq    chan []byte
	resultChan chan writeResult
}

func NewTimeoutWriter(ctx context.Context) (*io.PipeReader, *TimeoutWriter) {
	pr, pw := io.Pipe()
	ctx, cancel := context.WithCancel(ctx)

	tw := &TimeoutWriter{
		pw:         pw,
		ctx:        ctx,
		cancel:     cancel,
		dataReq:    make(chan []byte),
		resultChan: make(chan writeResult),
	}

	// Start the single long-running worker
	go tw.pipeWorker()

	return pr, tw
}

// pipeWorker is the only goroutine that ever touches pw.Write
func (t *TimeoutWriter) pipeWorker() {
	defer t.pw.Close()

	for {
		select {
		case <-t.ctx.Done():
			return
		case p, ok := <-t.dataReq:
			if !ok {
				return
			}
			// This is critical write to the pipe. If reader stops reading
			// we will hang here forever
			n, err := t.pw.Write(p)

			// Send result back. If the Write call took too long,
			// the main Write() loop might have already moved on,
			// so we use a select to avoid blocking the worker forever.
			select {
			case t.resultChan <- writeResult{n, err}:
			case <-t.ctx.Done():
				return
			}
		}
	}
}

// Write to comply with io.Writer interface. Prefer using WriteContext
// to reuse your local context. The timeout is set to 15 seconds.
func (t *TimeoutWriter) Write(p []byte) (int, error) {
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()
	return t.WriteContext(ctx, p)
}

// WriteContext writes into pipe with write timeout protection controlled
// by ctx object passed as parameter. If ctx is cancelled, such as timeout was reached,
// WriteContext returns error and closes pipe to prevent further writes to get stuck.
// This provides protection agains sneaky readers who just stop reading.
func (t *TimeoutWriter) WriteContext(ctx context.Context, p []byte) (int, error) {
	// Are we still open?
	if t.ctx.Err() != nil {
		return 0, fmt.Errorf("writing on finished writer: %w", t.ctx.Err())
	}

	// Write data to the worker, this should succeed.
	select {
	case t.dataReq <- p:
	case <-t.ctx.Done():
		log.Infof("writer context cancelled in pipe write: %s", t.ctx.Err())
		return 0, fmt.Errorf("writer closed while attempting to write to pipe: %w", t.ctx.Err())
	}

	// Wait for result or time-out
	select {
	case res := <-t.resultChan:
		return res.n, res.err
	case <-ctx.Done(): // we exceeded callers write limit timeout
		log.Warning("write to pipe timed out, no read before context canceled, data loss")
		err := fmt.Errorf("write to pipe timed out. reader inactive: %w", ctx.Err())
		// Crucial: We must kill the pipe to unblock the worker
		// currently stuck inside t.pw.Write(p)
		t.pw.CloseWithError(err)
		t.cancel() // cancels global context preventing further writes
		return 0, err
	case <-t.ctx.Done(): // writer's context was canceled from outside
		return 0, fmt.Errorf("writer finished while waiting for reader: %w", t.ctx.Err())
	}
}

func (t *TimeoutWriter) Close() error {
	log.Info("closing writer")
	if t.ctx.Err() != nil {
		// nothing to do here.
		return nil
	}
	close(t.dataReq)
	t.cancel()
	return nil
}
