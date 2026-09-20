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
	"sync"
)

// enqueueResult wraps the response returned by an enqueue request after
// it has been processed by a worker.
type enqueueResult[Result any] struct {
	Val Result
	Err error
}

// workerJob is an work item sent to a worker.
type workerJob[Result any] struct {
	d  Digest
	ch chan enqueueResult[Result]
}

// workerResult is returned by the worker.
type workerResult[Result any] struct {
	d   Digest
	res Result
	err error
}

// queue manages a queue of requests, deduplicates identical simultaneous requests,
// and distributes work to a static pool of pre-created background workers.
type queue[Result any] struct {
	// maxWorkers limits the parallism of workFun
	maxWorkers int
	// workFn is executed by the workers (Download, Upload, Stat)
	workFn func(ctx context.Context, req Digest) (Result, error)
	// backlogChan contains the enqueued requests by the controller
	backlogChan chan *workerJob[Result]

	// toWorkersChan is used to send requests to the static workers
	toWorkersChan chan *workerJob[Result]
	// fromWorkersChan is the result returned from executing workFn
	fromWorkersChan chan workerResult[Result]

	// manage the background context for the workers
	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup
}

// maxRequestBacklog is the size of the backlog of requests if all
// workers are busy. This is the upper limit of unique CAS digests
// which can be requested at once.
const maxRequestBacklog = 1000

// newQueue creates a new queue, spawns the static worker pool, and starts the coordinator loop.
// The given context must be valid during the intended lifetime of the queue.
func newQueue[Result any](
	ctx context.Context,
	maxWorkers int,
	workFn func(context.Context, Digest) (Result, error),
) *queue[Result] {
	ctx, cancel := context.WithCancel(ctx)
	if maxWorkers <= 0 {
		maxWorkers = 1
	}
	// configure the queue
	q := &queue[Result]{
		backlogChan:     make(chan *workerJob[Result], maxRequestBacklog),
		fromWorkersChan: make(chan workerResult[Result], maxWorkers),
		toWorkersChan:   make(chan *workerJob[Result]),
		ctx:             ctx,
		cancel:          cancel,
		maxWorkers:      maxWorkers,
		workFn:          workFn,
	}
	// pre-create the static worker pool using WaitGroup.Go
	for range maxWorkers {
		q.wg.Go(q.worker)
	}
	// start the coordinator loop
	q.wg.Go(q.loop)
	return q
}

// Enqueue adds a request to the queue and returns a channel that will receive the result.
// Even if the caller cancels the context, the job continue to be processed in the background.
// There is currently no support for detecting if all requests for the same digest got cancelled.
func (q *queue[Result]) Enqueue(ctx context.Context, req Digest) (<-chan enqueueResult[Result], error) {
	ch := make(chan enqueueResult[Result], 1)
	j := &workerJob[Result]{
		d:  req,
		ch: ch,
	}
	select {
	// enqueue the job
	case q.backlogChan <- j:
		return ch, nil
	// return early if local Enqeue or background context is done
	case <-ctx.Done():
		return nil, ctx.Err()
	case <-q.ctx.Done():
		return nil, q.ctx.Err()
	}
}

// Close cancels the queue context (notifying workers and loop to exit) and waits for them.
func (q *queue[Result]) Close() {
	q.cancel()
	q.wg.Wait()
}

// worker runs in a static goroutine, pulling jobs from toWorkersChan and executing them.
func (q *queue[Result]) worker() {
	for {
		select {
		// pull and execute a job
		case j := <-q.toWorkersChan:
			res, err := q.workFn(q.ctx, j.d)
			// publish job result
			select {
			case q.fromWorkersChan <- workerResult[Result]{d: j.d, res: res, err: err}:
			case <-q.ctx.Done():
				return
			}
		case <-q.ctx.Done():
			return
		}
	}
}

// loop is the coordinator that manages queue state, deduplication, and dispatching.
// mutex free implementations purely based on channel communication.
func (q *queue[Result]) loop() {
	// active requests/jobs per digest
	active := make(map[Digest][]*workerJob[Result])
	running := 0
	// pending jobs waiting to be executed
	var pending []*workerJob[Result]

	// loop to process requests, dispatch jobs and publish results
	for {
		// If we are at max concurrency or have no pending work, activeJobsChan remains nil,
		// and the send case in the select below is ignored.
		var activeJobsChan chan<- *workerJob[Result]
		var nextJob *workerJob[Result]
		if running < q.maxWorkers && len(pending) > 0 {
			activeJobsChan = q.toWorkersChan
			nextJob = pending[0]
		}

		select {
		// Case 1: Handle incoming client requests.
		case j := <-q.backlogChan:
			if jobs, ok := active[j.d]; ok { // digest FOUND
				// Deduplicate: coalesce this job with existing active jobs
				active[j.d] = append(jobs, j)
				break // no need for duplicates to be pending
			}
			active[j.d] = []*workerJob[Result]{j}
			pending = append(pending, j)

		// Case 2: Dispatch the next pending job to an idle worker.
		// This case is only selected if activeJobsChan is non-nil (i.e. we have pending work
		// and we are below the concurrency limit).
		case activeJobsChan <- nextJob: // activeJobsChan alias for toWorkersChan if non-nil
			// Dispatched next job to a waiting worker
			pending[0] = nil // Allow GC of the job struct
			pending = pending[1:]
			running++

		// Case 3: Handle execution results returned by workers.
		case res := <-q.fromWorkersChan:
			waitingResponse := active[res.d]
			delete(active, res.d)
			running--

			// Notify all coalesced clients waiting for this request
			for _, j := range waitingResponse {
				j.ch <- enqueueResult[Result]{Val: res.res, Err: res.err}
				close(j.ch)
			}

		// Case 4: Handle queue shutdown/cancellation.
		case <-q.ctx.Done():
			// Cancel all active/waiting jobs with the context error
			for _, jobs := range active {
				for _, j := range jobs {
					j.ch <- enqueueResult[Result]{Err: q.ctx.Err()}
					close(j.ch)
				}
			}
			// Cancel jobs remaining in backlog
			for {
				select {
				case j := <-q.backlogChan:
					j.ch <- enqueueResult[Result]{Err: q.ctx.Err()}
					close(j.ch)
				default:
					return
				}
			}
		}
	}
}
