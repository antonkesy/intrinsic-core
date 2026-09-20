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

// Package transaction provided a basic transaction interface for best effort
// transactions across distributed systems.  The transactions are best effort,
// as a rollback can only be requested, not required, if any one operation in a
// transaction fails.
package transaction

import (
	"context"
	"fmt"

	log "github.com/golang/glog"
	"golang.org/x/sync/errgroup"
)

type operation struct {
	name      string
	completed bool
	// commit will be run when the transaction is committed.
	commit func(ctx context.Context) error
	// rollback will be run if the transaction is rolled-back due to failure.
	// This is done on a best effort basis and only a warning will be logged
	// with the returned error.
	rollback func(ctx context.Context) error
	// cleanup will be run after either the success or failure of an operation.
	cleanup func(ctx context.Context) error
}

// Option is an interface that applies options to operations added to the
// transaction.
type Option interface {
	apply(op *operation)
}

// Transaction defines the interface for a transaction that may be implemented
// with various different strategies.
type Transaction interface {
	// Adds an operation to the transaction with the given options.
	Add(func(ctx context.Context) error, ...Option)
	// Commits the transaction.
	Commit(context.Context) error
}

type unordered struct {
	ops []*operation
}

// NewUnordered creates a transaction where operations are completed in an
// arbitrary order.  Operations may be run simultaneously.  Rollbacks and
// cleanup operations will be performed in a similar manner.
func NewUnordered() Transaction {
	return &unordered{}
}

type option func(*operation)

func (f option) apply(op *operation) {
	f(op)
}

// WithName will name the operation in logs.
func WithName(name string) Option {
	return option(func(op *operation) {
		op.name = name
	})
}

// WithRollback adds a rollback function to the operation.  It will only be run
// if an operation is completed successfully.
func WithRollback(rollback func(ctx context.Context) error) Option {
	return option(func(op *operation) {
		op.rollback = rollback
	})
}

// WithCleanup adds a cleanup function for the operation.  It will always be
// run, regardless of whether the operation completed or not.
func WithCleanup(cleanup func(ctx context.Context) error) Option {
	return option(func(op *operation) {
		op.cleanup = cleanup
	})
}

func (t *unordered) Add(commit func(ctx context.Context) error, opts ...Option) {
	op := &operation{
		name:   fmt.Sprintf("%d", len(t.ops)),
		commit: commit,
	}
	for _, opt := range opts {
		opt.apply(op)
	}
	t.ops = append(t.ops, op)
}

func (t *unordered) Commit(ctx context.Context) error {
	group, childCtx := errgroup.WithContext(ctx)
	for _, op := range t.ops {
		ctx := childCtx
		group.Go(func() error {
			if err := op.commit(ctx); err != nil {
				log.ErrorContextf(ctx, "operation %q failed: %v", op.name, err)
				return err
			}
			op.completed = true
			return nil
		})
	}
	err := group.Wait()
	if err != nil {
		log.InfoContextf(ctx, "one or more operations in commit, attempting a rollback: %v", err)
		group, childCtx := errgroup.WithContext(ctx)
		for _, op := range t.ops {
			ctx := childCtx
			if op.rollback == nil || !op.completed {
				continue
			}
			group.Go(func() error {
				if err := op.rollback(ctx); err != nil {
					log.WarningContextf(ctx, "unable to rollback operation %q: %v", op.name, err)
					return err
				}
				return nil
			})
		}
		if err := group.Wait(); err != nil {
			log.ErrorContextf(ctx, "one or more rollbacks failed, system may be in an inconsistent state: %v", err)
		}
	}
	group, childCtx = errgroup.WithContext(ctx)
	for _, op := range t.ops {
		ctx := childCtx
		if op.cleanup == nil {
			continue
		}
		group.Go(func() error {
			if err := op.cleanup(ctx); err != nil {
				log.WarningContextf(ctx, "unable to cleanup operation %q: %v", op.name, err)
				return err
			}
			return nil
		})
	}
	if err := group.Wait(); err != nil {
		log.WarningContextf(ctx, "one or more cleanups failed: %v", err)
	}
	return err
}
