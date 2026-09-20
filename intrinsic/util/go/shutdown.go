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

// Package shutdown provides helpers to handle shutdown signals.
package shutdown

import (
	"context"
	"os"
	"os/signal"
	"syscall"
)

// defaultSignals provides a list of default signals used by both Register
// and RegisterContext.  While handling only SIGTERM is likely required for
// servers running in k8s (which should use register), I've added SIGINT
// and SIGHUP for local testing of binaries and containers.  SIGINT will be
// sent by ctrl-c and SIGHUP will be sent by programs like systemd to
// reload a configuration.  Given that most binaries simply load
// configuration at startup, this registers that here as well.  A similar
// logic is used for RegisterContext, which will likely be used in command
// line utilities.  Some programs will catch an interrupt and have
// additional logic.  Ours generally do not and expect to quit.  As a
// result, we wire up SIGHUP/INT/TERM to all be shutdown signals to allow
// the program to cancel operations and terminate gracefully.
var defaultSignals = []os.Signal{
	syscall.SIGHUP,
	syscall.SIGINT,
	syscall.SIGTERM,
}

type registerOpts struct {
	signals []os.Signal
}

// RegisterOpt defines an interface for adjusting options for Register.
type RegisterOpt interface {
	apply(*registerOpts)
}

type registerOpt func(*registerOpts)

func (o registerOpt) apply(opts *registerOpts) {
	o(opts)
}

// WithSignals allows customizing the signals should indicate a shutdown.  By
// default this is SIGTERM, SIGINT, and SIGHUP.
func WithSignals(signals []os.Signal) RegisterOpt {
	return registerOpt(func(opts *registerOpts) {
		opts.signals = signals
	})
}

// Register creates a channel that will listen for shutdown signals.  Multiple
// calls to this function will not unregister prior calls.  The returned
// channel will never return a value, but will be closed upon receiving a
// signal.  This notification should be handled promptly, otherwise the program
// may be forcefully terminated.  This is expected to typically be used in
// server implementations.
//
// Example:
//
//	func main() {
//	  intrinsic.Init()
//	  ctx := context.Background()
//	  stop := shutdown.Register()
//
//	  // Application logic, such as starting a server that returns a close
//	  // function.
//	  close := startServer()
//	  defer close()
//
//	  <-stop
//	  log.InfoContext(ctx, "Received shutdown signal, stopping")
//	}
//
// Users that need to know the signal that was received or handle a signal more
// than once should use the signal library directly.
func Register(opts ...RegisterOpt) <-chan struct{} {
	opt := &registerOpts{
		signals: defaultSignals,
	}
	for _, o := range opts {
		o.apply(opt)
	}
	recv := make(chan os.Signal, 1)
	signal.Notify(recv, opt.signals...)
	shutdown := make(chan struct{}, 0)
	go func() {
		<-recv
		close(shutdown)
	}()
	return shutdown
}

// RegisterContext creates a new context that will listen for shutdown signals.
// The returned context is cancelled when either stop is called or the
// registered signals are triggered.  By default, the program will no longer
// quit when receiving an interrupt.  This should typically be used by client
// programs, such as inctl.
//
// Example:
//
//	func main() {
//	  intrinsic.Init()
//	  ctx := shutdown.RegisterContext(context.Background())
//
//	  // Application logic, such as this request that would be cancelled on a
//	  // ctrl-c.
//	  resp, err := client.GetFoo(ctx, &foo.GetFooRequest{})
//	}
//
// Users that need to know the signal that was received or handle a signal more
// than once should use the signal library directly.
func RegisterContext(ctx context.Context, opts ...RegisterOpt) (context.Context, context.CancelFunc) {
	opt := &registerOpts{
		signals: defaultSignals,
	}
	for _, o := range opts {
		o.apply(opt)
	}
	return signal.NotifyContext(ctx, opt.signals...)
}
