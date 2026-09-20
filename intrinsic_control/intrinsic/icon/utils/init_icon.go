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

// Prepares environment variables and retries for ICON binaries.
package main

import (
	"flag"
	"os"
	"syscall"

	"intrinsic/kubernetes/intrinsic"

	log "github.com/golang/glog"
)

const (
	usageIcon = "usage: init_icon -- path/to/executable [args [...]]"
)

var (
	// We avoid bringing these constants in via cgo because it leads to bad packaging interactions with pkg_tar.
	// LINT.IfChange
	// Corresponds to intrinsic::icon::ExitCode::kFatalFaultDuringInit
	exitCodeFatalFaultDuringInit = 111
	// Corresponds to intrinsic::icon::ExitCode::kFatalFaultDuringExec
	exitCodeFatalFaultDuringExec = 112
	// LINT.ThenChange(//intrinsic_control/intrinsic/icon/utils/exit_code.h)
)

func main() {
	os.Exit(run())
}

func run() int {
	intrinsic.Init()

	args := flag.Args()
	if len(args) == 0 {
		log.Errorf("Bad invocation: No args given\n%s", usageIcon)
		return 1
	}

	// Make a copy of args before modifying it to avoid side effects if runMain is called again.
	runnerArgs := make([]string, len(args))
	copy(runnerArgs, args)
	runnerArgs = append(runnerArgs, "--opencensus_tracing")

	return runMain(RunnerOptions{
		Args: runnerArgs,
		RestartExitCodes: []int{
			exitCodeFatalFaultDuringInit,
			exitCodeFatalFaultDuringExec,
		},
		IgnoredSignals: map[os.Signal]bool{
			syscall.SIGCHLD: true,
			// Generates logspam: https://github.com/golang/go/issues/37942
			syscall.SIGURG: true,
		},
	})
}
