#!/usr/bin/env bash
# Session-local wrapper for invoking inctl from the @ioc Bzlmod dependency.
# Usage: source ./env.sh

inctl() {
  bazel run @ioc//intrinsic/tools/inctl:inctl_external -- "$@"
}
export -f inctl
