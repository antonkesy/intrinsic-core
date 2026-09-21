#!/usr/bin/env bash
# Session-local wrapper for invoking inctl from the @intrinsic-core Bzlmod dependency.
# Usage: source ./env.sh

inctl() {
  bazel run @intrinsic-core//intrinsic/tools/inctl:inctl_external -- "$@"
}
export -f inctl
