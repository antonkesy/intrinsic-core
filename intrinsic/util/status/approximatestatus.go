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

// Package approximatestatus provides a way to approximate the status code of
// an error. This is a replacement for the internal package util/task/go/status.go that cannot be
// externalized to go/insrc.
//
// Code was copied out from http://util/task/go/status.go;l=862;rcl=737244524
package approximatestatus

import (
	"context"
	"errors"
	"io"
	"os"
	"strconv"
	"strings"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

// statuser is the interface implemented by errors that have a gRPC status
// attached.  If Status returns an OK status, it is treated the
// same as if it didn't implement the interface.
type statuser interface {
	GRPCStatus() *status.Status
}

// convertCode converts a standard Go error into a canonical code.
func convertCode(err error) codes.Code {
	switch err {
	case io.EOF:
		return codes.OutOfRange
	case io.ErrClosedPipe, io.ErrNoProgress, io.ErrShortBuffer, io.ErrShortWrite, io.ErrUnexpectedEOF:
		return codes.FailedPrecondition
	case os.ErrInvalid:
		return codes.InvalidArgument
	case context.Canceled:
		return codes.Canceled
	case context.DeadlineExceeded:
		return codes.DeadlineExceeded
	}
	if pathErr, ok := err.(*os.PathError); ok && pathErr.Err == nil {
		return codes.OK
	}
	switch {
	case os.IsExist(err):
		return codes.AlreadyExists
	case os.IsNotExist(err):
		return codes.NotFound
	case os.IsPermission(err):
		return codes.PermissionDenied
	}
	if numErr, ok := err.(*strconv.NumError); ok {
		switch numErr.Err {
		case strconv.ErrRange:
			return codes.OutOfRange
		case strconv.ErrSyntax:
			return codes.InvalidArgument
		default:
			// The only other types of strconv.NumError are strconv.baseError and
			// bitSizeError, but they are private.
			// However, their strings begin "invalid base" and "invalid bit size".
			if strings.HasPrefix(numErr.Err.Error(), "invalid base ") || strings.HasPrefix(numErr.Err.Error(), "invalid bit size ") {
				return codes.InvalidArgument
			}
		}
	}
	return codes.Unknown
}

// strconvCanonical converts a strconv.NumError into a canonical code.
//
// This error type can't be handled in convertCode, which operates on leaf errors.
// NumError wraps an error and must be checked for before unwrapping.
func strconvCanonical(numErr *strconv.NumError) codes.Code {
	switch numErr.Err {
	case strconv.ErrRange:
		return codes.OutOfRange
	case strconv.ErrSyntax:
		return codes.InvalidArgument
	default:
		// The only other types of strconv.NumError are strconv.baseError and
		// bitSizeError, but they are private.
		// However, their strings begin "invalid base" and "invalid bit size".
		if strings.HasPrefix(numErr.Err.Error(), "invalid base ") || strings.HasPrefix(numErr.Err.Error(), "invalid bit size ") {
			return codes.InvalidArgument
		}
	}
	return codes.Unknown
}

// Code converts an error into the matching canonical status code.
//
// If the error contains a Status, then CanonicalCode is equivalent to
// calling the CanonicalCode method on the result of FromError.
//
// If err is or wraps a known error from the "io", "os", "net/url", or
// "context" packages, CanonicalCode returns a status with a code
// that best approximates err.
//
// Otherwise, it returns an status with code UNKNOWN.
func Code(err error) codes.Code {
	if err == nil {
		return codes.OK
	}
	for {
		switch e := err.(type) {
		case statuser:
			return e.GRPCStatus().Code()
		case *strconv.NumError:
			return strconvCanonical(e)
		}

		u := errors.Unwrap(err)
		if u == nil {
			return convertCode(err)
		}

		err = u
	}
}
