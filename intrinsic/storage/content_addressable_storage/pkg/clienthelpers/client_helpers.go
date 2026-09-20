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

// Package clienthelpers provides useful constants and helper functions for working with the
// content-addressable storage service.
package clienthelpers

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"hash"
	"hash/crc32"
	"io"
	"syscall"
	"time"

	backoff "github.com/cenkalti/backoff/v4"
	"go.opencensus.io/trace"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

	casgrpcpb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
	caspb "intrinsic/storage/content_addressable_storage/proto/cas_service_go_proto"
)

// DefaultUploadChunkSize is a recommended size of content to be included into one stream request.
//
// Why 1 MiB? https://cloud.google.com/blog/products/gcp/optimizing-your-cloud-storage-performance-google-cloud-performance-atlas
// suggests that 1 MB+ is a good chunk size.
const DefaultUploadChunkSize = 1 * 1024 * 1024

var crc32Castagnoli = crc32.MakeTable(crc32.Castagnoli)

// NewChecksummer returns a checksummer that should be used for checksumming the content sent to the
// content-addressable storage service. See https://cloud.google.com/storage/docs/hashes-etags#crc32c.
func NewChecksummer() hash.Hash32 {
	return crc32.New(crc32Castagnoli)
}

// Get is a convenience function that retrieves an object from CAS and writes the bytes to the
// provided writer. If an error is returned from the CAS service, it is passed down unwrapped so
// that clients can check for the canonical error code.
func Get(ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, objectID string, w io.Writer) error {
	ctx, span := trace.StartSpan(ctx, "clienthelpers.Get")
	defer span.End()

	ctx, cancel := context.WithCancel(ctx)
	defer cancel()

	req := &caspb.GetRequest{ObjectId: objectID}
	stream, err := casClient.Get(ctx, req)
	if err != nil {
		return fmt.Errorf("creating stream for %q: %w", req, err)
	}

	checksummer := NewChecksummer()
	for {
		checksummer.Reset()
		res, err := stream.Recv()
		if err == io.EOF {
			break
		} else if err != nil {
			return err
		}
		content := res.GetChecksummedData().GetContent()
		if _, err := checksummer.Write(content); err != nil {
			return fmt.Errorf("could not checksum content: %w", err)
		}
		if clientCRC, serverCRC := checksummer.Sum32(), res.GetChecksummedData().GetCrc32C(); clientCRC != serverCRC {
			return fmt.Errorf("checksum mismatch: client computed 0x%08x, server computed 0x%08x", clientCRC, serverCRC)
		}
		n, err := w.Write(content)
		if err != nil {
			return fmt.Errorf("could not write data: %w", err)
		}
		if n != len(content) {
			return fmt.Errorf("content write mismatch, wrote %d bytes, got %d from server", n, len(content))
		}
	}
	return nil
}

// GetRange is a convenience function that retrieves an object from CAS starting
// at readOffset and writes the bytes to the provided writer until EOF.
// If an error is returned from the CAS service, it is passed down unwrapped so that clients
// can check for the canonical error code. Returns the total size of the object.
func GetRange(ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, objectID string, readOffset int64, w io.Writer) (int64, error) {
	ctx, span := trace.StartSpan(ctx, "clienthelpers.GetRange")
	defer span.End()

	ctx, cancel := context.WithCancel(ctx)
	defer cancel()

	req := &caspb.GetRangeRequest{
		ObjectId:   objectID,
		ReadOffset: readOffset,
	}
	stream, err := casClient.GetRange(ctx, req)
	if err != nil {
		return 0, fmt.Errorf("creating stream for %q: %w", req.String(), err)
	}

	checksummer := NewChecksummer()
	var totalObjectSize int64
	for {
		checksummer.Reset()
		res, err := stream.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			return 0, err
		}
		totalObjectSize = res.GetTotalObjectSize()
		content := res.GetChecksummedData().GetContent()
		if _, err := checksummer.Write(content); err != nil {
			return 0, fmt.Errorf("could not checksum content: %w", err)
		}
		if clientCRC, serverCRC := checksummer.Sum32(), res.GetChecksummedData().GetCrc32C(); clientCRC != serverCRC {
			return 0, fmt.Errorf("checksum mismatch: client computed 0x%08x, server computed 0x%08x", clientCRC, serverCRC)
		}
		if _, err := w.Write(content); err != nil {
			return 0, fmt.Errorf("could not write data: %w", err)
		}
	}
	return totalObjectSize, nil
}

type downloadOptions struct {
	maxRetries      uint64
	infiniteRetries bool
	initialBackoff  time.Duration
	maxBackoff      time.Duration
}

func defaultDownloadOptions() *downloadOptions {
	return &downloadOptions{
		maxRetries:     5,
		initialBackoff: 100 * time.Millisecond,
		maxBackoff:     2 * time.Second,
	}
}

// DownloadOption configures GetResumable.
type DownloadOption func(*downloadOptions)

// WithMaxRetries configures the maximum number of resume retries upon transient errors.
func WithMaxRetries(retries uint64) DownloadOption {
	return func(o *downloadOptions) {
		o.maxRetries = retries
		o.infiniteRetries = false
	}
}

// WithInfiniteRetries configures GetResumable to retry upon transient errors
// indefinitely until the caller's context is done.
func WithInfiniteRetries() DownloadOption {
	return func(o *downloadOptions) {
		o.infiniteRetries = true
		o.maxRetries = 0
	}
}

// WithRetryBackoff configures the initial and maximum exponential backoff duration.
func WithRetryBackoff(initial, max time.Duration) DownloadOption {
	return func(o *downloadOptions) {
		o.initialBackoff = initial
		o.maxBackoff = max
	}
}

func (o *downloadOptions) backoff(ctx context.Context) backoff.BackOff {
	b := backoff.NewExponentialBackOff(
		backoff.WithInitialInterval(o.initialBackoff),
		backoff.WithMaxInterval(o.maxBackoff),
	)

	var bo backoff.BackOff = b
	if o.infiniteRetries {
		// Disable the default 15-minute MaxElapsedTime so the caller's context
		// is the sole authority for total timeout.
		b.MaxElapsedTime = 0
	} else {
		bo = backoff.WithMaxRetries(b, o.maxRetries)
	}
	return backoff.WithContext(bo, ctx)
}

type countingWriter struct {
	writer  io.Writer
	written int64
}

func (c *countingWriter) Write(p []byte) (int, error) {
	n, err := c.writer.Write(p)
	c.written += int64(n)
	return n, err
}

// GetResumable downloads an object from CAS to the provided io.Writer.
// If w implements io.Seeker (such as an *os.File), it resumes from the current file
// offset. In the event of transient network interruptions, it automatically retries
// with exponential backoff starting from the latest written offset. If the server
// does not implement GetRange and the offset is 0, it falls back to Get.
func GetResumable(
	ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, objectID string, w io.Writer, opts ...DownloadOption,
) error {
	ctx, span := trace.StartSpan(ctx, "clienthelpers.GetResumable")
	defer span.End()

	o := defaultDownloadOptions()
	for _, opt := range opts {
		opt(o)
	}

	var startOffset int64
	var seeker io.Seeker
	// Check if the destination supports seeking (like *os.File):
	// 1. Query the current cursor position to resume any existing partial file.
	// 2. Save the seeker handle to re-align the file cursor if retries occur.
	// Non-seekable writers (e.g. http.ResponseWriter, bytes.Buffer, pipes) default to offset 0.
	if s, ok := w.(io.Seeker); ok {
		// Set the cursor to the point where download stopped.
		pos, err := s.Seek(0, io.SeekCurrent)
		if err == nil { // if NO error
			startOffset = pos
			seeker = s
		} else if !errors.Is(err, syscall.ESPIPE) {
			// *os.File implements io.Seeker, but pipes, sockets, and os.Stdout return
			// syscall.ESPIPE ("Illegal seek"). We ignore ESPIPE so that streaming to pipes
			// works starting from offset 0, but propagate any real I/O error.
			return fmt.Errorf("determining current offset for %q: %w", objectID, err)
		}
	}

	cw := &countingWriter{writer: w, written: startOffset}
	b := o.backoff(ctx)

	return backoff.Retry(func() error {
		if seeker != nil {
			// sync the cursor to the latest tracked progress
			if _, err := seeker.Seek(cw.written, io.SeekStart); err != nil {
				return backoff.Permanent(fmt.Errorf("seeking to offset %d in writer: %w", cw.written, err))
			}
		}

		_, err := GetRange(ctx, casClient, objectID, cw.written, cw)
		if err == nil {
			return nil
		}

		// Fallback to Get if GetRange is unsupported and we are at offset 0.
		if status.Code(err) == codes.Unimplemented {
			if cw.written != 0 {
				return backoff.Permanent(err)
			}

			if err := Get(ctx, casClient, objectID, w); err != nil {
				return backoff.Permanent(err)
			}
			return nil
		}

		if !isRetriable(err) {
			return backoff.Permanent(err)
		}

		return err
	}, b)
}

func isRetriable(err error) bool {
	switch status.Code(err) {
	case codes.Unavailable, codes.DeadlineExceeded, codes.ResourceExhausted, codes.Aborted:
		return true
	}
	return false
}

// GetAsProto is a convenience function that retrieves an object from CAS and then unmarshals the
// bytes to the provided [proto.Message] type T. If an error is returned from the CAS service, it is
// passed down unwrapped so that clients can check for the canonical error code.
func GetAsProto[T proto.Message](ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, objectID string) (T, error) {
	ctx, span := trace.StartSpan(ctx, "blobstorage.GetAsProto")
	defer span.End()

	var nilT T
	buf := new(bytes.Buffer)
	if err := Get(ctx, casClient, objectID, buf); err != nil {
		return nilT, err
	}

	var res T
	res = res.ProtoReflect().New().Interface().(T)
	if err := proto.Unmarshal(buf.Bytes(), res); err != nil {
		return nilT, fmt.Errorf("could not unmarshal bytes to type %T: %w", res, err)
	}
	return res, nil
}

// Create is a convenience function that reads from the provided reader object and writes these
// bytes to CAS. This function returns the object ID and an error. In case an error is returned
// from the CAS service, it is passed down unwrapped so that clients can check for the canonical
// error code.
func Create(ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, r io.Reader, chunkSize int64) (string, error) {
	ctx, span := trace.StartSpan(ctx, "clienthelpers.Create")
	defer span.End()
	var err error
	ctx, cancel := context.WithCancelCause(ctx)
	defer cancel(err)

	stream, err := casClient.Create(ctx)
	if err != nil {
		return "", err
	}
	checksummer := NewChecksummer()
	buf := make([]byte, chunkSize)
	for {
		checksummer.Reset()

		n, readerErr := io.ReadFull(r, buf)
		if readerErr != io.EOF && readerErr != io.ErrUnexpectedEOF && readerErr != nil {
			err = fmt.Errorf("could not read from reader: %w", readerErr)
			return "", err
		}
		if n == 0 {
			break
		}
		content := buf[:n]
		if _, err = checksummer.Write(content); err != nil {
			err = fmt.Errorf("could not checksum buf: %w", err)
			return "", err
		}
		req := &caspb.CreateRequest{
			ChecksummedData: &caspb.ChecksummedData{
				Content: content,
				Crc32C:  proto.Uint32(checksummer.Sum32()),
			},
		}
		if err = stream.Send(req); err != nil {
			return "", stream.RecvMsg(nil)
		}
		if readerErr == io.EOF || readerErr == io.ErrUnexpectedEOF {
			break
		}
	}
	res, err := stream.CloseAndRecv()
	if err != nil {
		return "", fmt.Errorf("closing stream: %w", err)
	}
	return res.GetObjectId(), nil
}

// CreateFromProto is a convenience function that takes a [proto.Message], marshals it to the wire
// format, and writes these bytes to CAS. This function returns the object ID and an error. In case
// an error is returned from the CAS service, it is passed down unwrapped so that clients can check
// for the canonical error code.
func CreateFromProto(ctx context.Context, casClient casgrpcpb.ContentAddressableStorageServiceClient, msg proto.Message, chunkSize int64) (string, error) {
	ctx, span := trace.StartSpan(ctx, "clienthelpers.CreateFromProto")
	defer span.End()

	if msg == nil {
		return "", fmt.Errorf("nil proto message provided")
	}

	d, err := proto.Marshal(msg)
	if err != nil {
		return "", fmt.Errorf("could not marshal proto: %w", err)
	}

	r := bytes.NewReader(d)
	return Create(ctx, casClient, r, chunkSize)
}
