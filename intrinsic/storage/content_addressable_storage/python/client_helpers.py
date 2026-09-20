# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Useful constants and helper functions for working with the content-addressable storage (CAS) service."""

from collections.abc import Iterator
from collections.abc import Sequence
import io
import random
import time
from typing import Any
from typing import List
from typing import Optional
from typing import Protocol
from typing import runtime_checkable
from typing import Tuple

from absl import logging
import grpc

from intrinsic.storage.content_addressable_storage.proto import cas_service_pb2
from intrinsic.storage.content_addressable_storage.proto import cas_service_pb2_grpc

# Recommended size of content to be included into one stream request.
#
# Why 1 MiB?
# https://cloud.google.com/blog/products/gcp/optimizing-your-cloud-storage-performance-google-cloud-performance-atlas
# suggests that 1 MB+ is a good chunk size.
DEFAULT_UPLOAD_CHUNK_SIZE = 1 * 1024 * 1024  # 1 MiB

# Log a warning if user attempts to download a large file using
# client_helpers.get. The function is memory intensive and can be slow due
# to repeated memory re-allocation.
FILE_SIZE_THRESHOLD = 500 * 1024 * 1024  # 500 MiB.


@runtime_checkable
class BinaryWriter(Protocol):
  """Protocol representing an object that can write raw bytes"""

  def write(self, data: bytes, /) -> Any:
    ...


@runtime_checkable
class Seeker(Protocol):
  """Protocol representing a seekable stream (similar to Go's io.Seeker)."""

  def seek(self, offset: int, whence: int = 0, /) -> int:
    ...

  def tell(self) -> int:
    ...


class IncompleteDownloadError(RuntimeError):
  """Raised when a download terminates prematurely before receiving all expected bytes."""


_RETRIABLE_STATUS_CODES = {
    grpc.StatusCode.UNAVAILABLE,
    grpc.StatusCode.RESOURCE_EXHAUSTED,
    grpc.StatusCode.DEADLINE_EXCEEDED,
    grpc.StatusCode.ABORTED,
}


def _grpc_status_code(exc: BaseException) -> Optional[grpc.StatusCode]:
  """Returns the gRPC StatusCode if the error is a gRPC call exception."""
  return exc.code() if isinstance(exc, grpc.Call) else None


def _is_retriable(exc: BaseException) -> bool:
  """Returns True if the error is a retriable gRPC error or truncated stream."""
  return (
      isinstance(exc, IncompleteDownloadError)
      or _grpc_status_code(exc) in _RETRIABLE_STATUS_CODES
  )


class _CountingWriter:
  """Wrapper around a writer that tracks total bytes written."""

  def __init__(self, writer: BinaryWriter):
    self._writer = writer
    self.bytes_written = 0

  def write(self, data: bytes) -> int:
    n = self._writer.write(data)
    written = len(data) if n is None else n
    self.bytes_written += written
    return written


def get_range(
    cas_stub: cas_service_pb2_grpc.ContentAddressableStorageServiceStub,
    object_id: str,
    start_offset: int,
    writer: BinaryWriter,
    grpc_metadata: Optional[Sequence[Tuple[str, str]]] = None,
) -> int:
  """Downloads content from the CAS service starting from start_offset.

  Args:
    cas_stub: Stub for the content-addressable storage service.
    object_id: CAS object ID.
    start_offset: Zero-based byte offset to start reading from.
    writer: An object that implements write(bytes).
    grpc_metadata: Optional gRPC metadata.

  Returns:
    Total bytes read and written to the writer.

  Raises:
    ValueError: If start_offset is invalid (< 0).
    IncompleteDownloadError: If received byte count doesn't match expected size.
    grpc.RpcError: If the gRPC call fails.
  """
  if start_offset < 0:
    raise ValueError(f"Invalid start offset {start_offset}; must be >= 0.")

  request = cas_service_pb2.GetRangeRequest(
      object_id=object_id,
      read_offset=start_offset,
  )

  bytes_read = 0
  total_object_size: Optional[int] = None
  grpc_kwargs = {"metadata": grpc_metadata} if grpc_metadata else {}

  for response in cas_stub.GetRange(request, **grpc_kwargs):
    if response.total_object_size:
      total_object_size = response.total_object_size
    chunk = response.checksummed_data.content
    writer.write(chunk)
    bytes_read += len(chunk)

  if total_object_size:
    expected_bytes = total_object_size - start_offset
    if bytes_read != expected_bytes:
      raise IncompleteDownloadError(
          f"The total expected size for '{object_id}' from offset"
          f" {start_offset} is {expected_bytes} bytes, but received"
          f" {bytes_read} bytes."
      )

  return bytes_read


def _prepare_retry(
    exc: BaseException,
    object_id: str,
    seeker: Optional[Seeker],
    resume_offset: int,
    consecutive_retries: int,
    current_backoff: float,
) -> None:
  """Logs the retry attempt, rewinds seeker if needed, and sleeps.

  Raises:
    RuntimeError: If rewinding the seeker to the resume offset fails.
  """
  logging.warning(
      "Transient error during CAS download of %r at offset %d: %s."
      " Retrying (%d) in %.2fs...",
      object_id,
      resume_offset,
      exc,
      consecutive_retries,
      current_backoff,
  )

  # sync the cursor before retrying
  if seeker:
    try:
      seeker.seek(resume_offset)
    except (OSError, io.UnsupportedOperation) as seek_err:
      raise RuntimeError(
          f"Failed to seek back on retry: {seek_err} (original error: {exc})"
      ) from exc

  time.sleep(random.uniform(0, current_backoff))


def get_resumable(
    cas_stub: cas_service_pb2_grpc.ContentAddressableStorageServiceStub,
    object_id: str,
    writer: BinaryWriter,
    max_retries: int = 5,
    initial_backoff_sec: float = 0.1,
    max_backoff_sec: float = 2.0,
    backoff_multiplier: float = 1.5,
    infinite_retries: bool = False,
    grpc_metadata: Optional[Sequence[Tuple[str, str]]] = None,
) -> int:
  """Downloads an object with automatic chunk-level resumption on transient
    failures.

  Args:
    cas_stub: Stub for the content-addressable storage service.
    object_id: CAS object ID.
    writer: An object implementing write(bytes). If seekable, it will be rewound
      to the current offset on retry.
    max_retries: Maximum consecutive retries before failing (ignored if
      infinite_retries is True).
    initial_backoff_sec: Initial backoff duration in seconds.
    max_backoff_sec: Maximum backoff duration in seconds.
    backoff_multiplier: Multiplier for exponential backoff.
    infinite_retries: If True, retries indefinitely on retriable errors.
    grpc_metadata: Optional gRPC metadata to attach to each RPC. Note: any
      credentials passed here are static and will not be refreshed. If
      credentials need to be auto-updated (e.g. for long-running downloads that
      may exceed 1 hour), pass None here and configure credentials on the
      channel used to initialize cas_stub.

  Returns:
    Total bytes written to the writer.

  Raises:
    RuntimeError / grpc.RpcError: If download fails permanently or retries are
      exhausted.

  Example:
    To ensure credentials auto-refresh across long downloads (> 1 hour),
    initialize ``cas_stub`` using an authenticated channel (e.g. via
    ``dialerutil.create_channel_from_org``) rather than passing static
    ``grpc_metadata``:

      from intrinsic.storage.content_addressable_storage.proto import cas_service_pb2_grpc
      from intrinsic.storage.content_addressable_storage.python import client_helpers
      from intrinsic.util.grpc import auth, dialerutil

      org_info = auth.parse_info_from_string("<org>@<project>")
      with dialerutil.create_channel_from_org(org_info) as channel:
        stub = cas_service_pb2_grpc.ContentAddressableStorageServiceStub(channel)
        client_helpers.get_resumable(stub, object_id, writer)
  """
  start_offset = 0
  seeker: Optional[Seeker] = writer if isinstance(writer, Seeker) else None
  if seeker:
    try:
      start_offset = seeker.tell()
    except (OSError, io.UnsupportedOperation) as e:
      logging.warning("Writer is not seekable (tell() failed): %s", e)
      seeker = None

  counting_writer = _CountingWriter(writer)
  consecutive_retries = 0
  current_backoff = initial_backoff_sec

  while True:
    bytes_before = counting_writer.bytes_written
    try:
      current_offset = start_offset + bytes_before
      get_range(
          cas_stub,
          object_id,
          current_offset,
          counting_writer,
          grpc_metadata=grpc_metadata,
      )
      return counting_writer.bytes_written
    except (grpc.RpcError, IncompleteDownloadError) as e:
      if (
          _grpc_status_code(e) == grpc.StatusCode.UNIMPLEMENTED
          and start_offset == 0
          and counting_writer.bytes_written == 0
      ):
        for chunk in get_iter(cas_stub, object_id, grpc_metadata):
          counting_writer.write(chunk)
        return counting_writer.bytes_written

      if not _is_retriable(e):
        raise

      if counting_writer.bytes_written > bytes_before:
        consecutive_retries = 0
        current_backoff = initial_backoff_sec

      consecutive_retries += 1
      if not infinite_retries and consecutive_retries > max_retries:
        raise

      _prepare_retry(
          exc=e,
          object_id=object_id,
          seeker=seeker,
          resume_offset=start_offset + counting_writer.bytes_written,
          consecutive_retries=consecutive_retries,
          current_backoff=current_backoff,
      )
      current_backoff = min(
          current_backoff * backoff_multiplier, max_backoff_sec
      )


def get_iter(
    cas_stub: cas_service_pb2_grpc.ContentAddressableStorageServiceStub,
    object_id: str,
    grpc_metadata: Optional[Sequence[Tuple[str, str]]] = None,
) -> Iterator[bytes]:
  """Generator to retrieve an object from CAS.

  Prefer this function when the object is expected to be large or when written
  directly to a file.

  Args:
    cas_stub: Stub for the content-addressable storage service.
    object_id: CAS object ID.
    grpc_metadata: Optional gRPC metadata to be attached to the request.

  Yields:
    A chunk of the object. Its size is determined by the server.
  """
  stat_request = cas_service_pb2.StatRequest(object_id=object_id)
  stat_response = cas_stub.Stat(stat_request, metadata=grpc_metadata)
  expected_total_size = stat_response.size

  request = cas_service_pb2.GetRequest(object_id=object_id)
  total_received_size = 0
  if grpc_metadata:
    for response in cas_stub.Get(request, metadata=grpc_metadata):
      total_received_size += len(response.checksummed_data.content)
      yield response.checksummed_data.content
  else:
    for response in cas_stub.Get(request):
      # TODO: b/289500064 - Add checksum check.
      total_received_size += len(response.checksummed_data.content)
      yield response.checksummed_data.content

  # Check if all bytes have arrived. This is done because there is no checksum
  # check over the whole blob when streamed and we have observed incomplete
  # downloads without raised exceptions before. This led to hard to debug
  # downstream failures.
  if total_received_size != expected_total_size:
    error_message = (
        f"The total expected size of the downloaded CAS blob '{object_id}' is"
        " not equal to the sum of received bytes over all chunks!"
        f" (expected={expected_total_size} | received={total_received_size})!"
    )
    logging.error(error_message)
    raise IncompleteDownloadError(error_message)


def get(
    cas_stub: cas_service_pb2_grpc.ContentAddressableStorageServiceStub,
    object_id: str,
    grpc_metadata: Optional[List[Tuple[str, str]]] = None,
) -> bytes:
  """Retrieve an object from CAS into a bytes-like array.

  Prefer this function when the object is expected to be small and it's ok to
  allocate memory for it.

  Args:
    cas_stub: Stub for the content-addressable storage service.
    object_id: CAS object ID.
    grpc_metadata: Optional gRPC metadata to be attached to the request.

  Returns:
    A bytes-like array containing the object retrieved from CAS.
  """
  data = bytearray()
  warning_issued = False
  for chunk in get_iter(cas_stub, object_id, grpc_metadata):
    data += chunk
    if (not warning_issued) and (len(data) > FILE_SIZE_THRESHOLD):
      logging.warning(
          "client_helpers.get is downloading a large object in memory"
          " (>%.2f MiB) which can be slow and memory intensive. Prefer using"
          " client_helpers.get_iter to iteratively retrieve chunks of large"
          " files and save them to disk.",
          FILE_SIZE_THRESHOLD / (1024.0 * 1024.0),
      )
      warning_issued = True

  return bytes(data)


def create_from_reader(
    cas_stub: cas_service_pb2_grpc.ContentAddressableStorageServiceStub,
    source: io.IOBase,
    chunk_size: int = DEFAULT_UPLOAD_CHUNK_SIZE,
    grpc_metadata: Optional[List[Tuple[str, str]]] = None,
) -> str:
  """Write data to CAS from an object that implements read().

  Prefer this function when the object is large or when you're reading from a
  file.

  Args:
    cas_stub: Stub for the content-addressable storage service.
    source: Any object that implements read().
    chunk_size: Chunk size used for the upload, should be between 1 MB and 4 MB.
    grpc_metadata: Optional gRPC metadata to be attached to the request.

  Returns:
    CAS object ID of the uploaded data.
  """
  requests = (
      _make_create_request(chunk)
      for chunk in iter(lambda: source.read(chunk_size), b"")
  )
  if grpc_metadata:
    response = cas_stub.Create(requests, metadata=grpc_metadata)
  else:
    response = cas_stub.Create(requests)
  return response.object_id


def _make_create_request(chunk: bytes) -> cas_service_pb2.CreateRequest:
  return cas_service_pb2.CreateRequest(
      checksummed_data=cas_service_pb2.ChecksummedData(
          content=chunk
          # TODO: b/289500064 - Add checksum check.
      )
  )


def create(
    cas_stub: cas_service_pb2_grpc.ContentAddressableStorageServiceStub,
    data: bytes,
    chunk_size: int = DEFAULT_UPLOAD_CHUNK_SIZE,
    grpc_metadata: Optional[List[Tuple[str, str]]] = None,
) -> str:
  """Write data to CAS from an in-memory bytes-like object.

  Prefer this function for small objects that are held in memory.

  Args:
    cas_stub: Stub for the content-addressable storage service.
    data: A bytes-like array.
    chunk_size: Chunk size used for the upload, should be between 1 MB and 4 MB.
    grpc_metadata: Optional gRPC metadata to be attached to the request.

  Returns:
    CAS object ID of the uploaded data.
  """
  return create_from_reader(
      cas_stub, io.BytesIO(data), chunk_size, grpc_metadata
  )
