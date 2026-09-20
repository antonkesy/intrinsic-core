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

"""OpenTelemetry Tracing support utilities for ML Services."""

from absl import logging
from opentelemetry import trace
from opentelemetry.exporter.cloud_trace import CloudTraceSpanExporter
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.instrumentation.grpc import GrpcInstrumentorClient
from opentelemetry.instrumentation.grpc import GrpcInstrumentorServer
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.sdk.trace.sampling import ALWAYS_ON
from opentelemetry.sdk.trace.sampling import DEFAULT_ON

# Setup a global tracer instance for ML platform code.
ML_TRACER_NAME = 'ai.intrinsic.ml'
tracer = trace.get_tracer(ML_TRACER_NAME)

# Default in-cluster OTLP endpoint for OpenTelemetry collector.
_DEFAULT_COLLECTOR_ENDPOINT = (
    'oc-agent.app-intrinsic-base.svc.cluster.local:4317'
)


def setup_tracing(
    service_name: str,
    service_namespace: str | None = None,
    gcp_project_id: str | None = None,
    endpoint: str = _DEFAULT_COLLECTOR_ENDPOINT,
    always_on: bool = False,
) -> None:
  """Initializes OpenTelemetry tracing.

  This setup is idempotent (does nothing if a TracerProvider is already
  registered) and catches initialization errors to prevent crashing.

  Args:
    service_name: Name of the service to register (used to group spans).
    service_namespace: Optional namespace grouping (e.g. 'skills' or 'services')
      to classify microservices.
    gcp_project_id: GCP Project ID to export traces directly to Google Cloud
      Trace. If specified, overrides OTLP exporter.
    endpoint: Custom endpoint for the OTLP collector. If not specified (and not
      exporting to GCP), it defaults to the in-cluster 'oc-agent' gateway.
    always_on: If True, always record and export spans. If False, uses the
      default ParentBased sampler which respects the caller's sampling decision.
  """
  # Check if trace provider is already set to prevent re-initialization.
  if isinstance(trace.get_tracer_provider(), TracerProvider):
    logging.debug('OpenTelemetry tracing already initialized.')
    return

  try:
    # Configure the resource to define service identity & namespace grouping.
    # Ref: https://github.com/open-telemetry/semantic-conventions/blob/main/docs/resource/service.md
    resource_attributes = {'service.name': service_name}
    if service_namespace:
      resource_attributes['service.namespace'] = service_namespace
    # Bind service identity to OpenTelemetry resources.
    resource = Resource.create(resource_attributes)
    # Initialize SDK TracerProvider with the service resource.
    provider = TracerProvider(
        resource=resource, sampler=ALWAYS_ON if always_on else DEFAULT_ON
    )

    if gcp_project_id:
      logging.info(
          'Initializing OpenTelemetry tracing for %s exporting directly to GCP'
          ' Trace (project: %s)',
          service_name,
          gcp_project_id,
      )
      # Export all resource attributes (e.g. service name) as span labels.
      # By default GCP exporter filters them to stay under the 32-label limit.
      exporter = CloudTraceSpanExporter(
          project_id=gcp_project_id,
          resource_regex='.*',
      )
    else:
      logging.info(
          'Initializing OpenTelemetry tracing for %s targeting OTLP/gRPC: %s',
          service_name,
          endpoint,
      )
      exporter = OTLPSpanExporter(endpoint=endpoint, insecure=True)

    # Use BatchSpanProcessor to buffer spans and export them in batches.
    provider.add_span_processor(BatchSpanProcessor(exporter))
    trace.set_tracer_provider(provider)

    # Instrument gRPC client and server for automatic context propagation.
    # Automatically extracts and injects Trace IDs and Span IDs.
    GrpcInstrumentorClient().instrument()
    GrpcInstrumentorServer().instrument()

    logging.info('OpenTelemetry tracing successfully enabled.')
  except Exception as e:  # pylint: disable=broad-except
    # Tracing failures must NOT crash target applications.
    logging.exception(
        'Failed to initialize OpenTelemetry tracing: %s. Tracing disabled.', e
    )
