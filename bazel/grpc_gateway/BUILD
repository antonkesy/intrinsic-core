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

load("@io_bazel_rules_go//proto:compiler.bzl", "go_proto_compiler")

###################
# Proto compilers #
###################

# This is a redefinition of the `go_proto_compiler` defined in the grpc_gateway project
# that uses different options than the original compiler. It is redefined because there
# is no simple way to add or replace options and have them be replicated to targets by
# Gazelle.
# TODO: b/383722773 - This should be removed.
go_proto_compiler(
    name = "go_gen_grpc_gateway",
    options = [
        "paths=source_relative",
        "logtostderr=true",
        "allow_delete_body=true",
    ],
    plugin = "@com_github_grpc_ecosystem_grpc_gateway_v2//protoc-gen-grpc-gateway",
    suffix = ".pb.gw.go",
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_grpc_ecosystem_grpc_gateway_v2//runtime:go_default_library",
        "@com_github_grpc_ecosystem_grpc_gateway_v2//utilities:go_default_library",
        "@org_golang_google_grpc//grpclog:go_default_library",
        "@org_golang_google_grpc//metadata:go_default_library",
    ],
)

########################################
# Gazelle proto compiler configuration #
########################################

# These configurations are in place to tell Gazelle to use these compilers when generating code from `.proto` files
# We currently have 3 compilers configured:
# 1. Proto → Go (*.pb.go)
# 2. Proto  → Go gRPC (*_grpc.pb.go)
# 3. Proto → gRPC Gateway Go (*.pb.gw.go)

# gazelle:go_proto_compilers @com_github_grpc_ecosystem_grpc_gateway_v2//:go_apiv2
# gazelle:go_grpc_compilers @com_github_grpc_ecosystem_grpc_gateway_v2//:go_apiv2, @com_github_grpc_ecosystem_grpc_gateway_v2//:go_grpc, //bazel/grpc_gateway:go_gen_grpc_gateway

##################################
# Gazelle dependency resolutions #
##################################

# Gazelle uses a dependency resolution algorithm for proto files that is not as robust
# as the one they use for Go.
#
# For a discussion on this, see https://github.com/bazel-contrib/bazel-gazelle/issues/1084.
#
# Because of this, fairly common imports that are correctly resolved for Go BUILD targets do
# not get resolved for Protobuf ones, and instead Gazelle defaults to a local import that
# fails.
#
# In order to address this Gazelle limitation, and to allow it to automatically update proto
# targets, the following sections add `gazelle:resolve` directives to manually indicate how
# to resolve specific proto imports.

#####################################
# Gazelle proto_library resolutions #
#####################################

# gazelle:resolve proto protoc-gen-openapiv2/options/annotations.proto @com_github_grpc_ecosystem_grpc_gateway_v2//protoc-gen-openapiv2/options:options_proto

########################################
# Gazelle go_proto_library resolutions #
########################################

# gazelle:resolve proto go protoc-gen-openapiv2/options/annotations.proto @com_github_grpc_ecosystem_grpc_gateway_v2//protoc-gen-openapiv2/options

##################################
# Gazelle go_library resolutions #
##################################

# gazelle:resolve_regexp go github.com/grpc-ecosystem/grpc-gateway/v2/(.+) @com_github_grpc_ecosystem_grpc_gateway_v2//$1
