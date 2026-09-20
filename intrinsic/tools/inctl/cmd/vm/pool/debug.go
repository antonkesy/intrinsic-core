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

package pool

import (
	"context"
	"fmt"
	"strings"
	"time"

	"intrinsic/tools/inctl/util/agents"
	"intrinsic/tools/inctl/util/cobrautil"
	"intrinsic/tools/inctl/util/orgutil"

	log "github.com/golang/glog"
	"github.com/spf13/cobra"
	"go.opencensus.io/trace"
	"google.golang.org/protobuf/encoding/protojson"
	"google.golang.org/protobuf/proto"
	epb "google.golang.org/protobuf/types/known/emptypb"

	adminpb "intrinsic/kubernetes/vmpool/manager/api/v1/admin_api_go_proto"
	leaseapigrpcpb "intrinsic/kubernetes/vmpool/manager/api/v1/lease_api_go_proto"
)

var debugCmd = cobrautil.ParentOfNestedSubcommands("debug", "Internal debug commands")

func init() {
	debugCmd.AddCommand(vmpoolPingCmd)
	debugCmd.AddCommand(NewEditCommand())

	metricsCmd.Flags().StringVar(&flagPool, "pool", "", poolFlagDesc)
	debugCmd.AddCommand(metricsCmd)

	vmAddInstancesCmd.Flags().StringVar(&flagPool, "pool", "", poolFlagDesc)
	vmAddInstancesCmd.MarkFlagRequired("pool")
	vmAddInstancesCmd.Flags().IntVar(&flagCount, "count", 0, "Number of instances to add.")
	debugCmd.AddCommand(vmAddInstancesCmd)
	vmRemoveInstancesCmd.Flags().StringVar(&flagPool, "pool", "", poolFlagDesc)
	vmRemoveInstancesCmd.MarkFlagRequired("pool")
	vmRemoveInstancesCmd.Flags().IntVar(&flagCount, "count", 0, "Number of instances to remove.")
	debugCmd.AddCommand(vmRemoveInstancesCmd)

	PoolCmd.AddCommand(debugCmd)
}

func newAdminClient(ctx context.Context) (adminpb.VMPoolAdminServiceClient, error) {
	conn, err := newConn(ctx)
	if err != nil {
		return nil, err
	}
	return adminpb.NewVMPoolAdminServiceClient(conn), nil
}

var vmpoolPingCmd = &cobra.Command{
	Use: "ping", Short: "[Intrinsic Internal] Test connectivity to VM Pool Manager.",
	Long: "Test connectivity to VM Pool Manager.",
	RunE: func(cmd *cobra.Command, args []string) error {
		ctx := cmd.Context()
		cl, err := newLeaseClient(ctx)
		if err != nil {
			log.ExitContextf(ctx, "could not create lease client: %v", err)
		}
		return Ping(ctx, cl)
	},
}

// Ping pings the VM Pool Manager.
func Ping(ctx context.Context, cl leaseapigrpcpb.VMPoolLeaseServiceClient) error {
	ts := time.Now()
	resp, err := cl.Ping(ctx, &epb.Empty{})
	if err != nil {
		return fmt.Errorf("ping failed after %.3fs: %v", time.Since(ts).Seconds(), err)
	}
	fmt.Printf("Ping response %q in %.3fs\n", resp.GetMsg(), time.Since(ts).Seconds())
	return nil
}

func protoPrint(p proto.Message) {
	ms, err := protojson.MarshalOptions{
		Multiline:         true,
		UseProtoNames:     true,
		EmitUnpopulated:   true,
		EmitDefaultValues: true,
	}.Marshal(p)
	if err != nil {
		fmt.Println(err)
	}
	fmt.Println(string(ms))
}

var metricsDesc = `
Show metrics for a pool.

Example:
	inctl vm pool metrics --pool vmpool-testpool-1 --org intrinsic-dev
`

var metricsCmd = &cobra.Command{
	Use:   "metrics",
	Short: "[Intrinsic Internal] Show metrics for a pool.",
	Long:  metricsDesc,
	RunE: func(cmd *cobra.Command, args []string) error {
		if flagPool == "" {
			log.Exitf("--pool is required")
		}
		ctx, span := trace.StartSpan(cmd.Context(), "inctl.vmpool.metrics")
		span.AddAttributes(trace.StringAttribute("pool", flagPool))
		span.AddAttributes(trace.StringAttribute("org", viperLocal.GetString(orgutil.KeyOrganization)))
		defer span.End()
		cl, err := newAdminClient(ctx)
		if err != nil {
			log.ExitContextf(ctx, "could not create admin client: %v", err)
		}

		return ListMetrics(ctx, cl, flagPool)
	},
}

// ListMetrics lists metrics for a pool.
func ListMetrics(ctx context.Context, cl adminpb.VMPoolAdminServiceClient, poolName string) error {
	ps, err := cl.ListPools(ctx, &adminpb.ListPoolsRequest{})
	if err != nil {
		return fmt.Errorf("listing all pools failed: %v", err)
	}
	var matches []string
	for _, p := range ps.GetPools() {
		if p.GetName() == poolName || p.GetBaseName() == poolName {
			matches = append(matches, p.GetName())
		}
	}

	for _, match := range matches {
		r, err := cl.GetPoolStats(ctx, &adminpb.GetPoolStatsRequest{Name: match})
		if err != nil {
			return fmt.Errorf("get pool stats failed with: %v", err)
		}
		protoPrint(r)
	}

	if len(matches) == 0 {
		fmt.Printf("no pool found with name %q or base name %q\n", poolName, poolName)
	}
	return nil
}

// resolvePoolName resolves a pool name or basename into a single full pool resource name.
// Standard two-pass resolution:
// 1. Pass 1: Priority check for exact match on full pool name (p.GetName() == trimmedInput).
// resolvePoolName resolves a pool name or basename into a single full pool resource name.
// Standard resolution: Priority check for exact match on full pool name, otherwise collect basename matches.
// Returns error if input is empty, if ListPools fails, if 0 pools match, or if >1 pools match base_name.
func resolvePoolName(ctx context.Context, cl adminpb.VMPoolAdminServiceClient, poolInput string) (string, error) {
	trimmed := strings.TrimSpace(poolInput)
	if trimmed == "" {
		return "", fmt.Errorf("pool name cannot be empty")
	}

	ps, err := cl.ListPools(ctx, &adminpb.ListPoolsRequest{})
	if err != nil {
		return "", fmt.Errorf("failed to list pools for resolution: %w", err)
	}

	var basenameMatches []string
	for _, p := range ps.GetPools() {
		if p.GetName() == trimmed {
			return p.GetName(), nil
		}
		if p.GetBaseName() == trimmed {
			basenameMatches = append(basenameMatches, p.GetName())
		}
	}

	if len(basenameMatches) == 0 {
		return "", fmt.Errorf("pool %q not found", trimmed)
	}
	if len(basenameMatches) > 1 {
		return "", fmt.Errorf("multiple pools found for basename %q: %v", trimmed, basenameMatches)
	}

	return basenameMatches[0], nil
}

var vmAddInstancesCmd = &cobra.Command{
	Use:   "add-instances",
	Short: "[Intrinsic Internal] Add instances to a pool.",
	Long: `
	Add instances to a pool.

	The instances are not permanently added to the pool, but are one-off additions to the pool. Be aware
	that no validation is performed on the total number of instances in the pool or the number of concurrent
	provisioning instances. However, there is a server-side limit on the number of instances which can be
	added within a single RPC.
	`,
	RunE: func(cmd *cobra.Command, args []string) error {
		ctx, span := trace.StartSpan(cmd.Context(), "inctl.vm.add-instances")
		span.AddAttributes(trace.StringAttribute("pool", flagPool))
		span.AddAttributes(trace.StringAttribute("org", viperLocal.GetString(orgutil.KeyOrganization)))
		defer span.End()
		cl, err := newAdminClient(ctx)
		if err != nil {
			return fmt.Errorf("could not create admin client: %v", err)
		}
		return AddInstances(ctx, cl, flagPool, flagCount)
	},
}

// AddInstances adds instances to a pool.
func AddInstances(ctx context.Context, cl adminpb.VMPoolAdminServiceClient, poolName string, count int) error {
	resolvedPool, err := resolvePoolName(ctx, cl, poolName)
	if err != nil {
		return fmt.Errorf("could not resolve pool name: %w", err)
	}
	r := &adminpb.AddInstancesRequest{Name: resolvedPool, Count: int32(count)}
	if _, err := cl.AddInstances(ctx, r); err != nil {
		return fmt.Errorf("could not add instances: %v", err)
	}
	return nil
}

var vmRemoveInstancesCmd = &cobra.Command{
	Use:   "remove-instances",
	Short: "[Intrinsic Internal] Remove instances from a pool.",
	Long: `
	Remove instances from a pool.

	Removes up to the given number of unused instances from the given pool. An instance is considered
	unused if it is in the Ready or Provisioning phase. Does not remove instances which are leased or
	are in the decommission phase.
	`,
	RunE: func(cmd *cobra.Command, args []string) error {
		if err := agents.Check(cmd); err != nil {
			return err
		}
		ctx, span := trace.StartSpan(cmd.Context(), "inctl.vm.remove-instances")
		span.AddAttributes(trace.StringAttribute("pool", flagPool))
		span.AddAttributes(trace.StringAttribute("org", viperLocal.GetString(orgutil.KeyOrganization)))
		defer span.End()
		cl, err := newAdminClient(ctx)
		if err != nil {
			return fmt.Errorf("could not create admin client: %v", err)
		}
		return RemoveInstances(ctx, cl, flagPool, flagCount)
	},
}

// RemoveInstances removes instances from a pool.
func RemoveInstances(ctx context.Context, cl adminpb.VMPoolAdminServiceClient, poolName string, count int) error {
	resolvedPool, err := resolvePoolName(ctx, cl, poolName)
	if err != nil {
		return fmt.Errorf("could not resolve pool name: %w", err)
	}
	r := &adminpb.RemoveInstancesRequest{Name: resolvedPool, Count: int32(count)}
	if _, err := cl.RemoveInstances(ctx, r); err != nil {
		return fmt.Errorf("could not remove instances: %v", err)
	}
	return nil
}
