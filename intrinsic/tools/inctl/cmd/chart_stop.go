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

// Package chartstop contains the inctl chart stop command.
package chartstop

import (
	"fmt"
	"strings"

	"intrinsic/kubernetes/chartapply"
	"intrinsic/kubernetes/workcell_spec/chartassignment"
	"intrinsic/tools/inctl/util/gcpauth"

	"github.com/spf13/cobra"
	"github.com/spf13/pflag"
)

// The FParseErrWhitelist struct allows to ignore unknown arguments.
type FParseErrWhitelist pflag.ParseErrorsWhitelist

// Command returns the inctl chart stop subcommand.
func Command() *cobra.Command {
	var (
		flagContext string
		flagDryRun  bool
	)
	cmd := &cobra.Command{
		Use:   "stop [chart name]",
		Short: "Stop a running chart",
		Long:  "Stop a chart running in a Kubernetes cluster.",
		Args:  cobra.MinimumNArgs(1),
		FParseErrWhitelist: cobra.FParseErrWhitelist{
			UnknownFlags: true,
		},
		RunE: func(cmd *cobra.Command, args []string) error {
			ctx := cmd.Context()
			target := args[0]

			opts, err := gcpauth.ClientHTTPOptions(ctx)
			if err != nil {
				return fmt.Errorf("gcs client options: %w", err)
			}

			clusterConfig, err := chartapply.GetClusterConfig(ctx, opts, flagContext)
			if err != nil {
				return fmt.Errorf("get cluster config: %w", err)
			}

			argoCDManaged, err := chartapply.IsArgoCDManagedCluster(ctx, clusterConfig)
			if err != nil {
				fmt.Printf("Failed to determine if cluster is ArgoCD managed, assuming it is not: %s\n", err)
			}

			if argoCDManaged {
				fmt.Printf("Cluster is ArgoCD managed, deleting chart resources directly.\n")

				// TODO (b/454090421): load the chart name from the runfiles. This is not trivial because the bazel run //chartpath:chart.stop command accesses a different set of runfiles. This may be a problem for insrc charts.
				chartName := strings.Split(chartassignment.NormalizedTargetName(target), ":")[1]

				// Delete the chart from the cluster.
				err = chartapply.DeleteChart(ctx, chartName, clusterConfig, flagDryRun, "")
				if err != nil {
					return fmt.Errorf("failed to delete chart: %w", err)
				}
				return nil
			}
			// If not ArgoCD managed, use the ChartAssignment route.
			cs := clusterConfig.AppsCS
			k8s := clusterConfig.KubeCS

			fmt.Println("Deleting chart...")
			name, err := chartassignment.ChartName(cmd.Context(), cs, target)
			if err != nil {
				return fmt.Errorf("delete chartassignment: %w", err)
			}

			if err := chartassignment.DeleteAndWait(cmd.Context(), cs, k8s, name); err != nil {
				return err
			}
			fmt.Printf("Deleted app %q.\n", name)
			return nil
		},
	}

	cmd.PersistentFlags().StringVarP(&flagContext, "context", "c", "", "The Kubernetes cluster to use.")
	cmd.Flags().BoolVar(&flagDryRun, "dry_run", false, "Do not delete the chart from the cluster.")
	cmd.MarkPersistentFlagRequired("context")
	return cmd
}
