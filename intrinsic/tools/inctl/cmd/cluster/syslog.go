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

package cluster

import (
	"context"
	"fmt"
	"os"
	"text/tabwriter"

	"intrinsic/tools/inctl/util/orgutil"

	"github.com/spf13/cobra"

	clustermanagerpb "github.com/intrinsic-ai/insrc/incode/frontend/cloud/api/v1/clustermanager_api_go_proto"
)

type syslogServer struct {
	host      string
	port      int
	transport string
	format    string
}

func transportPB(s syslogServer) clustermanagerpb.LogReceiver_Transport {
	switch s.transport {
	case "udp":
		return clustermanagerpb.LogReceiver_TRANSPORT_UDP
	case "tcp":
		return clustermanagerpb.LogReceiver_TRANSPORT_TCP
	default:
		return clustermanagerpb.LogReceiver_TRANSPORT_DEFAULT
	}
}

func transportString(s clustermanagerpb.LogReceiver_Transport) string {
	switch s {
	case clustermanagerpb.LogReceiver_TRANSPORT_UDP:
		return "udp"
	case clustermanagerpb.LogReceiver_TRANSPORT_TCP:
		return "tcp"
	default:
		return "default"
	}
}

func formatPB(s syslogServer) clustermanagerpb.LogReceiver_Format {
	switch s.format {
	case "rfc5424":
		return clustermanagerpb.LogReceiver_FORMAT_RFC5424
	case "rfc3164":
		return clustermanagerpb.LogReceiver_FORMAT_RFC3164
	default:
		return clustermanagerpb.LogReceiver_FORMAT_DEFAULT
	}
}

func formatString(s clustermanagerpb.LogReceiver_Format) string {
	switch s {
	case clustermanagerpb.LogReceiver_FORMAT_RFC5424:
		return "rfc5424"
	case clustermanagerpb.LogReceiver_FORMAT_RFC3164:
		return "rfc3164"
	default:
		return "default"
	}
}

var syslog syslogServer

var syslogCmd = &cobra.Command{
	Use:   "syslog",
	Short: "Show the current syslog configuration.",
	Long:  "Show the current syslog server configuration of the cluster.",
	Args:  cobra.NoArgs,
	RunE: func(cmd *cobra.Command, _ []string) error {
		ctx := cmd.Context()

		projectName := ClusterCmdViper.GetString(orgutil.KeyProject)
		orgName := ClusterCmdViper.GetString(orgutil.KeyOrganization)

		ctx, c, err := newClient(ctx, orgName, projectName, clusterName)
		if err != nil {
			return fmt.Errorf("device manager client:\n%w", err)
		}
		defer c.close()

		req := clustermanagerpb.GetLogReceiverRequest{
			ClusterId: clusterName,
		}
		resp, err := c.grpcClient.GetLogReceiver(ctx, &req)
		if err != nil {
			return err
		}
		if resp.GetHost() == "" {
			fmt.Println("No syslog server configured.")
			return nil
		}
		w := tabwriter.NewWriter(os.Stdout, 0, 0, 3, ' ', 0)
		fmt.Fprintf(w, "host\tport\ttransport\tformat\n")
		fmt.Fprintf(w, "%s\t%d\t%s\t%s\n",
			resp.GetHost(), resp.GetPort(), transportString(resp.GetTransport()), formatString(resp.GetFormat()))
		w.Flush()
		return nil
	},
}

func setLogReceiver(ctx context.Context, org, project, cluster string, req *clustermanagerpb.CreateLogReceiverUpdateRequest) error {
	ctx, c, err := newClient(ctx, org, project, cluster)
	if err != nil {
		return fmt.Errorf("cluster upgrade client:\n%w", err)
	}
	defer c.close()

	_, err = c.grpcClient.CreateLogReceiverUpdate(ctx, req)
	return err
}

var syslogEnableCmd = &cobra.Command{
	Use:   "enable",
	Short: "Enable sending syslog messages.",
	Long:  "Configure which syslog server should receive log messages from the cluster.",
	Args:  cobra.NoArgs,
	RunE: func(cmd *cobra.Command, _ []string) error {
		projectName := ClusterCmdViper.GetString(orgutil.KeyProject)
		orgName := ClusterCmdViper.GetString(orgutil.KeyOrganization)
		req := &clustermanagerpb.CreateLogReceiverUpdateRequest{
			ClusterId: clusterName,
			LogReceiver: &clustermanagerpb.LogReceiver{
				Host:      syslog.host,
				Port:      int32(syslog.port),
				Transport: transportPB(syslog),
				Format:    formatPB(syslog),
			},
		}
		return setLogReceiver(cmd.Context(), orgName, projectName, clusterName, req)
	},
}

var syslogDisableCmd = &cobra.Command{
	Use:   "disable",
	Short: "Disable sending syslog messages.",
	Long:  "Configure the cluster to not send syslog messages to a syslog server.",
	Args:  cobra.NoArgs,
	RunE: func(cmd *cobra.Command, _ []string) error {
		projectName := ClusterCmdViper.GetString(orgutil.KeyProject)
		orgName := ClusterCmdViper.GetString(orgutil.KeyOrganization)
		req := &clustermanagerpb.CreateLogReceiverUpdateRequest{
			ClusterId: clusterName,
		}
		return setLogReceiver(cmd.Context(), orgName, projectName, clusterName, req)
	},
}

func init() {
	ClusterCmd.AddCommand(syslogCmd)
	syslogCmd.PersistentFlags().StringVar(&clusterName, "cluster", "", "Name of cluster to upgrade.")
	syslogCmd.MarkPersistentFlagRequired("cluster")
	syslogCmd.AddCommand(syslogDisableCmd)
	syslogCmd.AddCommand(syslogEnableCmd)
	syslogEnableCmd.Flags().StringVar(&syslog.host, "host", "", "Address of the receiving syslog server.")
	syslogEnableCmd.MarkFlagRequired("host")
	syslogEnableCmd.Flags().IntVar(&syslog.port, "port", 514, "Port of the receiving syslog server.")
	syslogEnableCmd.Flags().StringVar(&syslog.transport, "transport", "tcp", "Transport to use for the syslog connection. Valid values are tcp and udp.")
	syslogEnableCmd.Flags().StringVar(&syslog.format, "format", "rfc5424", "Format to use for the syslog connection. Valid values are rfc5424 and rfc3164.")
}
