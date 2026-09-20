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

package logs

import (
	"context"
	"time"

	"intrinsic/tools/inctl/auth/auth"

	"github.com/gdamore/tcell/v2"
	log "github.com/golang/glog"
	"github.com/pkg/errors"
	"github.com/rivo/tview"
	"github.com/spf13/cobra"
	"google.golang.org/protobuf/encoding/prototext"

	grpcpb "intrinsic/logging/proto/logger_service_go_proto"
	pb "intrinsic/logging/proto/logger_service_go_proto"

	emptypb "google.golang.org/protobuf/types/known/emptypb"

	// Required for pretty-printing.
	_ "intrinsic/hardware/gripper/gripper_go_proto"
	// Required for pretty-printing.
	_ "intrinsic/icon/equipment/icon_equipment_go_proto"
	// Required for pretty-printing.
	_ "intrinsic/simulation/gazebo/proto/introspection_go_proto"
)

var redacted = []byte("REDACTED")

var logsHistorianCmd = &cobra.Command{
	Use:   "historian",
	Short: "TUI for the historian",
	Long:  "TUI for the historian",
	RunE: func(cmd *cobra.Command, args []string) error {
		ctx := cmd.Context()

		clusterName, err := getClusterName(ctx, &cmdParams{
			projectName: cmdFlags.GetFlagProject(),
			org:         cmdFlags.GetFlagOrganization(),
			context:     flagContext,
		})
		if err != nil {
			return errors.Wrap(err, "could not resolve cluster name")
		}

		conn, err := auth.NewCloudConnection(ctx, auth.WithFlagValues(localViper), auth.WithCluster(clusterName))
		if err != nil {
			return errors.Wrap(err, "failed to create cloud connection")
		}
		defer conn.Close()

		client := grpcpb.NewDataLoggerClient(conn)

		layout := tview.NewFlex().SetDirection(tview.FlexColumn)
		topicList := tview.NewList()
		topicList.ShowSecondaryText(false).SetTitle("Topics").SetBorder(true)
		topicBox := tview.NewFlex().SetDirection(tview.FlexRow).AddItem(topicList, 0, 40, true)
		layout.AddItem(topicBox, 0, 1, true)
		messageBox := tview.NewFlex().SetDirection(tview.FlexRow)
		messageText := tview.NewTextView()
		messageText.SetScrollable(true).SetTitle("Message Stream").SetBorder(true)
		messageBox.AddItem(messageText, 0, 1, false)
		layout.AddItem(messageBox, 0, 4, true)

		topicChan := make(chan string, 1)
		topicList.SetChangedFunc(func(i int, main string, sec string, r rune) {
			topicChan <- main
		})
		response, err := client.ListLogSources(ctx, &emptypb.Empty{})
		if err != nil {
			log.ErrorContext(ctx, err)
			return errors.Wrap(err, "ListLogSources failed")
		}
		for _, topic := range response.EventSources {
			topicList.AddItem(topic, "", 0, nil)
		}
		app := tview.NewApplication().SetRoot(layout, true)
		go func() {
			topic := ""
			for {
				select {
				case topic = <-topicChan:
				case <-time.After(500 * time.Millisecond):
					if topic == "" {
						continue
					}
					response, err := client.GetMostRecentItem(context.Background(), &pb.GetMostRecentItemRequest{
						EventSource: topic,
					})
					messageToShow := ""
					if err == nil {
						if response.Item.BlobPayload != nil && len(response.Item.BlobPayload.Data) != 0 {
							response.Item.BlobPayload.Data = redacted
						}
						messageToShow = prototext.Format(response.Item)
					} else {
						messageToShow = err.Error()
					}
					app.QueueUpdateDraw(func() {
						messageText.SetText(messageToShow)
					})
				}
			}
		}()
		topicList.SetSelectedFunc(func(i int, main string, sec string, r rune) {
			app.SetFocus(messageText)
		})
		app.SetInputCapture(func(event *tcell.EventKey) *tcell.EventKey {
			if event.Key() == tcell.KeyEsc {
				app.SetFocus(topicList)
			}
			return event
		})

		if err := app.Run(); err != nil {
			log.FatalContext(ctx, err)
		}

		return nil
	},
}

func init() {
	showLogs.AddCommand(logsHistorianCmd)
	logsHistorianCmd.Flags().StringVarP(&flagContext, "context", "c", "", "The Kubernetes cluster to use.")
	logsHistorianCmd.MarkFlagRequired("context")
}
