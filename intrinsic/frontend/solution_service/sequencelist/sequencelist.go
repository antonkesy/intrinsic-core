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

// Package sequencelist implements tooling to generate a sequence list from log
// items of a Process execution.
package sequencelist

import (
	"context"
	"errors"
	"slices"

	log "github.com/golang/glog"
	"google.golang.org/protobuf/types/known/timestamppb"

	"intrinsic/executive/go/behaviortree"
	behaviortreepb "intrinsic/executive/proto/behavior_tree_go_proto"
	executivelogitemspb "intrinsic/executive/proto/log_items_go_proto"
	runmetadatapb "intrinsic/executive/proto/run_metadata_go_proto"
	sequencelistpb "intrinsic/frontend/solution_service/proto/sequence_list_go_proto"
	"intrinsic/frontend/solution_service/sequencelist/find"
	logitempb "intrinsic/logging/proto/log_item_go_proto"
)

// SequenceList is a partial or complete list of sequence list items.
type SequenceList struct {
	Items                    []*sequencelistpb.SequenceListItem
	lastConsumedLogItemIndex int
	totalLogItems            int
}

// NextUnconsumedLogItemIndex returns the 0-based index of the first log item in
// the input batch that was NOT consumed by the built sequence list items.
// Returns -1 if all log items were consumed or if no items were processed.
func (s *SequenceList) NextUnconsumedLogItemIndex() int {
	if s == nil || s.lastConsumedLogItemIndex < 0 || s.lastConsumedLogItemIndex >= s.totalLogItems-1 {
		return -1
	}
	return s.lastConsumedLogItemIndex + 1
}

type buildOptions struct {
	maxRootItems int
}

// BuildOption to allow customizing sequence list building.
type BuildOption func(*buildOptions)

// WithMaxRootItems controls how many root items the sequence list will contain
// at most. The default of 0 (or any negative number) means that no limit is
// applied.
func WithMaxRootItems(maxRootItems int) BuildOption {
	return func(opts *buildOptions) {
		opts.maxRootItems = maxRootItems
	}
}

// BuildSequenceList builds a sequence list from log items. The resulting items
// are sorted by the start time of each action, i.e. the acquisition time of the
// first log item for a certain action ID.
func BuildSequenceList(ctx context.Context, items []*logitempb.LogItem, opts ...BuildOption) (*SequenceList, error) {
	buildOpts := &buildOptions{}
	for _, opt := range opts {
		opt(buildOpts)
	}

	actions := make(map[uint64]*actionData)
	var actionIDs []uint64

	for i, item := range items {
		actionID := item.GetContext().GetExecutivePlanActionId()
		if actionID == 0 {
			continue
		}

		data, ok := actions[actionID]
		if !ok {
			data = &actionData{
				id:        actionID,
				startTime: item.GetMetadata().GetAcquisitionTime(),
			}
			actions[actionID] = data
			actionIDs = append(actionIDs, actionID)
		}

		data.endTime = item.GetMetadata().GetAcquisitionTime()
		data.latestOp = item.GetPayload().GetExecutiveOperation()
		data.lastLogItemIdx = i
	}

	// Sort action IDs by start time to maintain execution order.
	slices.SortFunc(actionIDs, func(a, b uint64) int {
		return actions[a].startTime.AsTime().Compare(actions[b].startTime.AsTime())
	})

	// Map to store tree_id -> parent action data slice
	treeToParents := make(map[string][]*actionData)
	idToItem := make(map[uint64]*sequencelistpb.SequenceListItem)

	for _, id := range actionIDs {
		data := actions[id]
		item, calledTreeID := buildItem(ctx, data)
		if item != nil {
			idToItem[id] = item
			if calledTreeID != "" {
				treeToParents[calledTreeID] = append(treeToParents[calledTreeID], data)
			}
		}
	}

	var rootItems []*sequencelistpb.SequenceListItem
	for _, id := range actionIDs {
		item := idToItem[id]
		if item == nil {
			continue
		}

		data := actions[id]
		nodeID := data.latestOp.GetBehaviorTreeContext().GetActionNodeIdentifier()
		if nodeID == nil {
			rootItems = append(rootItems, item)
			continue
		}

		parentID, ok := findParentID(treeToParents, nodeID.GetTreeId(), data)
		if ok {
			parent := idToItem[parentID]
			parent.Children = append(parent.Children, item)
		} else {
			rootItems = append(rootItems, item)
		}
	}

	actionLastIdx := make(map[uint64]int, len(actions))
	for id, a := range actions {
		actionLastIdx[id] = a.lastLogItemIdx
	}

	res := &SequenceList{
		lastConsumedLogItemIndex: -1,
		totalLogItems:            len(items),
	}

	if buildOpts.maxRootItems > 0 && len(rootItems) > buildOpts.maxRootItems {
		res.Items = rootItems[:buildOpts.maxRootItems]
	} else {
		res.Items = rootItems
	}

	for _, root := range res.Items {
		idx := maxLogItemIndex(root, actionLastIdx)
		if idx > res.lastConsumedLogItemIndex {
			res.lastConsumedLogItemIndex = idx
		}
	}

	if res.lastConsumedLogItemIndex == -1 && len(items) > 0 {
		res.lastConsumedLogItemIndex = len(items) - 1
	}

	return res, nil
}

func maxLogItemIndex(item *sequencelistpb.SequenceListItem, actionLastIdx map[uint64]int) int {
	maxIdx := actionLastIdx[item.GetId()]
	for _, child := range item.GetChildren() {
		childMax := maxLogItemIndex(child, actionLastIdx)
		if childMax > maxIdx {
			maxIdx = childMax
		}
	}
	return maxIdx
}

func findParentID(treeToParents map[string][]*actionData, treeID string, childData *actionData) (uint64, bool) {
	parents := treeToParents[treeID]
	if len(parents) == 0 {
		return 0, false
	}
	childStart := childData.startTime.AsTime()
	for i := len(parents) - 1; i >= 0; i-- {
		pData := parents[i]
		if pData.id == childData.id {
			continue
		}
		pStart := pData.startTime.AsTime()
		if childStart.Before(pStart) {
			continue
		}
		if pData.endTime != nil && childStart.After(pData.endTime.AsTime()) {
			continue
		}
		return pData.id, true
	}
	return 0, false
}

type actionData struct {
	id             uint64
	startTime      *timestamppb.Timestamp
	endTime        *timestamppb.Timestamp
	latestOp       *executivelogitemspb.LoggedOperation
	lastLogItemIdx int
}

func buildItem(ctx context.Context, data *actionData) (*sequencelistpb.SequenceListItem, string) {
	op := data.latestOp
	if op == nil {
		return nil, ""
	}

	rmAny := op.GetOperation().GetMetadata()
	if rmAny == nil {
		return nil, ""
	}

	var runMetadata runmetadatapb.RunMetadata
	if err := rmAny.UnmarshalTo(&runMetadata); err != nil {
		log.Errorf("failed to unmarshal run metadata: %v", err)
		return nil, ""
	}

	bt := runMetadata.GetBehaviorTree()
	if bt == nil {
		return nil, ""
	}

	nodeID := op.GetBehaviorTreeContext().GetActionNodeIdentifier()
	if nodeID == nil {
		return nil, ""
	}

	node, err := find.Node(nodeID.GetTreeId(), nodeID.GetNodeId()).InBehaviorTree(ctx, bt)
	if err != nil {
		return nil, ""
	}

	task := node.GetTask()
	if task == nil {
		return nil, ""
	}

	item := &sequencelistpb.SequenceListItem{
		Id:        data.id,
		Name:      node.GetName(),
		StartTime: data.startTime,
		EndTime:   data.endTime,
		State:     task.GetState(),
	}

	item.State = task.GetState()
	switch t := task.GetTaskType().(type) {
	case *behaviortreepb.BehaviorTree_TaskNode_CallBehavior:
		item.Details = &sequencelistpb.SequenceListItem_SkillDetails{
			SkillDetails: &sequencelistpb.SkillDetails{
				SkillId: t.CallBehavior.GetSkillId(),
			},
		}
	case *behaviortreepb.BehaviorTree_TaskNode_ExecuteCode:
		item.Details = &sequencelistpb.SequenceListItem_CodeExecutionDetails{
			CodeExecutionDetails: &sequencelistpb.CodeExecutionDetails{
				Stdout: t.ExecuteCode.GetStdout(),
			},
		}
	}
	var calledTreeID string
	if sub := task.GetCalledTreeState(); sub != nil {
		calledTreeID = sub.GetTreeId()
	}

	return item, calledTreeID
}

type nodeFindingVisitor struct {
	targetTreeID string
	targetNodeID uint32
}

type nodeFound struct {
	node *behaviortreepb.BehaviorTree_Node
}

func (e *nodeFound) Error() string {
	return "node found"
}

func (f *nodeFindingVisitor) Visit(ctx context.Context, element behaviortree.VisitElement) error {
	if node := element.Node(); node != nil && node.GetId() == f.targetNodeID {
		// We must traverse up the ancestor chain to validate that this node is
		// within the specified tree ID.
		for ancestor := range element.Ancestors() {
			if tree := ancestor.Tree(); tree != nil {
				if tree.GetTreeId() == f.targetTreeID {
					return &nodeFound{node}
				}
				// The first tree we encounter in the ancestor chain would need to hold
				// the tree ID. If not, this is not the node we're looking for. We can
				// skip further ancestors.
				break
			}
		}
	}
	return nil
}

func findNode(ctx context.Context, tree *behaviortreepb.BehaviorTree, treeID string, nodeID uint32) *behaviortreepb.BehaviorTree_Node {
	visitor := &nodeFindingVisitor{
		targetTreeID: treeID,
		targetNodeID: nodeID,
	}
	if err := behaviortree.Walk(ctx, tree, visitor, behaviortree.VisitCalledTreeState()); err != nil {
		var nodeFoundErr *nodeFound
		if errors.As(err, &nodeFoundErr) {
			return nodeFoundErr.node
		}
	}
	return nil
}
