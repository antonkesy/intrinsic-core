; Copyright 2026 Intrinsic Innovation LLC
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     https://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

; State proto updates.
;
; Rules to update the RunMetadata state proto on state updates.

; Requires:
; - behavior_tree.clp: general behavior tree access
; - saliences.clp: salience globals

; --------------------------------- FUNCTIONS ---------------------------------

(deffunction run-metadata-proto-update-field (?run-metadata-proto-path ?value ?operation-name)
  (do-for-fact ((?op operation-envelope)) (eq ?op:name ?operation-name)
    (if (<> ?op:run-metadata-proto 0) then
      (pb-set-field ?op:run-metadata-proto
                    ?run-metadata-proto-path
                    ?value)
    )
  )
)

(deffunction run-metadata-proto-clear-field (?run-metadata-proto-path ?operation-name)
  (do-for-fact ((?op operation-envelope)) (eq ?op:name ?operation-name)
    (if (<> ?op:run-metadata-proto 0) then
      (pb-clear-field ?op:run-metadata-proto
                      ?run-metadata-proto-path)
    )
  )
)

(deffunction run-metadata-proto-set-map-value
    (?run-metadata-proto-path ?key ?value ?operation-name)
  (do-for-fact ((?op operation-envelope)) (eq ?op:name ?operation-name)
    (if (<> ?op:run-metadata-proto 0) then
      (pb-set-map-value ?op:run-metadata-proto
                        ?run-metadata-proto-path
                        ?key ?value)
    )
  )
)

; Clears decorators field in state proto if no decorators set.
;
; Args:
;   ?node: node to check and clear
;   ?state-proto: associate tree's state proto
;   ?operation-name: Name of the operation
(deffunction run-metadata-proto-clear-unused-decorators (?node ?operation-name)
  (bind ?run-metadata-proto-path (fact-slot-value ?node run-metadata-proto-path))
  (if (neq ?run-metadata-proto-path "") then
    (do-for-fact ((?op operation-envelope)) (eq ?op:name ?operation-name)
      (if (<> ?op:run-metadata-proto 0) then
        (bind ?s-decorators-path (proto-path-join ?run-metadata-proto-path "decorators"))
        (bind ?decorators-proto (pb-get-field ?op:run-metadata-proto ?s-decorators-path))
        (if (pb-is-default ?decorators-proto) then
          (run-metadata-proto-clear-field ?s-decorators-path ?operation-name)
        )
        (pb-remove ?decorators-proto)
      )
    )
  )
)

(deffunction run-metadata-proto-sync-task-resources
  (?s-path-call-behavior ?parameter-proto ?operation-name)
  (bind ?s-path-resources-current
    (proto-path-join ?s-path-call-behavior "resources_current"))
  (run-metadata-proto-clear-field ?s-path-resources-current ?operation-name)

  (foreach ?resource-key
    (pb-get-map-keys ?parameter-proto "resources")
    (bind ?resource-specification-proto
      (pb-get-map-value ?parameter-proto "resources" ?resource-key))
    (bind ?resource-specification-proto-type (pb-which-oneof
      ?resource-specification-proto "resource_specification_type"))
    (switch ?resource-specification-proto-type
      (case handle then
        (run-metadata-proto-set-map-value ?s-path-resources-current
          ?resource-key
          (pb-get-field ?resource-specification-proto "handle") ?operation-name)
      )
      (case reference then
        (printout error (str-cat "BehaviorTree is executing task node "
          "with unassigned reference: " ?resource-key) crlf)
      )
    )
    (pb-remove ?resource-specification-proto)
  )
)

(deffunction run-metadata-proto-sync-task-skill-data
  (?s-path-call-behavior ?parameter-proto ?operation-name)
  ; Check this first to prevent an error log when returning MISSING-FIELD.
  ; This is often expected and normal here.
  (if (pb-has-field ?parameter-proto "skill_execution_data") then
    (bind ?skill-execution-proto
      (pb-get-field ?parameter-proto "skill_execution_data"))
    (if (neq ?skill-execution-proto MISSING-FIELD) then
      (run-metadata-proto-update-field
        (proto-path-join ?s-path-call-behavior "skill_execution_data")
        ?skill-execution-proto ?operation-name)
      (pb-remove ?skill-execution-proto)
    )
  )
)

; Resets execution state fields in the RunMetadata proto when a task node is reset.
;
; Args:
;   ?run-metadata-proto-path: Proto path for this task node
;   ?task-type: Task type (CALL-BEHAVIOR or EXECUTE-CODE)
;   ?operation-name: Name of the operation
(deffunction run-metadata-proto-reset-task-node-execution-info
  (?run-metadata-proto-path ?task-type ?operation-name)
  (switch ?task-type
    (case CALL-BEHAVIOR then
      (bind ?s-path-call-behavior
        (proto-path-join ?run-metadata-proto-path "task.call_behavior"))
      (run-metadata-proto-clear-field
        (proto-path-join ?s-path-call-behavior "resources_current")
        ?operation-name)
      (run-metadata-proto-clear-field
        (proto-path-join ?s-path-call-behavior "skill_execution_data")
        ?operation-name)
    )
    (case EXECUTE-CODE then
      (run-metadata-proto-clear-field
        (proto-path-join ?run-metadata-proto-path "task.execute_code.stdout")
        ?operation-name)
    )
  )
)

; ----------------------------------- RULES -----------------------------------

(defrule behavior-tree-state-proto-update-tree
  (declare (salience ?*SALIENCE-HIGHER*))
  ?tree <- (behavior-tree (id ?tree-id) (operation-name ?op)
                          (state ?state) (run-metadata-proto-state ?proto-state&~?state)
                          (run-metadata-proto-path ?run-metadata-proto-path))
  (or
   (behavior-tree (id ?tree-id) (parent-behavior-tree-id nil) (run-metadata-proto-path ""|"behavior_tree"))
   (behavior-tree (id ?tree-id) (parent-behavior-tree-id ~nil) (run-metadata-proto-path ~""))
  )
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "state"))
  (run-metadata-proto-update-field ?path ?state ?op)

  (modify ?tree (run-metadata-proto-state ?state))
)

(defrule behavior-tree-state-proto-update-node
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  ?node <- (behavior-tree-node (tree-id ?tree-id) (state ?state)
                               (failure-reason ?failure-reason)
                               (run-metadata-proto-state ?proto-state&~?state)
                               (run-metadata-proto-recovered-state
                                 ?run-metadata-proto-recovered-state)
                               (run-metadata-proto-path ?run-metadata-proto-path&~""))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "state"))
  (bind ?s-state (sym-cat (str-replace-all ?state "-" "_")))
  (if (eq ?s-state CANCELING_CONDITION) then (bind ?s-state CANCELING))
  (run-metadata-proto-update-field ?path ?s-state ?op)

  (if (neq ?run-metadata-proto-recovered-state NONE) then
    (bind ?s-path-recovered (proto-path-join ?run-metadata-proto-path "recovered"))
    (if (eq ?state ?run-metadata-proto-recovered-state)
      then
        ; Update from recovery
        (run-metadata-proto-update-field ?s-path-recovered TRUE ?op)
      else
        ; State was recovered before, but now the node is executing to a
        ; different state. Thus clear the recovered flag as its now based on
        ; execution.
        (run-metadata-proto-update-field ?s-path-recovered FALSE ?op)
        (bind ?run-metadata-proto-recovered-state NONE)
    )
  )

  (if (neq ?failure-reason UNKNOWN) then
    (bind ?s-fr-path (proto-path-join ?run-metadata-proto-path "failure_reason"))
    (bind ?s-fr-state (sym-cat (str-cat "FAILED_" ?failure-reason)))
    (run-metadata-proto-update-field ?s-fr-path ?s-fr-state ?op)
  )

  (modify ?node (run-metadata-proto-state ?state)
                (run-metadata-proto-recovered-state ?run-metadata-proto-recovered-state))
)

(defrule behavior-tree-state-proto-update-condition
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  ?cond <- (behavior-tree-condition (tree-id ?tree-id) (state ?state)
                                    (satisfied ?satisfied)
                                    (run-metadata-proto-state ?proto-state&~?state)
                                    (run-metadata-proto-path ?run-metadata-proto-path&~""))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "state"))
  (bind ?s-satisfied-path (proto-path-join ?run-metadata-proto-path "satisfied"))
  (bind ?s-state ?state)
  (if (eq ?s-state START) then (bind ?s-state EVALUATING))
  (run-metadata-proto-update-field ?path ?s-state ?op)
  (if (eq ?s-state FINISHED)
   then (run-metadata-proto-update-field ?s-satisfied-path ?satisfied ?op))
  (modify ?cond (run-metadata-proto-state ?state))
)

(defrule behavior-tree-state-proto-update-task-node-action
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  (behavior-tree-node (tree-id ?tree-id) (type TASK)
                      (task-action-uid ?task-action-uid)
                      (run-metadata-proto-path ?run-metadata-proto-path&~""))
  ?action <- (plan-action (uid ?task-action-uid) (state ?state)
                          (run-metadata-proto-state ?proto-state&~?state))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "task.state"))
  (bind ?task-state (sym-cat (str-replace-all ?state "-" "_")))
  ; These states need to be mapped to proto-specific values
  (switch ?state
    (case FOOTPRINT-CHECKING then (bind ?task-state CHECKING_FOOTPRINT))
    (case FOOTPRINT-CONFLICT-WAITING then (bind ?task-state FOOTPRINT_CONFLICT_WAITING))
    (case CANCELLATION-REQUESTED then (bind ?task-state CANCELING))
    (case CANCELLATION-PENDING then (bind ?task-state CANCELING))
    (case EXECUTION-SUCCEEDED then (bind ?task-state RUNNING))
    (case EXECUTION-FAILED then (bind ?task-state RUNNING))
  )
  (run-metadata-proto-update-field ?path ?task-state ?op)
  (modify ?action (run-metadata-proto-state ?state))
)

(defrule behavior-tree-state-proto-update-task-node-behavior-instance
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  (behavior-tree-node (tree-id ?tree-id) (type TASK)
                      (task-type CALL-BEHAVIOR)
                      (behavior-call-instance-uid ?bci-uid)
                      (run-metadata-proto-path ?run-metadata-proto-path&~""))
  ?bci <- (behavior-call-instance (uid ?bci-uid) (state ?state)
                                  (run-metadata-proto-state ?proto-state&~?state))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "task.state"))
  (bind ?task-state ?state)
  ; These states need to be mapped to proto-specific values
  (switch ?state
    (case CANCELLATION-REQUESTED then (bind ?task-state CANCELING))
    ; TODO(timdn): update with suspend refactor
    (case SUSPENDING then (bind ?task-state RUNNING))
    (case SUSPENDED then (bind ?task-state RUNNING))
  )
  (run-metadata-proto-update-field ?path ?task-state ?op)
  (modify ?bci (run-metadata-proto-state ?state))
)

(defrule behavior-tree-state-proto-update-task-node-code-execution-instance
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  (behavior-tree-node (tree-id ?tree-id) (id ?node-id) (type TASK)
                      (task-type EXECUTE-CODE)
                      (run-metadata-proto-path ?run-metadata-proto-path&~""))
  ?cei <- (code-execution-instance (operation-name ?op) (tree-id ?tree-id)
                                   (node-id ?node-id)
                                   (state ?state)
                                   (run-metadata-proto-state ?proto-state&~?state))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "task.state"))
  (bind ?task-state ?state)
  ; These states need to be mapped to proto-specific values
  (switch ?state
    (case CANCELATION-REQUESTED then (bind ?task-state CANCELING))
    (case CANCELATION-PENDING then (bind ?task-state CANCELING))
  )
  (run-metadata-proto-update-field ?path ?task-state ?op)
  (modify ?cei (run-metadata-proto-state ?state))
)


(defrule behavior-tree-state-proto-update-task-node-code-execution-response-stdout
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  (behavior-tree-node (tree-id ?tree-id) (id ?node-id) (type TASK)
                      (task-type EXECUTE-CODE)
                      (run-metadata-proto-path ?run-metadata-proto-path&~""))
  ?ces <- (code-execution-response-stdout
                              (operation-name ?op) (tree-id ?tree-id)
                              (node-id ?node-id)
                              (stdout ?stdout)
                              (sequence ?current-sequence))
  (not (exists (code-execution-response-stdout
                              (operation-name ?op) (tree-id ?tree-id)
                              (node-id ?node-id)
                              (sequence ?seq&:(< ?seq ?current-sequence)))))
 =>
  (bind ?path
    (proto-path-join ?run-metadata-proto-path "task.execute_code.stdout[*]"))
  (run-metadata-proto-update-field ?path ?stdout ?op)
  (retract ?ces)
)


(defrule behavior-tree-state-proto-update-loop-num-times
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  ?node <- (behavior-tree-node (tree-id ?tree-id) (type LOOP)
                      (loop-num-times ?loop-num-times)
                      (run-metadata-proto-num-iterations
                        ?proto-num-times&~?loop-num-times)
                      (run-metadata-proto-path ?run-metadata-proto-path&~""))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "loop.num_times"))
  (run-metadata-proto-update-field ?path ?loop-num-times ?op)
  (modify ?node (run-metadata-proto-num-iterations ?loop-num-times))
)

(defrule behavior-tree-state-proto-update-retry-num-tries
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  ?node <- (behavior-tree-node (tree-id ?tree-id) (type RETRY)
                      (retry-num-tries ?retry-num-tries)
                      (run-metadata-proto-num-iterations
                        ?proto-num-times&~?retry-num-tries)
                      (run-metadata-proto-path ?run-metadata-proto-path&~""))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "retry.num_tries"))
  (run-metadata-proto-update-field ?path ?retry-num-tries ?op)
  (modify ?node (run-metadata-proto-num-iterations ?retry-num-tries))
)

(defrule behavior-tree-state-proto-update-breakpoint
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  ?node <- (behavior-tree-node (tree-id ?tree-id)
                               (breakpoint-type ?type)
                               (run-metadata-proto-breakpoint-type ?proto-type&~?type)
                               (run-metadata-proto-path ?run-metadata-proto-path&~""))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "decorators.breakpoint"))
  (if (eq ?type NONE)
    then
      (run-metadata-proto-clear-field ?path ?op)
      (run-metadata-proto-clear-unused-decorators ?node ?op)
    else
      (run-metadata-proto-update-field ?path ?type ?op)
  )
  (modify ?node (run-metadata-proto-breakpoint-type ?type))
)

(defrule behavior-tree-state-proto-update-execution-settings
  (declare (salience ?*SALIENCE-HIGHER*))
  (behavior-tree (id ?tree-id) (operation-name ?op))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (execution-mode ?mode)
                               (execution-mode-result-state ?result-state)
                               (run-metadata-proto-path ?run-metadata-proto-path&~""))
  (or (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                          (run-metadata-proto-execution-mode ~?mode))
      (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                          (run-metadata-proto-execution-mode-result-state ~?result-state)))
 =>
  (bind ?path (proto-path-join ?run-metadata-proto-path "decorators.execution_settings"))
  (if (eq ?mode NORMAL)
    then
      (run-metadata-proto-clear-field ?path ?op)
      (run-metadata-proto-clear-unused-decorators ?node ?op)
    else
      (run-metadata-proto-update-field (proto-path-join ?path "mode")
                                         ?mode ?op)
      (bind ?path-result (proto-path-join ?path "disabled_result_state"))
      (if (eq ?result-state AUTO)
        then (run-metadata-proto-clear-field ?path-result ?op)
        else (run-metadata-proto-update-field ?path-result
                                                ?result-state ?op)
      )
  )
  (modify ?node (run-metadata-proto-execution-mode ?mode)
                (run-metadata-proto-execution-mode-result-state ?result-state))
)

(defrule run-metadata-proto-update-scene-id
  (declare (salience ?*SALIENCE-HIGHER*))
  ?op <- (operation-envelope (name ?operation-name)
                      (scene-id ?scene-id)
                      (run-metadata-proto-scene-id ~?scene-id)
  )
 =>
  (run-metadata-proto-update-field "scene_id" ?scene-id ?operation-name)
  (modify ?op (run-metadata-proto-scene-id ?scene-id))
)
