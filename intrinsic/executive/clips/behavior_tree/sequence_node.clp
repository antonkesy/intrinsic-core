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

; Processing rules for a node of type SEQUENCE.

; The SEQUENCE node runs its children in a specific order. It succeeds if all
; children have succeeded, if fails if any of the actions fails.

; ----------------------------------- RULES -----------------------------------
(defrule behavior-tree-sequence-node-start
  "A sequence node directly switches to running when ready"
  (behavior-tree (id ?tree-id) (state RUNNING))
  ?node <- (behavior-tree-node (tree-id ?tree-id) (type SEQUENCE)
                               (state READY))
 =>
  (modify ?node (state RUNNING))
)

(defrule behavior-tree-sequence-node-select-child
  "Select the next child in the sequence"
  (behavior-tree (id ?tree-id) (state RUNNING))
  (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                      (type SEQUENCE) (state RUNNING))
  ?child <- (behavior-tree-node (index ?child-index) (tree-id ?tree-id)
                                (parent-id ?node-id) (state ACCEPTED))
  (not (behavior-tree-node (index ?other-child-index&:(< ?other-child-index
                                                         ?child-index))
                           (tree-id ?tree-id) (parent-id ?node-id)
                           (state ~SUCCEEDED)))
 =>
  (behavior-tree-select-node ?child)
)

(defrule behavior-tree-sequence-node-success
  "A sequence node succeeds if all children have succeeded"
  (behavior-tree (id ?tree-id) (state RUNNING|CANCELING|SUSPENDING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (type SEQUENCE) (state RUNNING|CANCELING))
  (not (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                           (state ~SUCCEEDED)))
 =>
  (behavior-tree-set-node-succeeded ?node)
)

(defrule behavior-tree-sequence-node-failure
  "A child failed, let the sequence node fail"
  ?tree <- (behavior-tree (id ?tree-id) (state RUNNING|CANCELING|SUSPENDING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (type SEQUENCE) (state RUNNING|CANCELING))
  (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id) (state FAILED)
                      (extended-status-proto-id ?context-es))
 =>
  (behavior-tree-set-node-failed ?node EXECUTION "Child failed"
                                 EXTENDED-STATUS-CONTEXT-PROTO-ID ?context-es)
)

(defrule behavior-tree-sequence-node-suspend
  "Suspend node when tree suspends and children are inactive."
  (behavior-tree (id ?tree-id) (state SUSPENDING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (type SEQUENCE) (state ?state&RUNNING))
  ; No node is still busy, or has failed (then the failure rule will fire)
  (not (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                           (state EVALUATING-CONDITION|RUNNING|FAILED)))
  ; All child nodes succeeded, or there are no children: success rule will fire
  (exists (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                              (state ~SUCCEEDED)))
 =>
  (modify ?node (state SUSPENDED) (suspended-from-state ?state)
          (suspend-span-reference-id (tracing-start-suspend-span ?node)))
)

(defrule behavior-tree-sequence-node-resume
  "Resume node if tree becomes RUNNING again."
  (behavior-tree (id ?tree-id) (start-node-id ?start-node-id) (state RUNNING))
  ?node <- (behavior-tree-node (id ?id) (tree-id ?tree-id)
                               (parent-id ?parent-id)
                               (type SEQUENCE) (state SUSPENDED)
                               (suspended-from-state ?resume-state))
  (or (test (eq ?start-node-id ?id))
      (behavior-tree-node (id ?parent-id) (tree-id ?tree-id) (state RUNNING)))
 =>
  (tracing-end-suspend-span ?node)
  (modify ?node (state ?resume-state) (suspended-from-state nil)
          (suspend-span-reference-id ?*TRACING-INVALID-SPAN-ID*))
)

(defrule behavior-tree-sequence-node-cancel
  "Cancels sequence node by canceling the active in-flight child."
  (behavior-tree-node (id ?node-id) (tree-id ?tree-id) (type SEQUENCE)
                      (state CANCELING))
  ?child <- (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                                (state SELECTED|READY|RUNNING|SUSPENDED))
 =>
  (modify ?child (state CANCELING)
          (canceling-span-reference-id (tracing-start-canceling-span ?child)))
)

(defrule behavior-tree-sequence-node-canceled
  "Cancels the node if all children are waiting or finished."
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id) (type SEQUENCE)
           (state CANCELING))
  ; All children are either accepted, succeeded, or canceled
  (not (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                           (state ~ACCEPTED&~SUCCEEDED&~CANCELED)))

  ; At least one child is not finished or has been cancelled
  (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                      (state ACCEPTED|CANCELED))
 =>
  (behavior-tree-set-node-canceled ?node)
)

(defrule behavior-tree-sequence-node-fail-on-child-canceled
  "Node canceled but it wasn't expected to"
  ?tree <- (behavior-tree (id ?tree-id))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id) (type SEQUENCE)
                               (state ?state&~ACCEPTED&~CANCELING&~FAILED&~CANCELED))
  (behavior-tree-node (id ?child-node-id) (tree-id ?tree-id) (parent-id ?node-id)
                      (state CANCELED))
 =>
  (bind ?es-proto (extended-status-create 31800))
  (bind ?message (str-cat "Child canceled unexpectedly - from child node "
    ?child-node-id "while the sequence node is in state " ?state))
  (extended-status-set-message ?es-proto USER ?message)
  (extended-status-set-related-to ?es-proto ?tree ?node)

  (behavior-tree-set-node-failed ?node EXECUTION ?message
        EXTENDED-STATUS-CONTEXT-PROTO-ID ?es-proto)
  (pb-remove ?es-proto)
)

(defrule behavior-tree-sequence-node-predict-start
  "A sequence node directly switches the first child to selected for predict."
  (behavior-tree (id ?tree-id) (predict-generation-id ?predict-generation-id)
                 (state ACCEPTED|RUNNING))
  ?node <- (behavior-tree-node (tree-id ?tree-id) (type SEQUENCE)
                               (prediction-world-id ?prediction-world-id)
                               (predict-state SELECTED))
  ?first-child <- (behavior-tree-node (index 1) (tree-id ?tree-id)
                                      (parent-id ?node-id)
                                      (predict-state IDLE))
 =>
  (modify ?node (predict-state PREDICTING)
                (predict-generation-id ?predict-generation-id))
  (modify ?first-child (predict-state SELECTED)
                       (predict-generation-id ?predict-generation-id)
                       (prediction-world-id
                         (world-clone ?prediction-world-id "predict_sequence"
                           ?*TRACING-INVALID-SPAN-ID*)))
)

(defrule behavior-tree-sequence-node-predict-propagation
  "A sequence node directly switches from selected to predicting if one of the
   children is already predicted."
  (behavior-tree (id ?tree-id) (state ACCEPTED|RUNNING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id) (type SEQUENCE)
                               (predict-generation-id ?node-generation-id)
                               (predict-state SELECTED))
  (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                      (predict-state SUCCEEDED))
 =>
  (modify ?node (predict-state PREDICTING)
                (predict-generation-id ?node-generation-id))
)

(defrule behavior-tree-sequence-node-select-prediction-child
  "Select the next child in the sequence for prediction"
  ?tree <- (behavior-tree (id ?tree-id) (state ACCEPTED|RUNNING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (predict-state FAILED|PREDICTING)
                               (type SEQUENCE))
  ?child <- (behavior-tree-node (id ?child-node-id) (tree-id ?tree-id)
                                (index ?child-index) (parent-id ?node-id)
                                (predict-state IDLE))
  (behavior-tree-node (id ?previous-child-node-id)
                      (tree-id ?tree-id)
                      (parent-id ?node-id)
                      (index ?previous-child-index)
                      (prediction-world-id ?prediction-world-id)
                      (prediction-proto-id ?prediction-proto-id)
                      (predict-state SUCCEEDED))
  (not (behavior-tree-node (index ?other-child-index&:(< ?other-child-index
                                                         ?child-index))
                           (tree-id ?tree-id) (parent-id ?node-id)
                           (predict-state ~SUCCEEDED)))
  (test (eq ?child-index (+ ?previous-child-index 1)))
 =>
  (printout debug "Selecting node (" ?tree-id ", " ?child-node-id
                  ") for prediction with previous node (" ?tree-id ", "
                  ?previous-child-node-id ") as baseline" crlf)

  ; Start with the world from the previous child and add the prediction to it.
  ; This will be used as the prediction world for the current child
  (bind ?predict-world (world-clone-and-apply-prediction ?prediction-world-id
                                                         ?prediction-proto-id))

  (modify ?child (predict-state SELECTED)
                 (prediction-world-id ?predict-world))
  (modify ?node (predict-state PREDICTING))
)

(defrule behavior-tree-sequence-node-predict-success
  "A sequence node succeeds prediction if all children have succeeded"
  (behavior-tree (id ?tree-id) (state ACCEPTED|RUNNING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (type SEQUENCE) (predict-state PREDICTING))
  (not (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                           (predict-state ~SUCCEEDED)))
 =>
  ; Creates an empty starting prediction with a probability of 1.0
  (bind ?prediction-results (pb-create "intrinsic_proto.skills.Prediction"))
  (pb-set-field ?prediction-results "probability" 1.0)

  ; Combine the predictions from the children into ?prediction-results
  (bind ?children (behavior-tree-sorted-children ?node-id ?tree-id))
  (foreach ?child ?children
    (bind ?child-prediction-results (fact-slot-value ?child prediction-proto-id))
    (bind ?prediction-results (world-merge-predictions ?prediction-results
                                                       ?child-prediction-results))
  )

  (modify ?node (predict-state SUCCEEDED)
                (prediction-proto-id ?prediction-results))
)

(defrule behavior-tree-sequence-node-predict-failure
  "A child failed prediction, let the sequence node fail prediction"
  (behavior-tree (id ?tree-id) (state ACCEPTED|RUNNING))
  ?node <- (behavior-tree-node (id ?node-id) (tree-id ?tree-id)
                               (type SEQUENCE) (predict-state PREDICTING))
  (behavior-tree-node (tree-id ?tree-id) (parent-id ?node-id)
                      (predict-state FAILED))
 =>
  (modify ?node (predict-state FAILED))
)
