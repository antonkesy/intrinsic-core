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

; Plan prediction action representation and handling

; --------------------------------- TEMPLATES ---------------------------------

(deftemplate predict-action
  ; One unit of the prediction which is performed while following a plan.
  ; This is a CLIPS executive internal id to identify this fact within a plan.
  (slot id (type INTEGER))

  ; The plan this action belongs to.
  (slot plan-id (type SYMBOL))

  ; Skill id, this triggers the respective prediction engine.
  ; For executing skills, this refers to intrinsic_proto.skills.Skill.id.
  ; For no-op actions, a no-op action with this skill-id must exist.
  (slot skill-id (type STRING))

  ; Names and values for predict parameters
  (multislot param-names (type SYMBOL))
  ; TODO(timdn): reconcile with actual skill parameters
  (multislot param-values)

  ; This is associated with a globally unique ID in the scope of the current
  ; executive lifetime. Leave to automatically based on plan and action IDs.
  ; If setting a custom UID you must guarantee uniqueness by yourself.
  (slot uid (type SYMBOL) (default AUTOMATIC))

  ; Proto ID of the associated behavior call proto.
  (slot behavior-call-proto-id (type INTEGER))

  ; This status's LogID for related log-context generation
  ; 0 means that the predict status has not yet been logged.
  (slot log-id (type INTEGER))
  (slot logged-completion (type SYMBOL) (allowed-values FALSE TRUE))

  ; IDLE:       The status has been formulated and is awaiting selection. If and
  ;             when it is selected depends on the specific semantics, e.g., for
  ;             a behavior tree that task node must have been selected for
  ;             prediction. This is the initial state for all statuses.
  ; SELECTED:   The action has been selected for prediction, i.e., it is
  ;             expected to be processed shortly/next. It has a
  ;             valid prediction-world-id for calling predict.
  ; PREDICTING: The action is currently being predicted.
  ; SUCCEEDED:  Prediction of the node has terminated successfully.
  ; FAILED:     Prediction failed.
  (slot state (type SYMBOL) (default IDLE)
        (allowed-values IDLE SELECTED PREDICTING SUCCEEDED FAILED))

  ; An optional error message if the skill prediction failed
  (slot error-message (type STRING))

  ; This is the message ID for the associated Prediction proto.
  (slot prediction-proto-id (type INTEGER))

  (multislot prediction-select-time (type INTEGER) (cardinality 2 2))
  (multislot prediction-start-time (type INTEGER) (cardinality 2 2))
  (multislot prediction-end-time (type INTEGER) (cardinality 2 2))

  ; The world to use for preemptive prediction calls
  (slot prediction-world-id (type STRING))

  ; The span-reference-id for tracing this action.
  (slot span-reference-id (type INTEGER))
)

; --------------------------------- FUNCTIONS ---------------------------------

; Create a new number of new unique action IDs for a given plan.
; This iterates through all plan-actions associated to the respective plan and
; returns a sequence of new IDs larger than the maximum encountered ID of the
; given length. This can be used to inject new actions into a plan.
; If no plan for the given ID exists, returns nil.
(deffunction plan-next-predict-action-ids (?plan-id ?num-ids)
  (bind ?max-id 0)
  (bind ?new-ids (create$))
  ; Fact set query, runs over all plan-actions with the wanted ?plan-id.
  ; Finds the maximum assigned action ID in a plan.
  (do-for-all-facts ((?pa predict-action)) (eq ?pa:plan-id ?plan-id)
    (if (> ?pa:id ?max-id) then (bind ?max-id ?pa:id))
  )
  ; Generate a sequence of new actions, starting at the max ID + 1 and for
  ; the number of requested IDs.
  (loop-for-count (?i ?num-ids)
    (bind ?new-ids (append$ ?new-ids (+ ?max-id ?i))))
  (return ?new-ids)
)

; Create a new unique action ID for a given plan.
; This iterates through all plan-actions associated to the respective plan and
; returns a new ID larger than the maximum encountered ID. This can be used
; to inject a new action into a plan.
; If no no plan for the given ID exists, returns nil.
(deffunction plan-next-predict-action-id (?plan-id)
  (bind ?new-ids (plan-next-predict-action-ids ?plan-id 1))
  (return (nth$ 1 ?new-ids))
)

(deffunction predict-action-remove (?uid)
  (do-for-fact ((?pa predict-action)) (eq ?pa:uid ?uid)
    (if (neq ?pa:behavior-call-proto-id 0) then
      (pb-remove ?pa:behavior-call-proto-id))
    (retract ?pa)
  )
)

; Convert a given action to a nicely printable string.
(deffunction predict-action-tostring (?plan-id ?predict-action-id)
  (do-for-fact ((?pa predict-action)) (and (eq ?pa:plan-id ?plan-id)
                                           (eq ?pa:id ?predict-action-id))
    (return (str-cat "(" ?pa:skill-id
                     (if (non-empty$ ?pa:param-values) then " "  else "")
                     (implode$ ?pa:param-values) ")"))
  )
)

; Generate a unique ID symbol for an action in a plan.
(deffunction predict-action-uid (?plan-id ?predict-action-id)
  (return (sym-cat ?plan-id "-" ?predict-action-id "-" (gensym*)))
)

; ----------------------------------- RULES -----------------------------------
