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

; Action prediction through skills

; This file requires skill_info.clp and predict_action_state.clp to be loaded

; ----------------------------------- RULES -----------------------------------

(defrule action-prediction-skill-start
  "Start skill prediction after corresponding action was selected."
  (declare (salience ?*SALIENCE-ACTION-PREDICTION*))
  (executive-state (session-log-id ?session-log-id))
  (behavior-tree (id ?bt-id) (plan-id ?plan-id) (operation-name ?operation-name)
                 (log-id ?bt-log-id&~0))
  ?af <- (predict-action (plan-id ?plan-id) (id ?id) (uid ?uid)
                         (state SELECTED) (skill-id ?skill-id)
                         (prediction-world-id ?prediction-world-id)
                         (behavior-call-proto-id ?behavior-call-proto)
                         (log-id ?action-log-id&~0)
                         (span-reference-id ?parent-span-id))
  (skill-info (skill-id ?skill-id)
              (parameter-descriptor-pool-id ?parameter-descriptor-pool-id))
 =>
  (printout debug "Starting skill prediction for action "
                  (predict-action-tostring ?plan-id ?id)
                  " (skill " ?skill-id ")" crlf)

  (bind ?parent-log-id (log-find-parent-log-id ?bt-id))
  (bind ?log-context-proto
    (log-context-create ?session-log-id ?bt-log-id ?action-log-id
                        ?parent-log-id ?operation-name))

  (skill-preemptive-predict-async ?uid
                                  ?prediction-world-id
                                  ?behavior-call-proto
                                  ?log-context-proto
                                  ?parent-span-id
                                  ?parameter-descriptor-pool-id)

  (bind ?deployment-id (pb-get-field ?log-context-proto "deployment_id"))
  (if (neq ?deployment-id MISSING-FIELD) then
    (span-add-attribute ?parent-span-id "deployment_id" ?deployment-id)
  )

  (pb-remove ?log-context-proto)
  (modify ?af (state PREDICTING) (prediction-start-time (now)))
)

(defrule action-prediction-skill-succeeded
  "Skill prediction by ClipsSkillDispatcher has completed."
  ?af <- (predict-action (id ?id) (plan-id ?plan-id) (uid ?uid)
                         (skill-id ?skill-id) (state FAILED|PREDICTING))
  ?sf <- (skill-status (action-id ?uid) (predict-state SUCCEEDED)
                       (prediction-proto-id ?prediction-proto-id))
  (skill-info (skill-id ?skill-id))
 =>
  (printout debug "Skill preemptive prediction for action "
                  (predict-action-tostring ?plan-id ?id)
                  " (skill " ?skill-id ") has succeeded" crlf)

  (modify ?af (state SUCCEEDED)
              (prediction-end-time (now))
              (prediction-proto-id ?prediction-proto-id))
  ; clear the skill-status
  (retract ?sf)
)

(defrule action-prediction-skill-failed
  "Observe skill status by ClipsSkillDispatcher for prediction failure"
  ?af <- (predict-action (id ?id) (plan-id ?plan-id) (uid ?uid)
                         (skill-id ?skill-id) (state PREDICTING))
  ?sf <- (skill-status (action-id ?uid) (predict-state FAILED)
                       (message ?message))
  (skill-info (skill-id ?skill-id))
 =>
  (printout debug "Skill preemptive prediction for action "
                  (predict-action-tostring ?plan-id ?id)
                  " (skill " ?skill-id ") has failed with message: "
                  ?message crlf)

  (modify ?af (state FAILED)
              (prediction-end-time (now)))
  ; clear the skill-status
  (retract ?sf)
)
